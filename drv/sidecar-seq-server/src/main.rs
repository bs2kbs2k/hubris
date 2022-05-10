// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Server for managing the Sidecar sequencing process.

#![no_std]
#![no_main]

use drv_fpga_api::{Fpga, FpgaError};
use drv_i2c_api::{I2cDevice, ResponseCode};
use drv_sidecar_mainboard_controller_api::tofino2::{
    Tofino2Error, Tofino2State, Tofino2Vid, Sequencer,
};
use drv_sidecar_mainboard_controller_api::MainboardController;
use drv_sidecar_seq_api::{PowerState, SeqError};
use drv_stm32xx_sys_api::{self as sys_api, Sys};
use idol_runtime::{NotificationHandler, RequestError};
use ringbuf::*;
use userlib::*;

task_slot!(SYS, sys);
task_slot!(I2C, i2c_driver);
task_slot!(FPGA, fpga);

mod payload;

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));
use i2c_config::devices;

#[derive(Copy, Clone, PartialEq)]
enum Trace {
    None,
    FpgaAwaitingBitstream,
    FpgaBitstreamLoadError(u32),
    FpgaBitstreamLoadComplete,
    ValidControllerIdent(u32),
    InvalidControllerIdent(u32),
    LoadClockConfig,
    ClockConfigError(usize, ResponseCode),
    InitComplete,
    TofinoSeqState(u8),
    SetTofinoEn(bool),
    SampledVid(u8),
    SetVddCoreVout(userlib::units::Volts),
    GetState,
    SetState(PowerState, PowerState),
    TofinoSequencerError(Tofino2Error),
    TofinoPowerStateChange(Tofino2State, PowerState),
    TofinoVidAck,
}
ringbuf!(Trace, 64, Trace::None);

const TIMER_NOTIFICATION_MASK: u32 = 1 << 0;
const FPGA_NOTIFICATION_MASK: u32 = 1 << 1;
const TIMER_INTERVAL: u64 = 1000;

struct ServerImpl {
    mainboard_controller: MainboardController,
    tofino_sequencer: Sequencer,
    clockgen: I2cDevice,
    deadline: u64,
    clock_config_loaded: bool,
}

impl ServerImpl {
    fn apply_vid(&mut self, vid: Tofino2Vid) -> Result<(), SeqError> {
        use userlib::units::Volts;

        fn set_vout(value: Volts) -> Result<(), SeqError> {
            use drv_i2c_devices::raa229618::Raa229618;
            let i2c = I2C.get_task_id();

            let (device, rail) = i2c_config::pmbus::v0p8_tf2_vdd_core(i2c);
            let mut vddcore = Raa229618::new(&device, rail);

            /*
            vddcore
                .set_vout(value)
                .map_err(|_| SeqError::SetVddCoreVoutFailed)?;
            */

            ringbuf_entry!(Trace::SetVddCoreVout(value));
            Ok(())
        }

        match vid {
            Tofino2Vid::V0P922 => set_vout(Volts(0.922)),
            Tofino2Vid::V0P893 => set_vout(Volts(0.893)),
            Tofino2Vid::V0P867 => set_vout(Volts(0.867)),
            Tofino2Vid::V0P847 => set_vout(Volts(0.847)),
            Tofino2Vid::V0P831 => set_vout(Volts(0.831)),
            Tofino2Vid::V0P815 => set_vout(Volts(0.815)),
            Tofino2Vid::V0P790 => set_vout(Volts(0.790)),
            Tofino2Vid::V0P759 => set_vout(Volts(0.759)),
        }
    }

    fn power_up_tofino(&mut self) -> Result<(), SeqError> {
        // Initiate the power up sequence.
        self.tofino_sequencer.set_enable(true)?;

        // Wait for the VID to become valid, retrying if needed.
        for i in 1..4 {
            // Sleep first since there is a delay between the sequencer
            // receiving the EN bit and the VID being valid.
            hl::sleep_for(i * 25);

            let maybe_vid = self.tofino_sequencer.vid().map_err(|e| {
                if let FpgaError::InvalidValue = e {
                    SeqError::InvalidTofinoVid
                } else {
                    SeqError::FpgaError
                }
            })?;

            // Set Vout accordingy to the VID and acknowledge the change to the
            // sequencer.
            if let Some(vid) = maybe_vid {
                self.apply_vid(vid)?;
                self.tofino_sequencer.ack_vid()?;
                ringbuf_entry!(Trace::TofinoVidAck);
                return Ok(());
            }
        }

        Err(SeqError::SequencerTimeout)
    }

    fn power_down_tofino(&mut self) -> Result<(), SeqError> {
        Ok(self
            .tofino_sequencer
            .set_enable(false)
            .map_err(|_| SeqError::SequencerError)?)
    }

    fn set_tofino_state(
        &mut self,
        desired_state: PowerState,
    ) -> Result<(), SeqError> {
        let seq_state = self
            .tofino_sequencer
            .state()
            .map_err(|_| SeqError::SequencerError)?;
        let seq_error = self
            .tofino_sequencer
            .error()
            .map_err(|_| SeqError::SequencerError)?;

        if seq_error != Tofino2Error::None {
            ringbuf_entry!(Trace::TofinoSequencerError(seq_error));
            Err(SeqError::SequencerError)
        } else {
            match (seq_state, desired_state) {
                (Tofino2State::A2, PowerState::A0) => {
                    ringbuf_entry!(Trace::TofinoPowerStateChange(
                        seq_state,
                        desired_state
                    ));
                    self.power_up_tofino()
                }
                (_, PowerState::A2) => {
                    ringbuf_entry!(Trace::TofinoPowerStateChange(
                        seq_state,
                        desired_state
                    ));
                    self.power_down_tofino()
                }
                _ => Err(SeqError::IllegalTransition),
            }
        }
    }

    fn load_clock_config(&mut self) -> Result<(), SeqError> {
        ringbuf_entry!(Trace::LoadClockConfig);

        let mut packet = 0;

        /*
        payload::idt8a3xxxx_payload(|buf| {
            match self.clockgen.write(buf) {
                Err(err) => {
                    ringbuf_entry!(Trace::ClockConfigError(packet, err));
                    Err(SeqError::ClockConfigFailed)
                }

                Ok(_) => {
                    packet += 1;
                    Ok(())
                }
            }
        })?;
        */

        Ok(())
    }
}

impl idl::InOrderSequencerImpl for ServerImpl {
    fn tofino_seq_state(
        &mut self,
        _: &RecvMessage,
    ) -> Result<Tofino2State, RequestError<SeqError>> {
        Ok(self.tofino_sequencer.state().map_err(SeqError::from)?)
    }

    fn tofino_seq_error(
        &mut self,
        _: &RecvMessage,
    ) -> Result<Tofino2Error, RequestError<SeqError>> {
        Ok(self.tofino_sequencer.error().map_err(SeqError::from)?)
    }

    fn tofino_power_status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<SeqError>> {
        Ok(self
            .tofino_sequencer
            .power_status()
            .map_err(SeqError::from)?)
    }

    fn set_tofino_state(
        &mut self,
        _: &RecvMessage,
        state: PowerState,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self.set_tofino_state(state)?)
    }

    fn load_clock_config(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<SeqError>> {
        self.clock_config_loaded = true;
        Ok(())
    }

    fn is_clock_config_loaded(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u8, RequestError<SeqError>> {
        Ok(self.clock_config_loaded as u8)
    }
}

impl NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        TIMER_NOTIFICATION_MASK
    }

    fn handle_notification(&mut self, _bits: u32) {
        self.deadline += TIMER_INTERVAL;
        sys_set_timer(Some(self.deadline), TIMER_NOTIFICATION_MASK);
    }
}

#[export_name = "main"]
fn main() -> ! {
    let mut buffer = [0; idl::INCOMING_SIZE];
    let deadline = sys_get_timer().now;

    //
    // This will put our timer in the past, and should immediately kick us.
    //
    sys_set_timer(Some(deadline), TIMER_NOTIFICATION_MASK);

    let mut server = ServerImpl {
        clockgen: devices::idt8a34001(I2C.get_task_id())[0],
        mainboard_controller: MainboardController::new(FPGA.get_task_id()),
        tofino_sequencer: Sequencer::new(FPGA.get_task_id()),
        deadline,
        clock_config_loaded: false,
    };

    server
        .mainboard_controller
        .await_fpga_ready_for_bitstream(25)
        .unwrap();

    if let Err(e) = server.mainboard_controller.load_bitstream() {
        ringbuf_entry!(Trace::FpgaBitstreamLoadError(
            u32::try_from(e).unwrap()
        ));
        panic!();
    }

    ringbuf_entry!(Trace::FpgaBitstreamLoadComplete);

    let ident = server.mainboard_controller.ident().unwrap();
    if !server.mainboard_controller.ident_valid(ident) {
        ringbuf_entry!(Trace::InvalidControllerIdent(ident));
        panic!();
    }
    ringbuf_entry!(Trace::ValidControllerIdent(ident));

    if let Err(e) = server.load_clock_config() {
        panic!();
    }

    ringbuf_entry!(Trace::InitComplete);

    loop {
        idol_runtime::dispatch_n(&mut buffer, &mut server);
    }
}

cfg_if::cfg_if! {
    if #[cfg(target_board = "sidecar-1")] {
        const BOARD_CONTROLLER_FPGA: u8 = 0;
        const BOARD_CONTROLLER_APP: u8 = 1;
    } else if #[cfg(target_board = "gimletlet-2")] {
        const BOARD_CONTROLLER_FPGA: u8 = 0;
        const BOARD_CONTROLLER_APP: u8 = 0;
    } else {
        compiler_error!("unsupported target board");
    }
}

mod idl {
    use super::{PowerState, SeqError, Tofino2Error, Tofino2State};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
