// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

use drv_fpga_api::{
    BitstreamType, DeviceState, Fpga, FpgaApplication, FpgaError, idl::Fpga as FpgaRaw,
};
use userlib::hl::sleep_for;

static COMPRESSED_BITSTREAM: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/ecp5.bin.rle"));

include!(concat!(env!("OUT_DIR"), "/sidecar_mainboard_controller.rs"));

pub mod tofino2;

pub struct MainboardController {
    fpga: Fpga,
    application: FpgaApplication,
    fpga_raw: FpgaRaw,
}

impl MainboardController {
    pub const EXPECTED_IDENT: u32 = 0x1DE_AA55;

    pub fn new(task_id: userlib::TaskId) -> Self {
        Self {
            fpga: Fpga::new(task_id),
            application: FpgaApplication::new(task_id),
            fpga_raw: FpgaRaw::from(task_id),
        }
    }

    pub fn await_fpga_ready(
        &mut self,
        sleep_ticks: u64,
    ) -> Result<(), FpgaError> {
        while match self.fpga.state()? {
            DeviceState::AwaitingBitstream => false,
            DeviceState::RunningApplication => {
                self.fpga.reset()?;
                true
            }
            _ => true,
        } {
            sleep_for(sleep_ticks);
        }
        Ok(())
    }

    pub fn reset_fpga(&mut self) -> Result<(), FpgaError> {
        self.fpga.reset()
    }

    pub fn load_bitstream(&mut self) -> Result<(), FpgaError> {
        /*
        let mut bitstream =
            self.fpga.start_bitstream_load(BitstreamType::Compressed)?;

        for chunk in COMPRESSED_BITSTREAM[..].chunks(128) {
            bitstream.continue_load(chunk)?;
        }

        bitstream.finish_load()
        */

        self.fpga_raw.start_bitstream_load(BitstreamType::Compressed)?;

        for chunk in COMPRESSED_BITSTREAM[..].chunks(128) {
            self.fpga_raw.continue_bitstream_load(chunk)?;
        }

        self.fpga_raw.finish_bitstream_load()
    }

    /// Reads the IDENT0:3 registers as a big-endian 32-bit integer.
    pub fn ident(&self) -> Result<u32, FpgaError> {
        Ok(u32::from_be(self.application.read(Addr::ID0)?))
    }

    /// Check for a valid identifier
    pub fn ident_valid(&self, ident: u32) -> bool {
        ident == Self::EXPECTED_IDENT
    }
}
