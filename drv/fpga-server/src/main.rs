// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Server for managing FPGA devices.

#![no_std]
#![no_main]

use ringbuf::*;
use userlib::*;
use zerocopy::{byteorder, AsBytes, Unaligned, U16};

use drv_fpga_api::*;
use drv_fpga_devices::{ecp5, ecp5::Ecp5, ecp5_spi::Ecp5UsingSpi, Fpga};
use drv_spi_api::{self as spi_api, Spi, SpiDevice};
use drv_stm32xx_sys_api::{self as sys_api, Sys};
use idol_runtime::{ClientError, Leased, LenLimit, R, W};

task_slot!(SYS, sys);
task_slot!(SPI, spi_driver);

#[derive(Copy, Clone, Debug, PartialEq)]
enum Trace {
    None,
    DeviceId(u32),
    StartBitstreamLoad(BitstreamType),
    ContinueBitstreamLoad(usize),
    FinishBitstreamLoad(usize),
    Locked(TaskId),
    Released(TaskId),
    ChunkLen(usize),
}
ringbuf!(Trace, 64, Trace::None);

#[export_name = "main"]
fn main() -> ! {
    let sys = Sys::from(SYS.get_task_id());
    let spi = Spi::from(SPI.get_task_id()).device(0);

    cfg_if::cfg_if! {
        if #[cfg(target_board = "sidecar-1")] {
            let driver = Ecp5UsingSpi {
                sys,
                spi,
                done: sys_api::Port::J.pin(15),
                init_n: sys_api::Port::J.pin(12),
                program_n: sys_api::Port::J.pin(13),
                design_reset_n: sys_api::Port::J.pin(14),
            };
        } else if #[cfg(target_board = "gimletlet-2")] {
            let driver = Ecp5UsingSpi {
                sys,
                spi,
                done: sys_api::Port::E.pin(15),
                init_n: sys_api::Port::D.pin(12),
                program_n: sys_api::Port::B.pin(10),
                design_reset_n: sys_api::Port::D.pin(11),
            };
        } else {
            compile_error!("Board is not supported by the task/fpga");
        }
    }
    driver.configure_gpio();

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl {
        lock_holder: None,
        device: Ecp5::new(driver),
        device_reset_ticks: ecp5::DEVICE_RESET_DURATION,
        application: Spi::from(SPI.get_task_id()).device(1),
        application_reset_ticks: ecp5::APPLICATION_RESET_DURATION,
        buffer: [0u8; 128],
        bitstream_loader: BitstreamLoader::None,
    };

    if let Ok(DeviceState::AwaitingBitstream) = server.device.device_state() {
        ringbuf_entry!(Trace::DeviceId(server.device.device_id().unwrap()));
    }

    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

enum BitstreamLoader {
    None,
    UncompressedLoadInprogress(usize),
    CompressedLoadInProgress(gnarle::Decompressor, usize),
}

struct ServerImpl<FpgaT: Fpga> {
    lock_holder: Option<userlib::TaskId>,
    device: FpgaT,
    device_reset_ticks: u64,
    application: SpiDevice,
    application_reset_ticks: u64,
    buffer: [u8; 128],
    bitstream_loader: BitstreamLoader,
}

type RequestError = idol_runtime::RequestError<FpgaError>;
type ReadDataLease = LenLimit<Leased<R, [u8]>, 128>;
type WriteDataLease = LenLimit<Leased<W, [u8]>, 128>;

impl<FpgaT: Fpga> idl::InOrderFpgaImpl for ServerImpl<FpgaT> {
    fn recv_source(&self) -> Option<userlib::TaskId> {
        self.lock_holder
    }

    fn closed_recv_fail(&mut self) {
        // Welp, someone had asked us to lock and then died. Release the
        // lock.
        self.lock_holder = None;
    }

    fn lock(&mut self, msg: &userlib::RecvMessage) -> Result<(), RequestError> {
        if let Some(task) = self.lock_holder {
            // The fact that we received this message _at all_ means
            // that the sender matched our closed receive, but just
            // in case we have a server logic bug, let's check.
            assert!(task == msg.sender);
        }

        self.lock_holder = Some(msg.sender);
        ringbuf_entry!(Trace::Locked(msg.sender));
        Ok(())
    }

    fn release(
        &mut self,
        msg: &userlib::RecvMessage,
    ) -> Result<(), RequestError> {
        if let Some(task) = self.lock_holder {
            // The fact that we received this message _at all_ means
            // that the sender matched our closed receive, but just
            // in case we have a server logic bug, let's check.
            assert!(task == msg.sender);

            self.lock_holder = None;
            ringbuf_entry!(Trace::Released(msg.sender));
            Ok(())
        } else {
            Err(FpgaError::NotLocked.into())
        }
    }

    fn device_enabled(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError> {
        Ok(self.device.device_enabled()?)
    }

    fn set_device_enabled(
        &mut self,
        _: &RecvMessage,
        enabled: bool,
    ) -> Result<(), RequestError> {
        Ok(self.device.set_device_enabled(enabled)?)
    }

    fn reset_device(&mut self, _: &RecvMessage) -> Result<(), RequestError> {
        Ok(self.device.reset_device(self.device_reset_ticks)?)
    }

    fn device_state(
        &mut self,
        _: &RecvMessage,
    ) -> Result<DeviceState, RequestError> {
        Ok(self.device.device_state()?)
    }

    fn device_id(&mut self, _: &RecvMessage) -> Result<u32, RequestError> {
        Ok(self.device.device_id()?)
    }

    fn application_enabled(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError> {
        Ok(self.device.application_enabled()?)
    }

    fn set_application_enabled(
        &mut self,
        _: &RecvMessage,
        enabled: bool,
    ) -> Result<(), RequestError> {
        Ok(self.device.set_application_enabled(enabled)?)
    }

    fn reset_application(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError> {
        Ok(self
            .device
            .reset_application(self.application_reset_ticks)?)
    }

    fn start_bitstream_load(
        &mut self,
        _: &RecvMessage,
        bitstream_type: BitstreamType,
    ) -> Result<(), RequestError> {
        self.bitstream_loader = match bitstream_type {
            BitstreamType::Uncompressed => {
                BitstreamLoader::UncompressedLoadInprogress(0)
            }
            BitstreamType::Compressed => {
                BitstreamLoader::CompressedLoadInProgress(
                    gnarle::Decompressor::default(), 0
                )
            }
        };

        self.device.start_bitstream_load()?;
        ringbuf_entry!(Trace::StartBitstreamLoad(bitstream_type));
        Ok(())
    }

    fn continue_bitstream_load(
        &mut self,
        _: &RecvMessage,
        data: LenLimit<Leased<R, [u8]>, 128>,
    ) -> Result<(), RequestError> {
        data.read_range(0..data.len(), &mut self.buffer[..data.len()])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        let mut chunk = &self.buffer[..data.len()];
        let mut decompress_buffer = [0; 1024];

        match &mut self.bitstream_loader {
            BitstreamLoader::None => panic!(),
            BitstreamLoader::UncompressedLoadInprogress(bitstream_len) => {
                self.device.continue_bitstream_load(chunk)?;
                *bitstream_len += chunk.len();
            }
            BitstreamLoader::CompressedLoadInProgress(decompressor, bitstream_len) => {
                while !chunk.is_empty() {
                    ringbuf_entry!(Trace::ChunkLen(chunk.len()));

                    let decompressed_chunk = gnarle::decompress(
                        decompressor,
                        &mut chunk,
                        &mut decompress_buffer,
                    );

                    // The compressor may have encountered a partial run at the
                    // end of the `chunk`, in which case `deocompressed_chunk`
                    // will be empty since more data is needed before output is
                    // generated.
                    if decompressed_chunk.len() > 0 {
                        self.device.continue_bitstream_load(decompressed_chunk)?;
                        *bitstream_len += decompressed_chunk.len();
                    }
                }
            }
        }

        //ringbuf_entry!(Trace::ContinueBitstreamLoad(data.len()));
        Ok(())
    }

    fn finish_bitstream_load(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError> {
        match &mut self.bitstream_loader {
            BitstreamLoader::None => panic!(),
            BitstreamLoader::UncompressedLoadInprogress(bitstream_len) => {
                ringbuf_entry!(Trace::FinishBitstreamLoad(*bitstream_len));
                self.device
                    .finish_bitstream_load(self.application_reset_ticks)?;
            }
            BitstreamLoader::CompressedLoadInProgress(_, bitstream_len) => {
                ringbuf_entry!(Trace::FinishBitstreamLoad(*bitstream_len));
                self.device
                    .finish_bitstream_load(self.application_reset_ticks)?;
            }
        }

        self.bitstream_loader = BitstreamLoader::None;
        Ok(())
    }

    fn application_read_raw(
        &mut self,
        _: &userlib::RecvMessage,
        addr: u16,
        data: WriteDataLease,
    ) -> Result<(), RequestError> {
        let header = ApplicationRequestHeader {
            cmd: 0x1,
            addr: U16::new(addr),
        };

        self.application
            .lock(spi_api::CsState::Asserted)
            .map_err(FpgaError::from)?;
        self.application
            .write(header.as_bytes())
            .map_err(FpgaError::from)?;
        self.application
            .read(&mut self.buffer[..data.len()])
            .map_err(FpgaError::from)?;
        self.application.release().map_err(FpgaError::from)?;

        data.write_range(0..data.len(), &self.buffer[..data.len()])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        Ok(())
    }

    fn application_write_raw(
        &mut self,
        _: &userlib::RecvMessage,
        op: WriteOp,
        addr: u16,
        data: ReadDataLease,
    ) -> Result<(), RequestError> {
        data.read_range(0..data.len(), &mut self.buffer[..data.len()])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        let header = ApplicationRequestHeader {
            cmd: u8::from(op),
            addr: U16::new(addr),
        };

        self.application
            .lock(spi_api::CsState::Asserted)
            .map_err(FpgaError::from)?;
        self.application
            .write(header.as_bytes())
            .map_err(FpgaError::from)?;
        self.application
            .write(&self.buffer[..data.len()])
            .map_err(FpgaError::from)?;
        self.application.release().map_err(FpgaError::from)?;

        Ok(())
    }
}

#[derive(AsBytes, Unaligned)]
#[repr(C)]
struct ApplicationRequestHeader {
    cmd: u8,
    addr: U16<byteorder::BigEndian>,
}

mod idl {
    use super::{BitstreamType, DeviceState, FpgaError, WriteOp};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
