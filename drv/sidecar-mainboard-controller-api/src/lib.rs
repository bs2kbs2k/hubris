// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

use drv_fpga_api::{BitstreamType, DeviceState, Fpga, FpgaError};
use userlib::hl::sleep_for;

static COMPRESSED_BITSTREAM: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/ecp5.bin.rle"));

include!(concat!(env!("OUT_DIR"), "/sidecar_mainboard_controller.rs"));

pub mod tofino2;

pub struct MainboardController {
    fpga: Fpga,
}

impl MainboardController {
    pub const EXPECTED_IDENT: u32 = 0x1DE_AA55;

    pub fn new(fpga: Fpga) -> Self {
        Self { fpga }
    }

    pub fn await_fpga_ready(
        &mut self,
        sleep_ticks: u64,
    ) -> Result<(), FpgaError> {
        while match self.fpga.device_state()? {
            DeviceState::AwaitingBitstream => false,
            DeviceState::RunningApplication => {
                self.fpga.reset_device()?;
                true
            }
            _ => true,
        } {
            sleep_for(sleep_ticks);
        }
        Ok(())
    }

    pub fn load_bitstream(&mut self) -> Result<(), FpgaError> {
        self.fpga.start_bitstream_load(BitstreamType::Compressed)?;

        for chunk in COMPRESSED_BITSTREAM[..].chunks(128) {
            self.fpga.continue_bitstream_load(chunk)?;
        }

        self.fpga.finish_bitstream_load()
    }

    /// Reads the IDENT0:3 registers as a big-endian 32-bit integer.
    pub fn ident(&self) -> Result<u32, FpgaError> {
        Ok(u32::from_be(self.fpga.application_read(Addr::ID0)?))
    }

    /// Check for a valid identifier
    pub fn ident_valid(&self, ident: u32) -> bool {
        ident == Self::EXPECTED_IDENT
    }
}
