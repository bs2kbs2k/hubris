// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_fpga_api::{Bitstream, BitstreamType, Fpga, FpgaError};
use idol_runtime::{ClientError, Leased, LenLimit, R};
use userlib::*;

task_slot!(SYS, sys);
task_slot!(FPGA, fpga);

#[export_name = "main"]
fn main() -> ! {
    let mut buffer = [0u8; 128];
    let mut server = ServerImpl {
        fpga: Fpga::new(FPGA.get_task_id()),
        bitstream: None,
        chunk: [0u8; 128],
    };

    loop {
        idol_runtime::dispatch(&mut buffer, &mut server);
    }
}

struct ServerImpl {
    fpga: Fpga,
    bitstream: Option<Bitstream>,
    chunk: [u8; 128],
}

type RequestError = idol_runtime::RequestError<FpgaError>;

impl InOrderFpgaDebugImpl for ServerImpl {
    fn start_bitstream_load(
        &mut self,
        _: &RecvMessage,
        bitstream_type: BitstreamType,
    ) -> Result<(), RequestError> {
        match &mut self.bitstream {
            Some(_) => panic!(),
            None => {
                self.bitstream =
                    Some(self.fpga.start_bitstream_load(bitstream_type)?);
                Ok(())
            }
        }
    }

    fn continue_bitstream_load(
        &mut self,
        _: &RecvMessage,
        data: LenLimit<Leased<R, [u8]>, 128>,
    ) -> Result<(), RequestError> {
        match &mut self.bitstream {
            None => panic!(),
            Some(bitstream) => {
                data.read_range(0..data.len(), &mut self.chunk[..data.len()])
                    .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

                bitstream.continue_load(&self.chunk[..data.len()]).map_err(
                    |e| {
                        self.bitstream = None;
                        e.into()
                    },
                )
            }
        }
    }

    fn finish_bitstream_load(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError> {
        match &mut self.bitstream {
            None => panic!(),
            Some(bitstream) => {
                bitstream.finish_load()?;
                self.bitstream = None;
                Ok(())
            }
        }
    }
}

include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
