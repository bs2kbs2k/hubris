// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_st7789_common::*;
use idol_runtime::RequestError;
use userlib::*;

#[export_name = "main"]
fn main() -> ! {
    // Handle messages.
    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut serverimpl = ServerImpl;
    loop {
        idol_runtime::dispatch(&mut incoming, &mut serverimpl);
    }
}

struct HubrisDelaySource;

impl DelayUs<u32> for HubrisDelaySource {
    fn delay_us(&mut self, us: u32) {
        userlib::hl::sleep_for(us);
    }
}

struct ServerImpl {
    display: st7789::ST7789,
}

mod idl {
    use super::LedError;

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
