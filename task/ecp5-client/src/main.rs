// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_ecp5::client::*;
use ringbuf::*;
use userlib::*;

task_slot!(ECP5, ecp5);

#[derive(Copy, Clone, Debug, PartialEq)]
enum Trace {
    None,
    AwaitingReady,
    Ready,
    StateChange(DeviceState),
}
ringbuf!(Trace, 16, Trace::None);

#[export_name = "main"]
pub fn main() -> ! {
    const ECP5_NOTIFICATION: u32 = 1;

    let ecp5 = Ecp5::from(ECP5.get_task_id());
    let mut msg = [0; 16];

    ringbuf_entry!(Trace::AwaitingReady);

    // Check to see if the ECP5 is ready.
    while match ecp5.state() {
        Ok(state) => state != DeviceState::UserMode,
        Err(_) => true, // Always retry
    } {
        // Wait for a notification we should retry.
        let _ = sys_recv_open(&mut msg, ECP5_NOTIFICATION);
    }

    ringbuf_entry!(Trace::Ready);

    loop {
        let _ = sys_recv_open(&mut msg, ECP5_NOTIFICATION);

        if let Ok(state) = ecp5.state() {
            if state == DeviceState::Disabled
                || state == DeviceState::Configuration
            {
                kipc::restart_task(
                    hubris_num_tasks::Task::ecp5_client as usize,
                    true,
                );
            }

            ringbuf_entry!(Trace::StateChange(state));
        }
    }
}
