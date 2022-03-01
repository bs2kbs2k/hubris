// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

use drv_ecp5::spi::{Ecp5Spi, Ecp5SpiError};
use drv_ecp5::{DeviceId, DeviceState, BitstreamType, Ecp5, Ecp5Error};
use drv_spi_api::Spi;
use drv_stm32xx_sys_api::{self as sys_api, Sys};
use idol_runtime::{ClientError, Leased, LenLimit, R};
use ringbuf::*;
use userlib::*;

task_slot!(SYS, sys);
task_slot!(SPI, spi_driver);

#[derive(Copy, Clone, Debug, PartialEq)]
enum Trace {
    None,
    InitBitstreamLoad(BitstreamType),
    BufferLen(usize, usize, usize),
    NotifiedClients,
}
ringbuf!(Trace, 16, Trace::None);

#[export_name = "main"]
fn main() -> ! {
    cfg_if::cfg_if! {
        if #[cfg(target_board = "sidecar-1")] {
            let ecp5_bsp = Ecp5Spi {
                sys: Sys::from(SYS.get_task_id()),
                spi: Spi::from(SPI.get_task_id()).device(0),
                done: sys_api::Port::J.pin(15),
                init_n: sys_api::Port::J.pin(12),
                program_n: sys_api::Port::J.pin(13),
                design_reset_n: sys_api::Port::J.pin(14),
            };
            let skip_default_reset_on_boot = false;
        } else if #[cfg(target_board = "gimletlet-2")] {
            let ecp5_bsp = Ecp5Spi {
                sys: Sys::from(SYS.get_task_id()),
                spi: Spi::from(SPI.get_task_id()).device(0),
                done: sys_api::Port::E.pin(15),
                init_n: sys_api::Port::D.pin(12),
                program_n: sys_api::Port::B.pin(10),
                design_reset_n: sys_api::Port::D.pin(11),
            };
            let skip_default_reset_on_boot = false;
        } else {
            compile_error!("Board is not supported by the task/fpga");
        }
    }
    ecp5_bsp.configure_gpio();

    let mut incoming = [0u8; idl::INCOMING_SIZE];
    let mut server = ServerImpl {
        ecp5: Ecp5::new(&ecp5_bsp),
        buffer: [0u8; 128],
        clients: generated::NotificationSubscriptions::default(),
        decompressor: None,
    };

    // Do not reset the device if it is already in UserMode.
    let current_state = server.ecp5.state().unwrap();

    if !skip_default_reset_on_boot || current_state != DeviceState::UserMode {
        server.ecp5.reset_device().unwrap();
        server.ecp5.id().unwrap();
        //server.clients.notify();
    }

    loop {
        idol_runtime::dispatch(&mut incoming, &mut server);
    }
}

struct ServerImpl<'a, Ecp5SpiError> {
    ecp5: Ecp5<'a, Ecp5SpiError>,
    buffer: [u8; 128],
    clients: generated::NotificationSubscriptions,
    decompressor: Option<gnarle::Decompressor>,
}

type RequestError = idol_runtime::RequestError<Ecp5Error>;

impl<'a> idl::InOrderEcp5Impl for ServerImpl<'a, Ecp5SpiError> {
    fn device_enabled(&mut self, _: &RecvMessage) -> Result<u8, RequestError> {
        Ok(self.ecp5.device_enabled()? as u8)
    }

    fn set_device_enable(&mut self, _: &RecvMessage, enabled: u8) -> Result<(), RequestError> {
        Ok(self.ecp5.set_device_enable(enabled != 0)?)
    }

    fn reset_device(&mut self, _: &RecvMessage) -> Result<(), RequestError> {
        Ok(self.ecp5.reset_device()?)
    }

    fn state(&mut self, _: &RecvMessage) -> Result<DeviceState, RequestError> {
        Ok(self.ecp5.state()?)
    }

    fn user_design_enabled(&mut self, _: &RecvMessage) -> Result<u8, RequestError> {
        Ok(self.ecp5.user_design_enabled()? as u8)
    }

    fn set_user_design_enable(&mut self, _: &RecvMessage, enabled: u8) -> Result<(), RequestError> {
        Ok(self.ecp5.set_user_design_enable(enabled != 0)?)
    }

    fn reset_user_design(&mut self, _: &RecvMessage) -> Result<(), RequestError> {
        Ok(self.ecp5.reset_user_design()?)
    }

    fn id(&mut self, _: &RecvMessage) -> Result<DeviceId, RequestError> {
        Ok(u32::from(self.ecp5.id()?))
    }

    fn status(&mut self, _: &RecvMessage) -> Result<u32, RequestError> {
        Ok(self.ecp5.status()?.0)
    }

    fn init_bitstream_load(
        &mut self,
        _: &RecvMessage,
        bitstream_type: BitstreamType
    ) -> Result<(), RequestError> {
        ringbuf_entry!(Trace::InitBitstreamLoad(bitstream_type));

        if let BitstreamType::Compressed = bitstream_type {
            self.decompressor = Some(gnarle::Decompressor::default())
        }
        Ok(self.ecp5.initiate_bitstream_load()?)
    }

    fn continue_bitstream_load(
        &mut self,
        _: &RecvMessage,
        data: LenLimit<Leased<R, [u8]>, 128>,
    ) -> Result<(), RequestError> {
        data.read_range(0..data.len(), &mut self.buffer[..data.len()])
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;

        let chunk = &mut &self.buffer[..data.len()];
        let mut decompress_buffer = [0; 256];

        match self.decompressor.as_mut() {
            Some(decompressor) => {
                while !chunk.is_empty() {
                    let decompressed_chunk = gnarle::decompress(decompressor, chunk, &mut decompress_buffer);
                    self.ecp5.continue_bitstream_load(decompressed_chunk)?;
                }
            },
            None => self.ecp5.continue_bitstream_load(chunk)?,
        }

        Ok(())
    }

    fn finalize_bitstream_load(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError> {
        self.decompressor = None;
        self.ecp5.finalize_bitstream_load()?;
        self.clients.notify_or_update_and_retry();
        Ok(())
    }
}

mod idl {
    use super::{DeviceId, DeviceState, BitstreamType, Ecp5Error};

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}

// This is not actually generated but could/should be. This is an attempt to
// first prototype the non-generic case and then go back to make this generated.
mod generated {
    pub struct NotificationSubscriptions(pub [(userlib::TaskId, u32); 1usize]);

    cfg_if::cfg_if! {
        if #[cfg(target_board = "sidecar-1")] {
            impl Default for NotificationSubscriptions {
                fn default() -> Self {
                    NotificationSubscriptions([(
                        userlib::TaskId::for_index_and_gen(
                            hubris_num_tasks::Task::sequencer as usize,
                            userlib::Generation::ZERO,
                        ),
                        0x2,
                    )])
                }
            }
        } else if #[cfg(target_board = "gimletlet-2")] {
            impl Default for NotificationSubscriptions {
                fn default() -> Self {
                    NotificationSubscriptions([(
                        userlib::TaskId::for_index_and_gen(
                            hubris_num_tasks::Task::sequencer as usize,
                            userlib::Generation::ZERO,
                        ),
                        0x2,
                    )])
                }
            }
        } else {
            compile_error!("Board is not supported by the task/ecp5");
        }
    }

    use ringbuf::*;
    use super::{Trace, __RINGBUF};

    impl NotificationSubscriptions {
        pub(crate) fn notify(&self) {
            for &(client, mask) in self.0.iter() {
                userlib::sys_post(client, mask);
            }
            ringbuf_entry!(Trace::NotifiedClients);
        }

        pub(crate) fn notify_or_update_and_retry(&mut self) {
            for subscription in self.0.iter_mut() {
                let (client, mask) = *subscription;
                let rc = userlib::sys_post(client, mask);

                // Handle a client TaskId having goine stale. This is done only
                // once as the client could be in a tight crash loop causing
                // this function never to catch up. The result is that
                // notifications would be dropped for such tasks, but if a task
                // is restarted that often it most likely has other things to
                // worry about.
                if (rc & 0xffffff00) == 0xffffff00 {
                    let client = userlib::TaskId::for_index_and_gen(
                        client.index(),
                        userlib::Generation::from((rc & 0x000000ff) as u8),
                    );
                    *subscription = (client, mask);

                    // Retry the notification.
                    userlib::sys_post(client, mask);
                }
            }
            ringbuf_entry!(Trace::NotifiedClients);
        }
    }
}
