// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

//pub mod client;
pub mod spi;
pub mod types;

use ringbuf::*;
use userlib::hl::sleep_for;
use zerocopy::{AsBytes, FromBytes};

pub use types::*;

#[derive(Copy, Clone, Debug, PartialEq)]
enum Trace {
    None,
    Disabled,
    Enabled,
    Command(Command),
    ReadId(u32, DeviceId),
    ReadUserCode(u32),
    ReadStatus(u32),
    CheckBusy(bool),
    Read32(Command, u32),
    LoadBitstreamChunk(usize),
    StandardBitstreamDetected,
    EncryptedBitstreamDetected,
    BitstreamError(BitstreamError),
    DesignResetAsserted,
    DesignResetDeasserted,
}
ringbuf!(Trace, 16, Trace::None);

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum Ecp5Error {
    ImplError(u8),
    BitstreamError(BitstreamError),
    PortDisabled,
    InvalidMode,
}

pub trait Ecp5Impl {
    type Error;

    /// PROGAM_N interface. This pin acts as a device reset and when asserted
    /// low force to (re)start the bitstream loading process.
    ///
    /// See FPGA-TN-02039-2.0, 4.5.2 for details.
    fn program_n(&self) -> Result<bool, Self::Error>;
    fn set_program_n(&self, asserted: bool) -> Result<(), Self::Error>;

    /// INIT_N interface. This pin can be driven after reset/power up to keep
    /// the device from entering Configuration state. As input it signals
    /// Initialization complete or an error occured during bitstream loading.
    ///
    /// See FPGA-TN-02039-2.0, 4.5.3 for details.
    fn init_n(&self) -> Result<bool, Self::Error>;
    fn set_init_n(&self, asserted: bool) -> Result<(), Self::Error>;

    /// DONE interface. This pin signals the device is in User Mode. Asserting
    /// the pin keeps the device from entering User Mode after Configuration.
    ///
    /// See FPGA-TN-02039-2.0, 4.5.4 for details.
    fn done(&self) -> Result<bool, Self::Error>;
    fn set_done(&self, asserted: bool) -> Result<(), Self::Error>;

    /// Design Reset.
    fn design_reset_n(&self) -> Result<bool, Self::Error>;
    fn set_design_reset_n(&self, asserted: bool) -> Result<(), Self::Error>;

    /// A generic interface to send commands and read/write data from a
    /// configuration port. This interface is intended to be somewhat transport
    /// agnostic so either SPI or JTAG could be used if desired.
    fn write_command(&self, c: Command) -> Result<(), Self::Error>;
    fn read(&self, buf: &mut [u8]) -> Result<(), Self::Error>;
    fn write(&self, buf: &[u8]) -> Result<(), Self::Error>;

    /// The command interface may exist on a shared medium such as SPI. The
    /// following primitives allow the upper half of the driver to issue atomic
    /// commands.
    ///
    /// If no lock control of the medium is needed these can be implemented as
    /// no-op.
    fn lock(&self) -> Result<(), Self::Error>;
    fn release(&self) -> Result<(), Self::Error>;
}

/// Opague ECP5 device handle.
pub struct Ecp5<'a, Ecp5ImplError> {
    device: &'a dyn Ecp5Impl<Error = Ecp5ImplError>,
}

impl<'a, Ecp5ImplError> Ecp5<'a, Ecp5ImplError>
where
    Ecp5Error: From<Ecp5ImplError>,
{
    pub fn new(device: &'a dyn Ecp5Impl<Error = Ecp5ImplError>) -> Self {
        Ecp5 { device }
    }

    pub fn device_enabled(&self) -> Result<bool, Ecp5Error> {
        Ok(self.device.program_n()?)
    }

    pub fn set_device_enable(&self, enabled: bool) -> Result<(), Ecp5Error> {
        self.device.set_program_n(enabled)?;
        ringbuf_entry!(if enabled {
            Trace::Enabled
        } else {
            Trace::Disabled
        });
        Ok(())
    }

    pub fn reset_device(&self) -> Result<(), Ecp5Error> {
        self.set_device_enable(false)?;
        sleep_for(50);
        self.set_device_enable(true)?;
        Ok(())
    }

    /// Return the device state based on the current state of its control pins.
    pub fn state(&self) -> Result<DeviceState, Ecp5Error> {
        if !self.device.program_n()? {
            Ok(DeviceState::Disabled)
        } else {
            if self.device.done()? {
                Ok(DeviceState::UserMode)
            } else {
                if self.device.init_n()? {
                    Ok(DeviceState::Configuration)
                } else {
                    Ok(DeviceState::InitializationOrConfigurationError)
                }
            }
        }
    }

    pub fn user_design_enabled(&self) -> Result<bool, Ecp5Error> {
        Ok(self.device.design_reset_n()?)
    }

    pub fn set_user_design_enable(
        &self,
        enabled: bool,
    ) -> Result<(), Ecp5Error> {
        self.device.set_design_reset_n(enabled)?;
        ringbuf_entry!(if enabled {
            Trace::DesignResetDeasserted
        } else {
            Trace::DesignResetAsserted
        });
        Ok(())
    }

    pub fn reset_user_design(&self) -> Result<(), Ecp5Error> {
        self.set_user_design_enable(false)?;
        sleep_for(10);
        self.set_user_design_enable(true)?;
        Ok(())
    }

    /// Send a command to the device which does not return or require additional
    /// data. FPGA-TN-02039-2.0, 6.2.5 refers to this as a Class C command.
    pub fn send_command(&self, c: Command) -> Result<(), Ecp5Error> {
        self.device.lock()?;
        self.device.write_command(c)?;
        self.device.release()?;
        ringbuf_entry!(Trace::Command(c));
        Ok(())
    }

    /// Send a command and read back a number of bytes given by the type T. Note
    /// that data is always returned in big endian order.
    pub fn read<T: Default + AsBytes + FromBytes>(
        &self,
        c: Command,
    ) -> Result<T, Ecp5Error> {
        let mut buf = T::default();

        self.device.lock()?;
        self.device.write_command(c)?;
        self.device.read(buf.as_bytes_mut())?;
        self.device.release()?;

        Ok(buf)
    }

    pub fn read16(&self, c: Command) -> Result<u16, Ecp5Error> {
        Ok(u16::from_be(self.read(c)?))
    }

    pub fn read32(&self, c: Command) -> Result<u32, Ecp5Error> {
        let v = u32::from_be(self.read(c)?);

        match c {
            Command::ReadId => {
                ringbuf_entry!(Trace::ReadId(v, DeviceId::from(v)))
            }
            Command::ReadStatus => ringbuf_entry!(Trace::ReadStatus(v)),
            Command::ReadUserCode => ringbuf_entry!(Trace::ReadUserCode(v)),
            _ => ringbuf_entry!(Trace::Read32(c, v)),
        }

        // The configuration port may be disabled and in the case of a ReadStatus or
        // ReadId command this is easily detectable. Since these are mostly likely
        // the first commands to be issued it is useful to have these return an
        // appropriate error.
        if (c == Command::ReadStatus || c == Command::ReadId) && v == 0xffffffff
        {
            return Err(Ecp5Error::PortDisabled);
        }

        Ok(v)
    }

    pub fn write(&self, buf: &[u8]) -> Result<(), Ecp5Error> {
        Ok(self.device.write(buf)?)
    }

    /// Read the device ID.
    pub fn id(&self) -> Result<DeviceId, Ecp5Error> {
        let id = self.read32(Command::ReadId)?;
        Ok(DeviceId::from(id))
    }

    /// Read the device user code.
    pub fn user_code(&self) -> Result<u32, Ecp5Error> {
        self.read32(Command::ReadUserCode)
    }

    /// Read the Status register
    pub fn status(&self) -> Result<Status, Ecp5Error> {
        Ok(Status(self.read32(Command::ReadStatus)?))
    }

    pub fn enable_configuration_mode(&self) -> Result<(), Ecp5Error> {
        self.send_command(Command::EnableConfigurationMode)
    }

    pub fn disable_configuration_mode(&self) -> Result<(), Ecp5Error> {
        self.send_command(Command::DisableConfigurationMode)
    }

    pub fn await_not_busy(&self, sleep_interval: u64) -> Result<(), Ecp5Error> {
        while self.status()?.busy() {
            sleep_for(sleep_interval);
        }
        Ok(())
    }

    pub fn await_done(&self, sleep_interval: u64) -> Result<(), Ecp5Error> {
        while !self.status()?.done() {
            sleep_for(sleep_interval);
        }
        Ok(())
    }

    pub fn initiate_bitstream_load(&self) -> Result<(), Ecp5Error> {
        // Put device in configuration mode if required.
        if !self.status()?.write_enabled() {
            self.enable_configuration_mode()?;
        }

        if !self.status()?.write_enabled() {
            return Err(Ecp5Error::InvalidMode);
        }

        // Assert the design reset.
        self.set_user_design_enable(false)?;

        self.device.lock()?;
        // Use the Impl to write the command and leave the device locked for the
        // byte stream to follow.
        self.device.write_command(Command::BitstreamBurst)?;
        ringbuf_entry!(Trace::Command(Command::BitstreamBurst));

        Ok(())
    }

    pub fn continue_bitstream_load(&self, buf: &[u8]) -> Result<(), Ecp5Error> {
        self.device.write(buf)?;
        //ringbuf_entry!(Trace::LoadBitstreamChunk(buf.len()));
        Ok(())
    }

    pub fn finalize_bitstream_load(&self) -> Result<(), Ecp5Error> {
        self.device.release()?;
        self.await_not_busy(10)?;

        // Perform climb-out checklist; determine if the bitstream was accepted
        // and the device is ready for wake up.
        let status = self.status()?;

        if status.encrypt_preamble_detected() {
            ringbuf_entry!(Trace::EncryptedBitstreamDetected);
        }
        if status.standard_preamble_detected() {
            ringbuf_entry!(Trace::StandardBitstreamDetected);
        }

        let error = status.bitstream_error();

        if error != BitstreamError::None {
            // Log and bail. This leaves the device in configuration mode (and
            // the SPI port enabled), allowing the caller to issue a Refresh
            // command and try again if so desired.
            ringbuf_entry!(Trace::BitstreamError(error));
            return Err(Ecp5Error::BitstreamError(error));
        }

        ringbuf_entry!(Trace::BitstreamError(BitstreamError::None));

        // Return to user mode, initiating the control sequence which will start
        // the fabric. Completion of this transition is externally observable
        // with the DONE pin going high.
        //
        // Unless the port is set to remain enabled through the FAE bits it will
        // be disabled at this point, i.e. performing a read of the ID or Status
        // registers will result in a PortDisabled error.
        self.disable_configuration_mode()?;

        while !self.device.done()? {
            sleep_for(10);
        }

        // Release design reset after 10 ms.
        sleep_for(10);
        self.set_user_design_enable(true)?;

        Ok(())
    }
}

impl From<Ecp5Error> for u16 {
    fn from(e: Ecp5Error) -> Self {
        match e {
            Ecp5Error::ImplError(c) => 0x100 | (c as u16),
            Ecp5Error::BitstreamError(e) => match e {
                BitstreamError::None => 0x200 | 0,
                BitstreamError::InvalidId => 0x200 | 1,
                BitstreamError::IllegalCommand => 0x200 | 2,
                BitstreamError::CrcMismatch => 0x200 | 3,
                BitstreamError::InvalidPreamble => 0x200 | 4,
                BitstreamError::UserAbort => 0x200 | 5,
                BitstreamError::DataOverflow => 0x200 | 6,
                BitstreamError::SramDataOverflow => 0x200 | 7,
            },
            Ecp5Error::PortDisabled => 0x300,
            Ecp5Error::InvalidMode => 0x301,
            Ecp5Error::Notification => 0x302,
        }
    }
}

impl From<Ecp5Error> for u32 {
    fn from(e: Ecp5Error) -> Self {
        u16::from(e) as u32
    }
}

impl core::convert::TryFrom<u32> for Ecp5Error {
    type Error = ();
    fn try_from(rc: u32) -> Result<Self, Self::Error> {
        match rc & 0xffffff00 {
            0x100 => Ok(Ecp5Error::ImplError((rc & 0x000000ff) as u8)),
            0x200 => match rc & 0x000000ff {
                0 => Ok(Ecp5Error::BitstreamError(BitstreamError::None)),
                1 => Ok(Ecp5Error::BitstreamError(BitstreamError::InvalidId)),
                2 => Ok(Ecp5Error::BitstreamError(
                    BitstreamError::IllegalCommand,
                )),
                3 => Ok(Ecp5Error::BitstreamError(BitstreamError::CrcMismatch)),
                4 => Ok(Ecp5Error::BitstreamError(
                    BitstreamError::InvalidPreamble,
                )),
                5 => Ok(Ecp5Error::BitstreamError(BitstreamError::UserAbort)),
                6 => {
                    Ok(Ecp5Error::BitstreamError(BitstreamError::DataOverflow))
                }
                7 => Ok(Ecp5Error::BitstreamError(
                    BitstreamError::SramDataOverflow,
                )),
                _ => Err(()),
            },
            0x300 => Ok(Ecp5Error::PortDisabled),
            0x301 => Ok(Ecp5Error::InvalidMode),
            0x302 => Ok(Ecp5Error::Notification),
            _ => Err(()),
        }
    }
}
