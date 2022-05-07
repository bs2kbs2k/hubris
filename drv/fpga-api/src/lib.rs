// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! API crate for the FPGA server.

#![no_std]

use drv_spi_api::SpiError;
use userlib::*;
use zerocopy::{AsBytes, FromBytes};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum FpgaError {
    ImplError(u8),
    BitstreamError(u8),
    InvalidState,
    InvalidValue,
    PortDisabled,
    NotLocked,
}

impl From<FpgaError> for u16 {
    fn from(e: FpgaError) -> Self {
        match e {
            FpgaError::ImplError(error_code) => 0x0100 | (error_code as u16),
            FpgaError::BitstreamError(error_code) => {
                0x0200 | (error_code as u16)
            }
            FpgaError::InvalidState => 0x0300,
            FpgaError::InvalidValue => 0x0301,
            FpgaError::PortDisabled => 0x0400,
            FpgaError::NotLocked => 0x0500,
        }
    }
}

impl From<SpiError> for FpgaError {
    fn from(e: SpiError) -> Self {
        FpgaError::ImplError(u32::from(e) as u8)
    }
}

impl From<FpgaError> for u32 {
    fn from(e: FpgaError) -> Self {
        u16::from(e) as u32
    }
}

impl core::convert::TryFrom<u16> for FpgaError {
    type Error = ();

    fn try_from(v: u16) -> Result<Self, Self::Error> {
        match v & 0xff00 {
            0x0100 => Ok(FpgaError::ImplError(v as u8)),
            0x0200 => Ok(FpgaError::BitstreamError(v as u8)),
            _ => match v {
                0x0300 => Ok(FpgaError::InvalidState),
                0x0301 => Ok(FpgaError::InvalidValue),
                0x0400 => Ok(FpgaError::PortDisabled),
                0x0500 => Ok(FpgaError::NotLocked),
                _ => Err(()),
            },
        }
    }
}

impl core::convert::TryFrom<u32> for FpgaError {
    type Error = ();

    fn try_from(v: u32) -> Result<Self, Self::Error> {
        let v: u16 = v.try_into().map_err(|_| ())?;
        Self::try_from(v)
    }
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum DeviceState {
    Unknown = 0,
    Disabled = 1,
    AwaitingBitstream = 2,
    RunningApplication = 3,
    Error = 4,
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum BitstreamType {
    Uncompressed = 0,
    Compressed = 1,
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum WriteOp {
    Write = 0,
    // This maps onto a generic Op type in Bluespec which defines Read = 1. The
    // read/write split obviates the need for that operation.
    BitSet = 2,
    BitClear = 3,
}

impl From<WriteOp> for u8 {
    fn from(op: WriteOp) -> Self {
        op as u8
    }
}

pub struct FpgaLock<'a>(&'a idl::Fpga);

impl Drop for FpgaLock<'_> {
    fn drop(&mut self) {
        // We ignore the result of release because, if the server has restarted,
        // we don't need to do anything.
        self.0.release().ok();
    }
}

pub struct Fpga(idl::Fpga);

impl Fpga {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self(idl::Fpga::from(task_id))
    }

    pub fn enabled(&self) -> Result<bool, FpgaError> {
        self.0.device_enabled()
    }

    pub fn set_enabled(&mut self, enabled: bool) -> Result<(), FpgaError> {
        self.0.set_device_enabled(enabled)
    }

    pub fn reset(&mut self) -> Result<(), FpgaError> {
        self.0.reset_device()
    }

    pub fn state(&self) -> Result<DeviceState, FpgaError> {
        self.0.device_state()
    }

    pub fn id(&self) -> Result<u32, FpgaError> {
        self.0.device_id()
    }

    pub fn start_bitstream_load(
        &mut self,
        bitstream_type: BitstreamType,
    ) -> Result<Bitstream, FpgaError> {
        let bitstream = Bitstream(self.lock()?);
        bitstream.0 .0.start_bitstream_load(bitstream_type)?;
        Ok(bitstream)
    }

    pub fn lock(&mut self) -> Result<FpgaLock, FpgaError> {
        self.0.lock()?;
        Ok(FpgaLock(&self.0))
    }
}

pub struct Bitstream<'a>(FpgaLock<'a>);

impl Bitstream<'_> {
    pub fn continue_load(&mut self, data: &[u8]) -> Result<(), FpgaError> {
        self.0 .0.continue_bitstream_load(data)
    }

    pub fn finish_load(&mut self) -> Result<(), FpgaError> {
        self.0 .0.finish_bitstream_load()
    }
}

pub struct FpgaApplication(idl::Fpga);

impl FpgaApplication {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self(idl::Fpga::from(task_id))
    }

    pub fn enabled(&self) -> Result<bool, FpgaError> {
        self.0.application_enabled()
    }

    pub fn set_enabled(&mut self, enabled: bool) -> Result<(), FpgaError> {
        self.0.set_application_enabled(enabled)
    }

    pub fn reset(&mut self) -> Result<(), FpgaError> {
        self.0.reset_application()
    }

    pub fn read<T>(&self, addr: impl Into<u16>) -> Result<T, FpgaError>
    where
        T: AsBytes + Default + FromBytes,
    {
        let mut v = T::default();
        self.0.application_read_raw(addr.into(), v.as_bytes_mut())?;
        Ok(v)
    }

    pub fn write<T>(
        &self,
        op: WriteOp,
        addr: impl Into<u16>,
        value: T,
    ) -> Result<(), FpgaError>
    where
        T: AsBytes + FromBytes,
    {
        Ok(self
            .0
            .application_write_raw(op, addr.into(), value.as_bytes())?)
    }

    pub fn lock(&self) -> Result<FpgaLock, FpgaError> {
        self.0.lock()?;
        Ok(FpgaLock(&self.0))
    }
}

mod idl {
    use super::{BitstreamType, DeviceState, FpgaError, WriteOp};
    use userlib::*;

    include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
}
