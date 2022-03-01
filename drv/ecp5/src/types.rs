// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use bitfield::bitfield;
use userlib::FromPrimitive;
use zerocopy::AsBytes;

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum DeviceId {
    Invalid,
    Lfe5u12,
    Lfe5u25,
    Lfe5u45,
    Lfe5u85,
    Lfe5um25,
    Lfe5um45,
    Lfe5um85,
    Lfe5um5g25,
    Lfe5um5g45,
    Lfe5um5g85,
}

impl From<u32> for DeviceId {
    fn from(id: u32) -> Self {
        match id {
            0x21111043 => DeviceId::Lfe5u12,
            0x41111043 => DeviceId::Lfe5u25,
            0x41112043 => DeviceId::Lfe5u45,
            0x41113043 => DeviceId::Lfe5u85,
            0x01111043 => DeviceId::Lfe5um25,
            0x01112043 => DeviceId::Lfe5um45,
            0x01113043 => DeviceId::Lfe5um85,
            0x81111043 => DeviceId::Lfe5um5g25,
            0x81112043 => DeviceId::Lfe5um5g45,
            0x81113043 => DeviceId::Lfe5um5g85,
            _ => DeviceId::Invalid,
        }
    }
}

impl From<DeviceId> for u32 {
    fn from(id: DeviceId) -> Self {
        match id {
            DeviceId::Lfe5u12 => 0x21111043,
            DeviceId::Lfe5u25 => 0x41111043,
            DeviceId::Lfe5u45 => 0x41112043,
            DeviceId::Lfe5u85 => 0x41113043,
            DeviceId::Lfe5um25 => 0x01111043,
            DeviceId::Lfe5um45 => 0x01112043,
            DeviceId::Lfe5um85 => 0x01113043,
            DeviceId::Lfe5um5g25 => 0x81111043,
            DeviceId::Lfe5um5g45 => 0x81112043,
            DeviceId::Lfe5um5g85 => 0x81113043,
            DeviceId::Invalid => 0x00000000,
        }
    }
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum DeviceState {
    Unknown = 0,
    Disabled = 1,
    Configuration = 2,
    UserMode = 3,
    Error = 4,
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum BitstreamError {
    None = 0,
    InvalidId = 0b001,
    IllegalCommand = 0b010,
    CrcMismatch = 0b011,
    InvalidPreamble = 0b100,
    UserAbort = 0b101,
    DataOverflow = 0b110,
    SramDataOverflow = 0b111,
}

bitfield! {
    pub struct Status(u32);
    pub transparent_mode, _: 0;
    pub config_target_selection, _: 3, 1;
    pub jtag_active, _: 4;
    pub pwd_protection, _: 5;
    reserved1, _: 6;
    pub decrypt_enable, _: 7;
    pub done, _: 8;
    pub isc_enabled, _: 9;
    pub write_enabled, _: 10;
    pub read_enabled, _: 11;
    pub busy, _: 12;
    pub fail, _: 13;
    pub fea_otp, _: 14;
    pub decrypt_only, _: 15;
    pub pwd_enabled, _: 16;
    reserved2, _: 19, 17;
    pub encrypt_preamble_detected, _: 20;
    pub standard_preamble_detected, _: 21;
    pub spim_fail1, _: 22;
    pub bse_error_code, _: 25, 23;
    pub execution_error, _: 26;
    pub id_error, _: 27;
    pub invalid_command, _: 28;
    pub sed_error, _: 29;
    pub bypass_mode, _: 30;
    pub flow_through_mode, _: 31;
}

impl Status {
    pub fn bitstream_error(&self) -> BitstreamError {
        match self.bse_error_code() {
            0b001 => BitstreamError::InvalidId,
            0b010 => BitstreamError::IllegalCommand,
            0b011 => BitstreamError::CrcMismatch,
            0b100 => BitstreamError::InvalidPreamble,
            0b101 => BitstreamError::UserAbort,
            0b110 => BitstreamError::DataOverflow,
            0b111 => BitstreamError::SramDataOverflow,
            _ => BitstreamError::None,
        }
    }
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum Command {
    Noop = 0xff,
    ReadId = 0xe0,
    ReadUserCode = 0xc0,
    ReadStatus = 0x3c,
    CheckBusy = 0xf0,
    Refresh = 0x79,
    EnableConfigurationMode = 0xc6,
    EnableTransparentConfigurationMode = 0x74,
    DisableConfigurationMode = 0x26,
    Erase = 0x0e,
    BitstreamBurst = 0x7a,
}
