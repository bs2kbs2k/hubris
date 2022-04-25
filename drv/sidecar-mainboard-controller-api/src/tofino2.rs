// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use super::{Addr, Reg};
use drv_fpga_api::{FpgaApplication, FpgaError, WriteOp};
use userlib::FromPrimitive;
use zerocopy::AsBytes;

#[derive(Copy, Clone, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum State {
    Initial = 0,
    A2 = 1,
    A0 = 2,
    InPowerUp = 3,
    InPowerDown = 4,
}

#[derive(Copy, Clone, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum Error {
    None = 0,
    PowerGoodTimeout = 1,
    PowerFault = 2,
    PowerVrHot = 3,
    PowerInvalidState = 4,
    UserAbort = 5,
    VidAckTimeout = 6,
    ThermalAlert = 7,
}

#[derive(Copy, Clone, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum Vid {
    V0P922,
    V0P893,
    V0P867,
    V0P847,
    V0P831,
    V0P815,
    V0P790,
    V0P759,
}

pub struct Sequencer {
    fpga: FpgaApplication,
}

impl Sequencer {
    pub fn new(task_id: userlib::TaskId) -> Self {
        Self {
            fpga: FpgaApplication::new(task_id),
        }
    }

    fn read_masked(&self, addr: Addr, mask: u8) -> Result<u8, FpgaError> {
        let v: u8 = self.fpga.read(addr)?;
        Ok(v & mask)
    }

    fn write_ctrl(&self, op: WriteOp, value: u8) -> Result<(), FpgaError> {
        self.fpga.write(op, Addr::TOFINO_SEQ_CTRL, value)
    }

    pub fn clear_error(&self) -> Result<(), FpgaError> {
        self.write_ctrl(WriteOp::BitSet, Reg::TOFINO_SEQ_CTRL::CLEAR_ERROR)
    }

    pub fn enabled(&self) -> Result<bool, FpgaError> {
        Ok(
            self.read_masked(Addr::TOFINO_SEQ_CTRL, Reg::TOFINO_SEQ_CTRL::EN)?
                != 0,
        )
    }

    pub fn set_enable(&self, enabled: bool) -> Result<(), FpgaError> {
        let op = if enabled {
            WriteOp::BitSet
        } else {
            WriteOp::BitClear
        };
        self.write_ctrl(op, Reg::TOFINO_SEQ_CTRL::EN)
    }

    pub fn ack_vid(&self) -> Result<(), FpgaError> {
        self.write_ctrl(WriteOp::BitSet, Reg::TOFINO_SEQ_CTRL::ACK_VID)
    }

    pub fn state(&self) -> Result<State, FpgaError> {
        match self
            .read_masked(Addr::TOFINO_SEQ_STATE, Reg::TOFINO_SEQ_STATE::STATE)?
        {
            0 => Ok(State::Initial),
            1 => Ok(State::A2),
            2 => Ok(State::A0),
            3 => Ok(State::InPowerUp),
            4 => Ok(State::InPowerDown),
            _ => Err(FpgaError::InvalidValue),
        }
    }

    pub fn error(&self) -> Result<Error, FpgaError> {
        match self
            .read_masked(Addr::TOFINO_SEQ_ERROR, Reg::TOFINO_SEQ_ERROR::ERROR)?
        {
            0 => Ok(Error::None),
            1 => Ok(Error::PowerGoodTimeout),
            2 => Ok(Error::PowerFault),
            3 => Ok(Error::PowerVrHot),
            4 => Ok(Error::PowerInvalidState),
            5 => Ok(Error::UserAbort),
            6 => Ok(Error::VidAckTimeout),
            7 => Ok(Error::ThermalAlert),
            _ => Err(FpgaError::InvalidValue),
        }
    }

    pub fn power_status(&self) -> Result<u32, FpgaError> {
        self.fpga.read(Addr::TOFINO_POWER_ENABLE)
    }

    pub fn vid(&self) -> Result<Option<Vid>, FpgaError> {
        let mask =
            Reg::TOFINO_POWER_VID::VID_VALID | Reg::TOFINO_POWER_VID::VID;
        let vid = self.read_masked(Addr::TOFINO_POWER_VID, mask)?;

        if (vid & Reg::TOFINO_POWER_VID::VID_VALID) != 0 {
            return match vid & Reg::TOFINO_POWER_VID::VID {
                0b1111 => Ok(Some(Vid::V0P922)),
                0b1110 => Ok(Some(Vid::V0P893)),
                0b1101 => Ok(Some(Vid::V0P867)),
                0b1100 => Ok(Some(Vid::V0P847)),
                0b1011 => Ok(Some(Vid::V0P831)),
                0b1010 => Ok(Some(Vid::V0P815)),
                0b1001 => Ok(Some(Vid::V0P790)),
                0b1000 => Ok(Some(Vid::V0P759)),
                _ => Err(FpgaError::InvalidValue),
            };
        } else {
            Ok(None)
        }
    }
}
