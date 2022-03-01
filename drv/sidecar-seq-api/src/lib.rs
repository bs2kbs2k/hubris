// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! API crate for the Sidecar Sequencer server.

#![no_std]

use derive_idol_err::IdolError;
use userlib::*;
use zerocopy::AsBytes;
use drv_fpga_api::FpgaError;
use drv_sidecar_mainboard_controller_api::tofino2::{State, Error};

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, IdolError)]
pub enum SeqError {
    FpgaError = 1,
    IllegalTransition = 2,
    ClockConfigFailed = 3,
    SequencerError = 4,
    SequencerTimeout = 5,
    InvalidTofinoVid = 6,
    SetVddCoreVoutFailed = 7,
}

#[derive(Copy, Clone, Debug, FromPrimitive, PartialEq, AsBytes)]
#[repr(u8)]
pub enum PowerState {
    A2 = 1,
    A0 = 2,
}

impl From<FpgaError> for SeqError {
    fn from(_: FpgaError) -> Self {
        Self::FpgaError
    }
}

include!(concat!(env!("OUT_DIR"), "/client_stub.rs"));
