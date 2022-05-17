// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Common code for the ST7789 driver.

#![no_std]

use userlib::FromPrimitive;
use zerocopy::AsBytes;

#[derive(Copy, Clone, Debug, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum TearingEffect {
    Off = 0,
    Vertical,
    HorizontalAndVertical,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum Orientation {
    Portrait = 0,
    Landscape,
    PortraitSwapped,
    LandscapeSwapped,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, FromPrimitive, AsBytes)]
#[repr(u8)]
pub enum DisplayError {
    DisplayError = 0,
    PinError,
}
