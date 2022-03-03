// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]

mod dev;
pub mod mac;
pub mod miim_phy;
mod port;
mod serdes10g;
mod serdes1g;
mod serdes6g;
pub mod spi;

use userlib::hl::sleep_for;
use vsc7448_pac::{types::RegisterAddress, *};
pub use vsc_err::VscError;

use crate::dev::{Dev10g, DevGeneric};

/// This trait abstracts over various ways of talking to a VSC7448.
pub trait Vsc7448Rw {
    /// Writes to a VSC7448 register.  Depending on the underlying transit
    /// mechanism, this may panic if registers are written outside of the
    /// switch core block (0x71000000 to 0x72000000)
    fn write<T>(
        &self,
        reg: RegisterAddress<T>,
        value: T,
    ) -> Result<(), VscError>
    where
        u32: From<T>;

    fn read<T>(&self, reg: RegisterAddress<T>) -> Result<T, VscError>
    where
        T: From<u32>;

    /// Performs a write operation on the given register, where the value is
    /// calculated by calling f(0).  This is helpful as a way to reduce manual
    /// type information.
    ///
    /// The register must be in the switch core register block, i.e. having an
    /// address in the range 0x71000000-0x72000000; otherwise, this will panic.
    fn write_with<T, F>(
        &self,
        reg: RegisterAddress<T>,
        f: F,
    ) -> Result<(), VscError>
    where
        T: From<u32>,
        u32: From<T>,
        F: Fn(&mut T),
    {
        let mut data = 0.into();
        f(&mut data);
        self.write(reg, data)
    }

    /// Performs a read-modify-write operation on a VSC7448 register
    ///
    /// The register must be in the switch core register block, i.e. having an
    /// address in the range 0x71000000-0x72000000; otherwise, this will panic.
    fn modify<T, F>(
        &self,
        reg: RegisterAddress<T>,
        f: F,
    ) -> Result<(), VscError>
    where
        T: From<u32>,
        u32: From<T>,
        F: Fn(&mut T),
    {
        let mut data = self.read(reg)?;
        f(&mut data);
        self.write(reg, data)
    }

    /// Writes to a port mask, which is assumed to be a pair of adjacent
    /// registers representing all 53 ports.
    fn write_port_mask<T>(
        &self,
        mut reg: RegisterAddress<T>,
        value: u64,
    ) -> Result<(), VscError>
    where
        T: From<u32>,
        u32: From<T>,
    {
        self.write(reg, ((value & 0xFFFFFFFF) as u32).into())?;
        reg.addr += 4; // Good luck!
        self.write(reg, (((value >> 32) as u32) & 0x1FFFFF).into())
    }
}

////////////////////////////////////////////////////////////////////////////////

/// Top-level state wrapper for a VSC7448 chip.
pub struct Vsc7448<'a, R> {
    pub rw: &'a mut R,
}

impl<R: Vsc7448Rw> Vsc7448Rw for Vsc7448<'_, R> {
    /// Write a register to the VSC7448
    fn write<T>(
        &self,
        reg: RegisterAddress<T>,
        value: T,
    ) -> Result<(), VscError>
    where
        u32: From<T>,
    {
        self.rw.write(reg, value)
    }

    /// Read a register from the VSC7448
    fn read<T>(&self, reg: RegisterAddress<T>) -> Result<T, VscError>
    where
        T: From<u32>,
    {
        self.rw.read(reg)
    }
}

impl<'a, R: Vsc7448Rw> Vsc7448<'a, R> {
    pub fn new(rw: &'a mut R) -> Self {
        Self { rw }
    }

    /// Initializes the given ports as an SFI connection.  The given ports must
    /// be in the range 49..=52, otherwise this function will panic.
    ///
    /// This will configure the appropriate DEV10G and SERDES10G.
    pub fn init_sfi(&self, ports: &[u8]) -> Result<(), VscError> {
        let serdes_cfg = serdes10g::Config::new(serdes10g::Mode::Lan10g)?;
        for &port in ports {
            assert!(port >= 49);
            assert!(port <= 52);
            let dev = Dev10g::new(port - 49)?;
            dev.init_sfi(self.rw)?;
            // Disable ASM / DSM stat collection for this port, since that
            // data will be collected in the DEV10G instead
            self.modify(ASM().CFG().PORT_CFG(port), |r| {
                r.set_csc_stat_dis(1);
            })?;
            self.modify(DSM().CFG().BUF_CFG(port), |r| {
                r.set_csc_stat_dis(1);
            })?;
            serdes_cfg.apply(dev.index(), self.rw)?;

            self.set_calendar_bandwidth(port, Bandwidth::Bw10G)?;
        }
        Ok(())
    }

    /// Enables 100M SGMII for the given port, using Table 5 in the datasheet to
    /// convert from ports to DEV and SERDES.
    ///
    /// Each value in `ports` must be between 0 and 31, or 48 (the NPI port)
    pub fn init_sgmii(&self, ports: &[u8]) -> Result<(), VscError> {
        let sd1g_cfg = serdes1g::Config::new(serdes1g::Mode::Sgmii);
        let sd6g_cfg = serdes6g::Config::new(serdes6g::Mode::Sgmii);

        for &p in ports {
            assert!(p <= 31 || p == 48);
            let dev_type = match p {
                0..=7 => DevGeneric::new_1g,
                8..=31 | 48 => DevGeneric::new_2g5,
                _ => panic!(),
            };
            let dev = match p {
                0..=7 => p,
                8..=31 => p - 8,
                48 => 24,
                _ => panic!(),
            };
            let dev = dev_type(dev)?;
            assert_eq!(dev.port(), p);

            dev.init_sgmii(self.rw, dev::Speed::Speed100M)?;

            // SERDES1G_1 maps to Port 0, SERDES1G_2 to Port 1, etc
            // SERDES6G_0 maps to Port 8, SERDES6G_1 to Port 9, etc
            // (notice that there's an offset here; SERDES1G_0 is used by the
            //  NPI port, i.e. port 48)
            match p {
                0..=7 => sd1g_cfg.apply(p + 1, self.rw),
                8..=31 => sd6g_cfg.apply(p - 8, self.rw),
                48 => sd1g_cfg.apply(0, self.rw),
                _ => panic!(),
            }?;

            self.set_calendar_bandwidth(p, Bandwidth::Bw1G)?;
        }
        Ok(())
    }

    /// Enables QSGMII mode for blocks of four ports beginning at `start_port`.
    /// This will configure the appropriate DEV1G or DEV2G5 devices, and the
    /// appropriate SERDES6G, based on Table 8 in the datasheet;
    ///
    /// Each value in `start_ports` must be divisible by 4 and below 48;
    /// otherwise, this function will panic.
    pub fn init_qsgmii(&self, start_ports: &[u8]) -> Result<(), VscError> {
        let qsgmii_cfg = serdes6g::Config::new(serdes6g::Mode::Qsgmii);

        // Set a bit to enable QSGMII for these block
        self.modify(HSIO().HW_CFGSTAT().HW_CFG(), |r| {
            let mut e = r.qsgmii_ena();
            for p in start_ports {
                e |= 1 << (p / 4);
            }
            r.set_qsgmii_ena(e);
        })?;
        for &start_port in start_ports {
            assert!(start_port < 48);
            assert_eq!(start_port % 4, 0);

            let (dev_type, start_dev): (fn(u8) -> Result<DevGeneric, _>, u8) =
                match start_port {
                    0 => (DevGeneric::new_1g, 0),
                    4 => (DevGeneric::new_1g, 4),
                    8 => (DevGeneric::new_2g5, 0),
                    12 => (DevGeneric::new_2g5, 4),
                    16 => (DevGeneric::new_2g5, 8),
                    20 => (DevGeneric::new_2g5, 12),
                    24 => (DevGeneric::new_2g5, 16),
                    28 => (DevGeneric::new_2g5, 20),
                    32 => (DevGeneric::new_1g, 8),
                    36 => (DevGeneric::new_1g, 12),
                    40 => (DevGeneric::new_1g, 16),
                    44 => (DevGeneric::new_1g, 20),
                    _ => panic!(),
                };

            // Ports 0-3 use SERDES6G_4, 4-7 use SERDES6G_5, etc
            let serde = (start_port / 4) + 4;

            // Reset the PCS TX clock domain.  In the SDK, this is accompanied
            // by the cryptic comment "BZ23738", which may refer to an errata
            // of some kind?
            for dev in (start_dev + 1)..(start_dev + 4) {
                self.modify(
                    dev_type(dev)?.regs().DEV_CFG_STATUS().DEV_RST_CTRL(),
                    |r| r.set_pcs_tx_rst(0),
                )?;
            }

            qsgmii_cfg.apply(serde, self.rw)?;

            for dev in start_dev..(start_dev + 4) {
                dev_type(dev)?.init_sgmii(self.rw, dev::Speed::Speed100M)?;
            }
            for port in start_port..start_port + 4 {
                self.set_calendar_bandwidth(port, Bandwidth::Bw1G)?;
            }
        }
        Ok(())
    }

    /// Configures a port to run DEV2G5_XX through a 10G SERDES.
    ///
    /// This is only valid for ports 49-52, and will panic otherwise; see
    /// Table 9 for details.
    pub fn init_10g_sgmii(&self, ports: &[u8]) -> Result<(), VscError> {
        let serdes10g_cfg_sgmii =
            serdes10g::Config::new(serdes10g::Mode::Sgmii)?;
        for &port in ports {
            assert!(port >= 49);
            assert!(port <= 52);
            let d2g5 = DevGeneric::new_2g5(port - 24).unwrap();
            let d10g = Dev10g::new(port - 49).unwrap();
            assert!(d2g5.port() == d10g.port());

            // We have to disable and flush the 10G port that shadows this port
            port::port10g_flush(&d10g, self)?;

            // "Configure the 10G Mux mode to DEV2G5"
            self.modify(HSIO().HW_CFGSTAT().HW_CFG(), |r| {
                match d10g.index() {
                    0 => r.set_dev10g_0_mode(3),
                    1 => r.set_dev10g_1_mode(3),
                    2 => r.set_dev10g_2_mode(3),
                    3 => r.set_dev10g_3_mode(3),
                    d => panic!("Invalid DEV10G {}", d),
                }
            })?;
            // This bit must be set when a 10G port runs below 10G speed
            self.modify(DSM().CFG().DEV_TX_STOP_WM_CFG(d2g5.port()), |r| {
                r.set_dev10g_shadow_ena(1);
            })?;
            serdes10g_cfg_sgmii.apply(d10g.index(), self.rw)?;
            d2g5.init_sgmii(self.rw, dev::Speed::Speed100M)?;

            self.set_calendar_bandwidth(port, Bandwidth::Bw1G)?;
        }
        Ok(())
    }

    /// Performs initial configuration (endianness, soft reset, read padding) of
    /// the VSC7448, checks that its chip ID is correct, and brings core systems
    /// out of reset.
    ///
    /// Takes the REFCLK frequency, as well as an optional frequency for
    /// REFCLK2 (used to configure the PLL boost).
    pub fn init(
        &self,
        f1: RefClockFreq,
        f2: Option<RefClockFreq>,
    ) -> Result<(), VscError> {
        // Write the byte ordering / endianness configuration
        self.write(DEVCPU_ORG().DEVCPU_ORG().IF_CTRL(), 0x81818181.into())?;

        // Trigger a soft reset
        self.write_with(DEVCPU_GCB().CHIP_REGS().SOFT_RST(), |r| {
            r.set_soft_chip_rst(1);
        })?;

        // Re-write byte ordering / endianness
        self.write(DEVCPU_ORG().DEVCPU_ORG().IF_CTRL(), 0x81818181.into())?;

        // Configure reads to include padding bytes, since we're reading quickly
        self.write_with(DEVCPU_ORG().DEVCPU_ORG().IF_CFGSTAT(), |r| {
            r.set_if_cfg(spi::SPI_NUM_PAD_BYTES as u32);
        })?;

        let chip_id = self.read(DEVCPU_GCB().CHIP_REGS().CHIP_ID())?;
        if chip_id.rev_id() != 0x3
            || chip_id.part_id() != 0x7468
            || chip_id.mfg_id() != 0x74
            || chip_id.one() != 0x1
        {
            return Err(VscError::BadChipId(chip_id.into()));
        }

        // Core chip bringup, bringing all of the main subsystems out of reset
        // (based on `jr2_init_conf_set` in the SDK)
        self.modify(ANA_AC().STAT_GLOBAL_CFG_PORT().STAT_RESET(), |r| {
            r.set_reset(1)
        })?;
        self.modify(ASM().CFG().STAT_CFG(), |r| r.set_stat_cnt_clr_shot(1))?;
        self.modify(QSYS().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;
        self.modify(REW().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;
        // The VOP isn't in the datasheet, but it's in the SDK
        self.modify(VOP().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;
        self.modify(ANA_AC().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;
        self.modify(ASM().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;
        self.modify(DSM().RAM_CTRL().RAM_INIT(), |r| {
            r.set_ram_init(1);
            r.set_ram_ena(1);
        })?;

        // The RAM initialization should take about 40 µs, according to
        // the datasheet.
        sleep_for(1);

        // Confirm that the RAM_INIT bits have cleared themselves.
        // This should never fail, and there's not much we can do about it
        // if it _does_ fail.
        if self.read(QSYS().RAM_CTRL().RAM_INIT())?.ram_init() != 0
            || self.read(REW().RAM_CTRL().RAM_INIT())?.ram_init() != 0
            || self.read(VOP().RAM_CTRL().RAM_INIT())?.ram_init() != 0
            || self.read(ANA_AC().RAM_CTRL().RAM_INIT())?.ram_init() != 0
            || self.read(ASM().RAM_CTRL().RAM_INIT())?.ram_init() != 0
            || self.read(DSM().RAM_CTRL().RAM_INIT())?.ram_init() != 0
        {
            return Err(VscError::RamInitFailed);
        }

        // Enable the 5G PLL boost on the main clock, and optionally on
        // the secondary clock (if present)
        self.pll5g_setup(0, f1)?;
        if let Some(f2) = f2 {
            self.pll5g_setup(1, f2)?;
        }

        // Enable the queue system
        self.write_with(QSYS().SYSTEM().RESET_CFG(), |r| r.set_core_ena(1))?;

        self.high_speed_mode()?;

        sleep_for(105); // Minimum time between reset and SMI access

        Ok(())
    }

    /// Based on `vtss_lc_pll5g_setup` and various functions that it calls
    fn pll5g_setup(&self, i: u8, freq: RefClockFreq) -> Result<(), VscError> {
        let pll5g = HSIO().PLL5G_CFG(i);
        self.modify(pll5g.PLL5G_CFG4(), |r| {
            r.set_ib_ctrl(0x7600);
        })?;
        let loop_bw_res = match freq {
            RefClockFreq::Clk25MHz => 10,
            RefClockFreq::Clk125MHz => 14,
            RefClockFreq::Clk156p25MHz => 17,
        };
        self.modify(pll5g.PLL5G_CFG0(), |r| {
            r.set_ena_vco_contrh(0);
            r.set_loop_bw_res(loop_bw_res);
            r.set_selbgv820(4);
        })?;
        for _ in 0..=9 {
            self.modify(pll5g.PLL5G_CFG2(), |r| {
                r.set_disable_fsm(1);
            })?;
            self.modify(pll5g.PLL5G_CFG2(), |r| {
                r.set_disable_fsm(0);
            })?;
            sleep_for(10);
            let v = self
                .read(HSIO().PLL5G_STATUS(i).PLL5G_STATUS1())?
                .gain_stat();
            if v > 2 && v < 0xa {
                sleep_for(5);
                return Ok(());
            }
        }
        Err(VscError::LcPllInitFailed(i))
    }

    /// Based on the section of `jr2_init_conf_set` beginning with the comment
    /// "Configuring core clock to run 278MHz"
    ///
    /// In the SDK, this only runs for VTSS_TARGET_SPARX_IV_90, but in
    /// conversations with Microchip support, they say to use this on the
    /// VSC7448 as well (which is nominally a SPARX_IV_80 target)
    fn high_speed_mode(&self) -> Result<(), VscError> {
        for i in 0..2 {
            self.modify(HSIO().PLL5G_CFG(i).PLL5G_CFG0(), |r| {
                r.set_core_clk_div(3);
            })?;
        }
        self.modify(ANA_AC_POL().COMMON_SDLB().DLB_CTRL(), |r| {
            r.set_clk_period_01ns(36);
        })?;
        self.modify(ANA_AC_POL().COMMON_BDLB().DLB_CTRL(), |r| {
            r.set_clk_period_01ns(36);
        })?;
        self.modify(ANA_AC_POL().COMMON_BUM_SLB().DLB_CTRL(), |r| {
            r.set_clk_period_01ns(36);
        })?;
        self.modify(ANA_AC_POL().POL_ALL_CFG().POL_UPD_INT_CFG(), |r| {
            r.set_pol_upd_int(693);
        })?;
        self.modify(LRN().COMMON().AUTOAGE_CFG_1(), |r| {
            r.set_clk_period_01ns(36);
        })?;
        for i in 0..2 {
            self.modify(DEVCPU_GCB().SIO_CTRL(i).SIO_CLOCK(), |r| {
                r.set_sys_clk_period(36);
            })?;
        }
        self.modify(HSCH().HSCH_MISC().SYS_CLK_PER(), |r| {
            r.set_sys_clk_per_100ps(36);
        })?;
        self.modify(VOP().COMMON().LOC_CTRL(), |r| {
            r.set_loc_base_tick_cnt(28);
        })?;
        self.modify(AFI().TTI_TICKS().TTI_TICK_BASE(), |r| {
            r.set_base_len(14444);
        })?;

        Ok(())
    }

    fn set_calendar_bandwidth(
        &self,
        port: u8,
        bw: Bandwidth,
    ) -> Result<(), VscError> {
        self.modify(QSYS().CALCFG().CAL_AUTO(port / 16), |r| {
            let shift = (port % 16) * 2;
            let mut v = r.cal_auto();
            v &= !(0b11 << shift);
            v |= match bw {
                Bandwidth::None => 0b00,
                Bandwidth::Bw1G => 0b01,
                Bandwidth::Bw2G5 => 0b10,
                Bandwidth::Bw10G => 0b11,
            } << shift;
            r.set_cal_auto(v);
        })?;
        Ok(())
    }

    pub fn apply_calendar(&self) -> Result<(), VscError> {
        let mut total_bw_mhz = 0;
        for i in 0..4 {
            let d = self.read(QSYS().CALCFG().CAL_AUTO(i))?.cal_auto();
            for j in 0..16 {
                let v = (d >> (j * 2)) & 0b11;
                let bw = match v {
                    0b00 => Bandwidth::None,
                    0b01 => Bandwidth::Bw1G,
                    0b10 => Bandwidth::Bw2G5,
                    0b11 => Bandwidth::Bw10G,
                    _ => unreachable!(),
                };
                total_bw_mhz += bw.bandwidth_mhz();
            }
        }

        // The chip nominally has 80 Gbps of bandwidth, but the SDK checks
        // against 84 Gbps.  Perhaps this is because we overclock the PLLs;
        // the datasheet mentions that this allows us to exceed 80 Gbps,
        // but doesn't specify exactly how much.
        if total_bw_mhz > 84_000 {
            return Err(VscError::TooMuchBandwidth(total_bw_mhz));
        }

        // "672->671, BZ19678"
        self.modify(QSYS().CALCFG().CAL_CTRL(), |r| {
            r.set_cal_auto_grant_rate(671);
        })?;

        // The SDK configures HSCH:HSCH_MISC.OUTB_SHARE_ENA here, but we're
        // not using CPU ports, so we can skip it

        // Configure CAL_CTRL to use the CAL_AUTO settings
        self.modify(QSYS().CALCFG().CAL_CTRL(), |r| {
            r.set_cal_mode(8);
        })?;

        // Confirm that the config was applied
        if self.read(QSYS().CALCFG().CAL_CTRL())?.cal_auto_error() == 1 {
            Err(VscError::CalConfigFailed)
        } else {
            Ok(())
        }
    }
}

enum Bandwidth {
    None,
    Bw1G,
    Bw2G5,
    Bw10G,
}

impl Bandwidth {
    fn bandwidth_mhz(&self) -> usize {
        match self {
            Self::None => 0,
            Self::Bw1G => 1_000,
            Self::Bw2G5 => 2_500,
            Self::Bw10G => 10_000,
        }
    }
}

/// Sets the frequency of the reference clock.  The specific values are based
/// on the REFCLK_SEL pins.
#[derive(Copy, Clone)]
pub enum RefClockFreq {
    Clk25MHz = 0b100,
    Clk125MHz = 0b000,
    Clk156p25MHz = 0b001,
}
