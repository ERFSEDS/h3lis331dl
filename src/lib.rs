#![cfg_attr(not(test), no_std)]
extern crate embedded_hal as hal;

use hal::blocking::spi::{Transfer, Write};
use hal::digital::v2::OutputPin;

const DEFAULT_WHO_AM_I: u8 = 0x32;
const WHO_AM_I: Reg = Reg(0x0F);
const CTRL_REG1: Reg = Reg(0x20);
const CTRL_REG2: Reg = Reg(0x21);
const CTRL_REG3: Reg = Reg(0x22);
const CTRL_REG4: Reg = Reg(0x23);
const CTRL_REG5: Reg = Reg(0x24);
const HP_FILTER_RESET: Reg = Reg(0x25);
const REFERENCE: Reg = Reg(0x26);
const STATUS_REG: Reg = Reg(0x27);
const OUT_X_L: Reg = Reg(0x28);
const OUT_X_H: Reg = Reg(0x29);
const OUT_Y_L: Reg = Reg(0x2A);
const OUT_Y_H: Reg = Reg(0x2B);
const OUT_Z_L: Reg = Reg(0x2C);
const OUT_Z_H: Reg = Reg(0x2D);
const INT1_CFG: Reg = Reg(0x30);
const INT1_SOURCE: Reg = Reg(0x31);
const INT1_THS: Reg = Reg(0x32);
const INT1_DURATION: Reg = Reg(0x33);
const INT2_CFG: Reg = Reg(0x34);
const INT2_SOURCE: Reg = Reg(0x35);
const INT2_THS: Reg = Reg(0x36);
const INT2_DURATION: Reg = Reg(0x37);

#[derive(Debug, Copy, Clone)]
struct Reg(u8);

#[repr(u8)]
enum comm_mode {
    USE_I2C = 0,
    USE_SPI = 1,
}

#[repr(u8)]
enum power_mode {
    POWER_DOWN = 0,
    NORMAL = 1,
    LOW_POWER_0_5HZ = 2,
    LOW_POWER_1HZ = 3,
    LOW_POWER_2HZ = 4,
    LOW_POWER_5HZ = 5,
    LOW_POWER_10HZ = 6,
}

#[repr(u8)]
enum data_rate {
    DR_50HZ = 0,
    DR_100HZ = 1,
    DR_400HZ = 2,
    DR_1000HZ = 3,
}

#[repr(u8)]
enum high_pass_cutoff_freq_cfg {
    HPC_8 = 0,
    HPC_16 = 1,
    HPC_32 = 2,
    HPC_64 = 3,
}

#[repr(u8)]
enum pp_od {
    PUSH_PULL = 0,
    OPEN_DRAIN = 1,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum int_sig_src {
    INT_SRC = 0,
    INT1_2_SRC = 1,
    DRDY = 2,
    BOOT = 3,
}

#[repr(u8)]
enum fs_range {
    LOW_RANGE = 0,
    MED_RANGE = 1,
    NO_RANGE = 2,
    HIGH_RANGE = 3,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum int_axis {
    X_AXIS = 0,
    Y_AXIS = 1,
    Z_AXIS = 2,
}

#[repr(u8)]
enum trig_on_level {
    TRIG_ON_HIGH = 0,
    TRIG_ON_LOW = 1,
}

// H3LIS331DL
pub struct H3LIS331DL<SPI, NCS, SpiE, CsE>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    NCS: OutputPin<Error = CsE>,
{
    spi: SPI,
    ncs: NCS,
}

#[derive(Debug)]
pub enum Error<SpiE, CsE> {
    /// SPI Communication error
    SPI(SpiE),
    /// Chip-select pin error (SPI)
    CS(CsE),
    ///
    InvalidWhoAmI(u8),
    RegisterTest,
}

impl<SPI, NCS, SpiE, CsE> H3LIS331DL<SPI, NCS, SpiE, CsE>
where
    SPI: Transfer<u8, Error = SpiE> + Write<u8, Error = SpiE>,
    NCS: OutputPin<Error = CsE>,
{
    ///Creates a new H3LIS331DL driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, ncs: NCS) -> Result<H3LIS331DL<SPI, NCS, SpiE, CsE>, Error<SpiE, CsE>> {
        let mut h3lis331dl = H3LIS331DL { spi, ncs };

        // FIXME: Remove unwrap once we have completed initial testing
        h3lis331dl.cs_enable()?;
        let whoami = h3lis331dl.read_reg(WHO_AM_I)?;
        h3lis331dl.cs_disable()?;

        if whoami != DEFAULT_WHO_AM_I {
            return Err(Error::InvalidWhoAmI(whoami));
        } else {
            return Err(Error::RegisterTest);
        }

        // TODO: look into using boot mode on the device.
        // See 7.3 CTRL_REG2 on the datasheet:
        // https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
        let zero: [u8; 1] = [0];

        let registers = [
            CTRL_REG2,
            CTRL_REG3,
            CTRL_REG4,
            CTRL_REG5,
            INT1_CFG,
            INT1_SOURCE,
            INT1_THS,
            INT1_DURATION,
            INT2_CFG,
            INT2_SOURCE,
            INT2_THS,
            INT2_DURATION,
        ];

        // Zero out above registers
        for register in &registers {
            h3lis331dl.H3LIS331DL_write(*register, &zero)?;
        }

        // Check to make sure the writes worked
        for register in &registers {
            let mut real_value: [u8; 1] = [0];
            h3lis331dl.H3LIS331DL_read(*register, &mut real_value)?;
            if real_value != zero {
                return Err(Error::RegisterTest);
            }
        }

        h3lis331dl.setPowerMode(power_mode::NORMAL)?;
        h3lis331dl.axisEnable(true)?;

        Ok(h3lis331dl)
    }

    fn setPowerMode(&mut self, mode: power_mode) -> Result<(), Error<SpiE, CsE>> {
        // let mut data: u8;
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG1, &mut data)?;

        // The power mode is the high three bits of CTRL_REG1. The mode
        //  constants are the appropriate bit values left shifted by five, so we
        //  need to right shift them to make them work. We also want to mask off the
        //  top three bits to zero, and leave the others untouched, so we *only*
        //  affect the power mode bits.
        data[0] &= !0xe0; // Clear the top three bits
        data[0] |= (mode as u8) << 5; // set the top three bits to our pmode value
                                      // TODO: Use Rust register abstractions to enable code like
                                      // `self.ctrl_reg().pmode(PowerMode::Whatever)`
                                      // That way we avoid manual bit manipulation
        self.H3LIS331DL_write(CTRL_REG1, &mut data) // write the new value to CTRL_REG1
    }

    pub fn axisEnable(&mut self, enable: bool) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG1, &mut data)?;
        if enable {
            data[0] |= 0x07;
        } else {
            data[0] &= !0x07;
        }
        self.H3LIS331DL_write(CTRL_REG1, &mut data)
    }

    fn setODR(&mut self, drate: data_rate) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG1, &mut data)?;

        // The data rate is bits 4:3 of CTRL_REG1. The data rate constants are the
        //  appropriate bit values; we need to right shift them by 3 to align them
        //  with the appropriate bits in the register. We also want to mask off the
        //  top three and bottom three bits, as those are unrelated to data rate and
        //  we want to only change the data rate.
        data[0] &= !0x18; // Clear the two data rate bits
        data[0] |= (drate as u8) << 3; // Set the two data rate bits appropriately.
        self.H3LIS331DL_write(CTRL_REG1, &mut data) // write the new value to CTRL_REG1
    }

    //FIXME
    // Return struct of values instead of passing mutable references
    pub fn readAxes(
        &mut self,
        x: &mut i16,
        y: &mut i16,
        z: &mut i16,
    ) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 6] = [0, 0, 0, 0, 0, 0];

        // LIS331_read(OUT_X_L, &data[0], 1);
        // FIXME: use seperate variables for each axis, and check bit math below. The or looks
        // wrong
        // Also look into starting one read and having the device send back mutiple registers of
        // data based on straight register values
        self.H3LIS331DL_read(OUT_X_L, &mut data[0..1])?;
        self.H3LIS331DL_read(OUT_X_H, &mut data[1..2])?;
        self.H3LIS331DL_read(OUT_Y_L, &mut data[2..3])?;
        self.H3LIS331DL_read(OUT_Y_H, &mut data[3..4])?;
        self.H3LIS331DL_read(OUT_Z_L, &mut data[4..5])?;
        self.H3LIS331DL_read(OUT_Z_H, &mut data[5..6])?;
        // The data that comes out is 12-bit data, left justified, so the lower
        //  four bits of the data are always zero. We need to right shift by four,
        //  then typecase the upper data to an integer type so it does a signed
        //  right shift.
        *x = ((data[0] | data[1]) as i16) << 8;
        *y = ((data[2] | data[3]) as i16) << 8;
        *z = ((data[4] | data[5]) as i16) << 8;
        *x = *x >> 4;
        *y = *y >> 4;
        *z = *z >> 4;
        Ok(())
    }

    fn cs_enable(&mut self) -> Result<(), Error<SpiE, CsE>> {
        self.ncs.set_low().map_err(|e| Error::CS(e))
    }

    fn cs_disable(&mut self) -> Result<(), Error<SpiE, CsE>> {
        self.ncs.set_high().map_err(|e| Error::CS(e))
    }

    fn read_reg(&mut self, register: Reg) -> Result<u8, Error<SpiE, CsE>> {
        // 0xC0 = 0b1000_0000
        let mut data: [u8; 1] = [register.0 | 0b1000_0000];
        let b = self.spi.transfer(&mut data).map_err(|e| Error::SPI(e))?;
        Ok(b[0])
    }

    fn H3LIS331DL_read(
        &mut self,
        reg_address: Reg,
        data: &mut [u8],
    ) -> Result<(), Error<SpiE, CsE>> {
        // SPI read handling code
        data[0] = reg_address.0 | 0xC0; //0b1100_0000
        self.ncs.set_low().map_err(|e| Error::CS(e))?;
        self.spi.transfer(data).map_err(|e| Error::SPI(e))?;
        self.ncs.set_high().map_err(|e| Error::CS(e))?;
        Ok(())
    }

    fn convertToG(&mut self, maxScale: i32, reading: i32) -> f32 {
        let result = ((maxScale as f32) * (reading as f32)) / (2047 as f32);
        result
    }

    fn setHighPassCoeff(
        &mut self,
        hpcoeff: high_pass_cutoff_freq_cfg,
    ) -> Result<(), Error<SpiE, CsE>> {
        // The HPF coeff depends on the output data rate. The cutoff frequency is
        //  is approximately fs/(6*HPc) where HPc is 8, 16, 32 or 64, corresponding
        //  to the various constants available for this parameter.
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG2, &mut data)?;
        data[0] &= !0xfc; // Clear the two low bits of the CTRL_REG2
        data[0] |= hpcoeff as u8;
        self.H3LIS331DL_write(CTRL_REG2, &mut data)
    }

    pub fn enableHPF(&mut self, enable: bool) -> Result<(), Error<SpiE, CsE>> {
        // Enable the high pass filter
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG2, &mut data)?;
        if enable {
            data[0] |= 1 << 5;
        } else {
            data[0] &= !(1 << 5);
        }
        self.H3LIS331DL_write(CTRL_REG2, &mut data)
    }

    fn HPFOnIntPin(&mut self, enable: bool, pin: u8) -> Result<(), Error<SpiE, CsE>> {
        // Enable the hpf on signal to int pins
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG2, &mut data)?;
        if enable {
            if pin == 1 {
                data[0] |= 1 << 3;
            }
            if pin == 2 {
                data[0] |= 1 << 4;
            }
        } else {
            if pin == 1 {
                data[0] &= !1 << 3;
            }
            if pin == 2 {
                data[0] &= !1 << 4;
            }
        }
        self.H3LIS331DL_write(CTRL_REG2, &mut data)
    }

    fn intActiveHigh(&mut self, enable: bool) -> Result<(), Error<SpiE, CsE>> {
        // Are the int pins active high or active low?
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG3, &mut data)?;
        // Setting bit 7 makes int pins active low
        if !enable {
            data[0] |= 1 << 7;
        } else {
            data[0] &= !(1 << 7);
        }
        self.H3LIS331DL_write(CTRL_REG3, &mut data)
    }

    fn intPinMode(&mut self, _pinMode: pp_od) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG3, &mut data)?;
        // Setting bit 6 makes int pins open drain.
        if (_pinMode as u8) == (pp_od::OPEN_DRAIN as u8) {
            data[0] |= 1 << 6;
        } else {
            data[0] &= !(1 << 6);
        }
        self.H3LIS331DL_write(CTRL_REG3, &mut data)
    }

    fn latchInterrupt(&mut self, enable: bool, intSource: u8) -> Result<(), Error<SpiE, CsE>> {
        // Latch mode for interrupt. When enabled, you must read the INTx_SRC reg
        //  to clear the interrupt and make way for another.
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG3, &mut data)?;
        // Enable latching by setting the appropriate bit.
        if enable {
            if intSource == 1 {
                data[0] |= 1 << 2;
            }
            if intSource == 2 {
                data[0] |= 1 << 5;
            }
        } else {
            if intSource == 1 {
                data[0] &= !1 << 2;
            }
            if intSource == 2 {
                data[0] &= !1 << 5;
            }
        }
        self.H3LIS331DL_write(CTRL_REG3, &mut data)
    }

    fn intSrcConfig(&mut self, src: int_sig_src, pin: u8) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(CTRL_REG3, &mut data)?;
        // Enable latching by setting the appropriate bit.
        if pin == 1 {
            data[0] &= !0xfc; // clear the low two bits of the register
            data[0] |= src as u8;
        }
        if pin == 2 {
            data[0] &= !0xe7; // clear bits 4:3 of the register
            data[0] |= (src as u8) << 4;
        }
        self.H3LIS331DL_write(CTRL_REG3, &mut data)
    }

    fn setFullScale(&mut self, range: fs_range) -> Result<(), Error<SpiE, CsE>> {
        let mut data = self.read_byte(CTRL_REG4)?;
        data &= !0xcf;
        data |= (range as u8) << 4;
        self.write_byte(CTRL_REG4, data)
    }

    fn newData_impl(&mut self) -> Result<u8, Error<SpiE, CsE>> {
        self.read_byte(STATUS_REG)
    }

    //FIXME. Return opaue status struct that has methods for this so we only have to do one read
    fn newXData(&mut self) -> Result<bool, Error<SpiE, CsE>> {
        Ok(((self.newData_impl()? >> 0) & 1) == 1)
    }

    fn newYData(&mut self) -> Result<bool, Error<SpiE, CsE>> {
        Ok(((self.newData_impl()? >> 1) & 1) == 1)
    }

    fn newZData(&mut self) -> Result<bool, Error<SpiE, CsE>> {
        Ok(((self.newData_impl()? >> 2) & 1) == 1)
    }

    fn enableInterrupt(
        &mut self,
        axis: int_axis,
        trigLevel: trig_on_level,
        //FIXME: Use enum for this
        interrupt: u8,
        enable: bool,
    ) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        let (mut reg, mut mask): (Reg, u8);
        mask = 0;
        if interrupt == 1 {
            reg = INT1_CFG;
        } else {
            reg = INT2_CFG;
        }
        self.H3LIS331DL_read(reg, &mut data)?;
        if (trigLevel as u8) == (trig_on_level::TRIG_ON_HIGH as u8) {
            mask = 1 << 1;
        } else {
            mask = 1;
        }
        if (axis as u8) == (int_axis::Z_AXIS as u8) {
            mask = mask << 4;
        }
        if (axis as u8) == (int_axis::Y_AXIS as u8) {
            mask = mask << 2;
        }
        if enable {
            data[0] |= mask;
        } else {
            data[0] &= !mask;
        }
        self.H3LIS331DL_write(reg, &mut data)
    }

    fn setIntDuration(&mut self, duration: u8, intSource: u8) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        data[0] = duration;
        let reg = if intSource == 1 {
            //FIXME: Use enum for this
            INT1_DURATION
        } else {
            INT2_DURATION
        };
        self.H3LIS331DL_write(reg, &mut data)
    }

    fn setIntThreshold(&mut self, threshold: u8, intSource: u8) -> Result<(), Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        let reg = if intSource == 1 {
            //FIXME: Use enum for this
            INT1_THS
        } else {
            INT2_THS
        };
        self.write_byte(reg, threshold)
    }

    fn read_byte(&mut self, reg_address: Reg) -> Result<u8, Error<SpiE, CsE>> {
        let mut data: [u8; 1] = [0];
        self.H3LIS331DL_read(reg_address, &mut data)?;
        Ok(data[0])
    }

    fn H3LIS331DL_write(&mut self, reg_address: Reg, data: &[u8]) -> Result<(), Error<SpiE, CsE>> {
        // SPI write handling code
        self.ncs.set_low().map_err(|e| Error::CS(e))?;
        self.spi
            .write(&[reg_address.0 | 0x40])
            .map_err(|e| Error::SPI(e))?; //0b0100_0000
        self.spi.write(&data).map_err(|e| Error::SPI(e))?;
        self.ncs.set_high().map_err(|e| Error::CS(e))?;
        Ok(())
    }

    fn write_byte(&mut self, reg_address: Reg, value: u8) -> Result<(), Error<SpiE, CsE>> {
        let data: [u8; 1] = [value];
        self.H3LIS331DL_write(reg_address, &data)
    }
}
