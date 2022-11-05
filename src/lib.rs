//! Rust Library for the Microchip MCP23X17
//! ========================================
//! In its current incarnation, this only supports I2C but the register
//! map is the same for SPI as well.
//! 
//! Internally, the chip supports a segreggated layout of registers to make
//! two 8 bit GPIO ports or can interleave the registers to emulate one
//! 16 bit GPIO port. This library works on the former layout and so disables
//! setting `BANK` when calling `set_config()`.
//! 
//! ```
//! use linux_hal::I2cdev;
//! use mcp23x17::{
//!     Mcp23x17 as Expander,  
//!     Port
//! };
//! 
//! fn main() -> Result<(), Box<Error>> {
//!     let i2c = I2cdev::new("/dev/i2c-1")?;
//!     let mut exp = Expander::new(i2c)?;
//! 
//!     exp.select_port(Port::B);
//!     exp.set_direction(0x00)?;
//!     exp.set_data(0xff)?;
//! }
//! ```
//! 
//! Implementation details taken from
//! http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf
//! 
//! ## Notes:
//! When wiring up the chip, you *must* pull reset high. Ideally pull
//! SCL and SDA high with 1Kohm resistors
//! 
//! For some reason during testing, the only way to bring the chip
//! out an unusual state after reset was to set the value of OLAT on Port A
//! to 0xff. No other registers seemed to make a difference, inclusing OLAT
//! on Port B. This state caused problems not just for Rust but `i2cset` as
//! well.


#![no_std]
#![deny(missing_docs)]
//#![deny(warnings)]

// TODO: Learn how to use macros! Silly amounts of code duplication here

extern crate embedded_hal as hal;
extern crate bitflags;

// use hal::blocking::i2c::{
//     Write,
//     WriteRead
// };
use hal::{blocking::i2c, blocking::spi};
use bitflags::bitflags;

/// IO Direction. 1 = input, Default 0xff
const REG_IODIR: u8 = 0x00;
/// Input polarity inversion. 1 = invert logic
const REG_IPOL: u8 = 0x01;
/// interrupt on change. 1 = enabled
const REG_GPINTEN: u8 = 0x02;
/// Comparison for interrupts
const REG_DEFVAL: u8 = 0x03;
/// Interrupt on change configuration.
const REG_INTCON: u8 = 0x04;
/// Chip configuration
const REG_CONFIG: u8 = 0x05;
/// Internal 100KOhm pull-up resistors. 1 = enabled
const REG_GPPU: u8 = 0x06;
/// Interrupt flag
const REG_INTF: u8 = 0x07;
/// Interrupt captured value
const REG_INTCAP: u8 = 0x08;
/// General Purpose IO value. 1 = high
const REG_GPIO: u8 = 0x09;
/// Output latch. 1 = high
const REG_OLAT: u8 = 0x0A;

/// Device address
pub const ADDRESS: u8 = 0x20;


/// Which port we're actively using. Currently you must select which is
/// active by using select_port on `mcp23x17`. Port A is the default.
pub enum Port {
    /// Port A
    A,
    /// Port B
    B,
}

bitflags! {
    /// Configuration register definition. This register is mirrored
    /// in the register map and pertains to all ports
    pub struct Config: u8 {
        /// If true, interleave port A/B register locations
        const BANK = 1 << 7;
        /// If true, connect interrupt pins
        const MIRROR = 1 << 6;
        /// If true, automatically increment address pointer
        const SEQOP = 1 << 5;
        /// If true, enable slew rate
        const DISSLW = 1 << 4;
        /// If true, use address pins (MCP23S17 only)
        const HAEN = 1 << 3;
        /// If true, output is open-drain 
        const ODR = 1 << 2;
        /// If true, interrupt pins are active high
        const INTPOL = 1 << 1;
        /// Just stops me from looking dumb! This bit is "unimplemented" :D
        const _nothin = 1 << 0;
    }
}

/// 16-bit GPIO Expander (I2C version)
pub struct Mcp23x17<Bus> {
    bus: Bus,
    active_port: Port,
}

/// 16-bit GPIO Expander (SPI version)
pub struct Mcp23s17<Bus> {
    bus: Bus,
    active_port: Port,
}

/**
 * Get/Set trait for any bus. Returns Error 'E' on failed operations.
 */
pub trait GetSetThing {
    /// Error to be returned by I/O instructions
    type Error;
    /// Set i2c/spi/... register to data
    fn set_thing(&mut self, register: u8, data: u8) -> Result<(), Self::Error>;
    /// Get i2c/spi/... register
    fn get_thing(&mut self, register: u8) -> Result<u8, Self::Error>;
}

// trait GetPort {
//     fn get_port(&self, register: u8) -> u8;
// }

// impl<Bus> GetPort for Mcp23x17<Bus> {
//     /// Some quick math for the current register
//     fn get_port(&self, register: u8) -> u8 {
//         match &self.active_port {
//             Port::A => register,
//             Port::B => 0x10 | register
//         }
//     }
// }

/// Functions to be implemented to be able to be treated as an MCP23X17
pub trait Mcp23x17Ops<Bus, E>: GetSetThing<Error = E>
{
    /// Create a new instance of the GPIO expander
    fn new(bus: Bus) -> Result<Self, E> where Self: Sized;

    /// Some quick math for the current register
    fn get_port(&self, register: u8) -> u8;

    /// This chip optionally splits its registers between two eight bit ports
    /// or virtuall one large 16 bit port. This function sets the port
    /// internally.
    fn select_port(&mut self, port: Port);

    /// Set the I/O direction for the currently active port. A value
    /// of 1 is for input, 0 for output
    fn set_direction(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_IODIR, data)?)
    }

    /// Get the I/O direction for the active port
    fn direction(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_IODIR)?)
    }

    /// Set configuration register. Given the structure of this library and how
    /// the chip can rearrange its registers, any attempt to set the `BANK` bit
    /// will be masked to zero.
    fn set_config(&mut self, data: Config) -> Result<(), E> {
        // Safety mechanism to avoid breaking the calls made in the library
        let data = data.bits & !Config::BANK.bits;

        Ok(self.set_thing(REG_CONFIG, data)?)
    }

    /// Read the data state from the active port
    fn config(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_CONFIG)?)
    }

    /// Set the pullups. A value of 1 enables the 100KOhm pullup.
    fn set_pullups(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_GPPU, data)?)
    }

    /// Get the pullups.
    fn pullups(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_GPPU)?)
    }

    /// Read interrupt state. Each pin that caused an interrupt will have
    /// a bit is set. Not settable.
    /// 
    /// The value will be reset after a read from `data_at_interrupt` or
    /// `data()`.
    fn who_interrupted(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_INTF)?)
    }

    /// GPIO value at time of interrupt. It will remain latched to this value
    /// until another interrupt is fired. While it won't reset on read, it does
    /// reset the interrupt state on the corresponding interrupt output pin
    fn data_at_interrupt(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_INTCAP)?)
    }

    /// Set a comparison value for the interrupts. The interrupt will
    /// fire if the input value is *different* from what is set here
    fn set_int_compare(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_DEFVAL, data)?)
    }

    /// Read interrupt comparison value. Check `set_int_compare()` for more
    /// details
    fn int_compare(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_DEFVAL)?)
    }

    /// Decide how interrupts will fire. If a bit is set, the input data
    /// is compared against what's set by `int_compare()`. If unset, the
    /// interrupt will fire when the pin has changed.
    fn set_int_control(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_INTCON, data)?)
    }

    /// Read how interrupts will fire. More details on `set_int_control()`.
    fn int_control(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_INTCON)?)
    }

    /// Enable interrupts. If a bit is set, a change on this pin will trigger an
    /// interrupt. You'll also need to call `set_int_compare()` and
    /// `set_int_control()`
    fn set_interrupt(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_GPINTEN, data)?)
    }

    /// Read the data state from the active port. See `set_interrupt()` for
    /// more details
    fn interrupt(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_GPINTEN)?)
    }

    /// Read output latches. This essentially reads the values set from
    /// calling `set_data()`
    fn latches(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_OLAT)?)
    }

    /// Set polarity allows inverting the values from input pins. A
    /// value of 1 will flip the polarity.
    fn set_polarity(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_IPOL, data)?)
    }

    /// Read the data state from the active port
    fn polarity(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_IPOL)?)
    }

    /// Set the data for the active port
    fn set_data(&mut self, data: u8) -> Result<(), E> {
        Ok(self.set_thing(REG_GPIO, data)?)
    }

    /// Read the data state from the active port
    fn data(&mut self) -> Result<u8, E> {
        Ok(self.get_thing(REG_GPIO)?)
    }
}

impl<Bus, E> Mcp23x17Ops<Bus, E> for Mcp23s17<Bus>
where
    Bus: spi::Transfer<u8, Error = E>
{
    fn new(bus: Bus) -> Result<Self, E> where Self: Sized {
        let mut mcp = Self {
            bus,
            active_port: Port::A
        };
        let mut data_tx = [0x41, 1, 0];
        let val = match mcp.bus.transfer(&mut data_tx) {
            Ok(val)  => val,
            Err(err) => return Err(err),
        };
        // Check if second register is IODIRB,
        // if yes, we're likely in BANK = 0
        if val[2] == 0xff {
            // We require BANK = 1
            // So we set it
            let mut data_tx = [0x40, 0x0a, 0b1000_0000];
            mcp.bus.transfer(&mut data_tx)?;
        }

        Ok(mcp)
    }

    /// Some quick math for the current register
    fn get_port(&self, register: u8) -> u8 {
        match &self.active_port {
            Port::A => register,
            Port::B => 0x10 | register
        }
    }

    fn select_port(&mut self, port: Port) {
        self.active_port = port;
    }
}

impl<Bus, E> Mcp23x17Ops<Bus, E> for Mcp23x17<Bus>
where
    Bus: i2c::WriteRead<Error = E> + i2c::Write<Error = E>
{
    fn new(bus: Bus) -> Result<Self, E> where Self: Sized {
        Ok(Self {
            bus,
            active_port: Port::A
        })
    }

    /// Some quick math for the current register
    fn get_port(&self, register: u8) -> u8 {
        match &self.active_port {
            Port::A => register,
            Port::B => 0x10 | register
        }
    }

    fn select_port(&mut self, port: Port) {
        self.active_port = port;
    }
}


impl<SPIBus, E> GetSetThing for Mcp23s17<SPIBus>
where
    SPIBus: spi::Transfer<u8, Error = E>,
    Mcp23s17<SPIBus>: Mcp23x17Ops<SPIBus, E>
{
    type Error = E;
    /// Helper function to save my typing when setting
    fn set_thing(&mut self, register: u8, data: u8) -> Result<(), Self::Error> {
        let reg = Self::get_port(self,register);
        let mut data_tx = [0x40, reg, data];

        Ok({
            self.bus.transfer(&mut data_tx)?;
            ()
        })
    }

    /// Helper function to save my typing when reading
    fn get_thing(&mut self, register: u8) -> Result<u8, E> {
        let reg = self.get_port(register);
        let mut data = [0x41, reg, 0];

        self.bus.transfer(&mut data)?;
        Ok(data[2])
    }
}


impl<I2CBus, E> GetSetThing for Mcp23x17<I2CBus>
where
    I2CBus: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
    Mcp23x17<I2CBus>: Mcp23x17Ops<I2CBus, E>
{
    type Error = E;
    /// Helper function to save my typing when setting
    fn set_thing(&mut self, register: u8, data: u8) -> Result<(), E> {
        let reg = self.get_port(register);

        Ok(self.bus.write(ADDRESS, &[reg, data])?)
    }

    /// Helper function to save my typing when reading
    fn get_thing(&mut self, register: u8) -> Result<u8, E> {
        let reg = self.get_port(register);
        let mut data = [0u8; 1];

        self.bus.write_read(ADDRESS, &[reg], &mut data)?;
        Ok(data[0])
    }
}