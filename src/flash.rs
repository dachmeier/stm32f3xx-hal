use core::{fmt::Debug, mem::size_of};

use crate::pac::{flash, FLASH};

pub const FLASH_START: usize = 0x0800_0000;
pub const FLASH_END_MAX: usize = 0x0807_FFFF;

pub const SZ_1K: usize = 1024;


pub trait FlashWriteErase {
    type WriteEraseError: Debug;

    unsafe fn page_erase(&mut self, start_offset: usize) -> Result<(), Self::WriteEraseError>;

    /// Writes bytes in data slice to flash, starting from offset.
    /// The implementation has to check that data.len is an integer multiple of the smallest possible write.
    /// If not it should return LengthNotMultipleSmallest Error
    unsafe fn write(&mut self, offset: usize, buf: &[u8]) -> Result<(), Self::WriteEraseError>;
}

pub trait FlashRead {
    type ReadError: Debug;
    unsafe fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), Self::ReadError>;
}

pub struct FlashLayout {
    pub flash_size: usize,
    pub sector_size: usize,
}

// #[cfg(feature = "flash-256_2")]
pub const FLASH_LAYOUT: FlashLayout = FlashLayout {
    flash_size: 256 * SZ_1K,
    sector_size: 2 * SZ_1K,
};



#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum Error {
    OutOfBounds,
    OffsetOutOfBounds,
    AddressMisaligned,
    LengthNotMultipleSmallest,
    LengthTooLong,
    EraseError,
    ProgrammingError,
    WriteError,
    VerifyError,
    UnlockError,
    LockError,
}


pub struct FlashWriter<'a> {
    flash: &'a mut Parts,
}

impl<'a> FlashWriter<'a> {
    const KEY1: u32 = 0x45670123;
    const KEY2: u32 = 0xCDEF89AB;

    const WRITESIZE: usize = 2;

    fn valid_write_length(&self, offset: usize, length: usize) -> Result<(), Error> {
        if offset + length > FLASH_LAYOUT.flash_size {
            Err(Error::LengthTooLong)
        } else if offset % Self::WRITESIZE != 0 {
            Err(Error::AddressMisaligned)
        // } else if length % Self::WRITESIZE != 0 {
        //     Err(Error::LengthNotMultipleSmallest)
        } else {
            Ok(())
        }
    }

    fn valid_read_length(&self, offset: usize, length: usize) -> Result<(), Error> {
        if offset + length > FLASH_LAYOUT.flash_size {
            Err(Error::OutOfBounds)
        } else {
            Ok(())
        }
    }

    fn lock(&mut self) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        // why use modify instead of write here? (copied from stm32f1xx hal)
        self.flash.cr.cr().modify(|_, w| w.lock().lock());

        match self.flash.cr.cr().read().lock().is_locked() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn unlock(&mut self) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.keyr.keyr().write(|w| w.fkeyr().bits(Self::KEY1));

        self.flash.keyr.keyr().write(|w| w.fkeyr().bits(Self::KEY2));

        match self.flash.cr.cr().read().lock().is_unlocked() {
            true => Ok(()),
            false => Err(Error::UnlockError),
        }
    }

    fn is_locked(&mut self) -> bool {
        self.flash.cr.cr().read().lock().is_locked()
    }

    fn write_word(&mut self, word: u16, destination: *mut u16) -> Result<(), Error> {
        unsafe {
            core::ptr::write_volatile(destination, word);
        };

        while self.flash.sr.sr().read().bsy().bit_is_set() {}

        if self.flash.sr.sr().read().eop().bit_is_set() {
            self.flash.sr.sr().write(|w| w.eop().clear_bit());
        } else {
            self.lock()?;
            return Err(Error::ProgrammingError);
        }

        if word != unsafe { core::ptr::read_volatile(destination) } {
            self.lock()?;
            return Err(Error::VerifyError);
        }
        Ok(())
    }
}

impl<'a> FlashWriteErase for FlashWriter<'a> {
    type WriteEraseError = Error;

    

    // TODO: check what kind of addresses work. Only start address of each page? not stated in RM
    unsafe fn page_erase(&mut self, start_offset: usize) -> Result<(), Self::WriteEraseError> {
        if start_offset >= FLASH_LAYOUT.flash_size {
            return Err(Error::OffsetOutOfBounds);
        }

        self.unlock()?;

        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.cr.cr().modify(|_, w| w.per().set_bit());


        self.flash
            .ar
            .ar()
            .write(|w| w.far().bits((FLASH_START + start_offset) as u32));


        self.flash.cr.cr().modify(|_, w| w.strt().set_bit());

        // the stm32f3xx reference manual states having to wait at least one cpu cycle before checking bsy
        cortex_m::asm::delay(1);

        while self.flash.sr.sr().read().bsy().is_active() {}

        if self.flash.sr.sr().read().eop().bit_is_set() {
            self.flash.sr.sr().write(|w| w.eop().clear_bit());
            Ok(())
        } else {
            Err(Error::EraseError)
        }
    }

    

    unsafe fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), Self::WriteEraseError> {
        self.valid_write_length(offset, data.len())?;

        self.unlock()?;
        let base_address = FLASH_START + offset;

        for idx in (0..(data.len() - 1)).step_by(2) {
            let write_address =
                (base_address + idx) as *mut u16;

            self.flash.cr.cr().write(|w| w.pg().set_bit());

            let to_write: u16 = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

            self.write_word(to_write, write_address)?;
            // unsafe {
            //     core::ptr::write_volatile(write_address, to_write);
            // };

            // while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // if self.flash.sr.sr().read().eop().bit_is_set() {
            //     self.flash.sr.sr().write(|w| w.eop().clear_bit());
            // } else {
            //     self.lock()?;
            //     return Err(Error::ProgrammingError);
            // }

            // if to_write != unsafe { core::ptr::read_volatile(write_address) } {
            //     self.lock()?;
            //     return Err(Error::VerifyError);
            // }
        }

        let uneven: bool = data.len() % Self::WRITESIZE != 0;

        if uneven {
            let to_write = *(data.last().unwrap()) as u16 | (0xFF as u16) << 8;

            // TODO: think about this again, is -1 correct?
            let address = (base_address + data.len() - 1) as *mut u16;

            self.write_word(to_write, address)?;


            // core::ptr::write_volatile(address, to_write);

            // while self.flash.sr.sr().read().bsy().bit_is_set() {}

            // if self.flash.sr.sr().read().eop().bit_is_set() {
            //     self.flash.sr.sr().write(|w| w.eop().clear_bit());
            // } else {
            //     self.lock()?;
            //     return Err(Error::ProgrammingError);
            // }

            // if to_write != core::ptr::read_volatile(address) {
            //     self.lock()?;
            //     return Err(Error::VerifyError);
            // }
        }

        self.lock()?;
        Ok(())
    }
}


impl<'a> FlashRead for FlashWriter<'a> {
    type ReadError = Error;

    unsafe fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), Self::ReadError> {
        self.valid_read_length(offset, buf.len())?;

        let address = (FLASH_START + offset) as *const _;

        // NOTE(unsafe) read with no side effects. The data returned will
        // remain valid for its lifetime because we take an immutable
        // reference to this FlashWriter, and any operation that would
        // invalidate the data returned would first require taking a mutable
        // reference to this FlashWriter.
        unsafe { buf.copy_from_slice(core::slice::from_raw_parts(address, buf.len())); }
        Ok(())
    }
}


pub trait FlashExt {
    fn constrain(self) -> Parts;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Parts {
        Parts {
            acr: ACR { _0: () },
            ar: AR { _0: () },
            cr: CR { _0: () },
            keyr: KEYR { _0: () },
            _obr: OBR { _0: () },
            _optkeyr: OPTKEYR { _0: () },
            sr: SR { _0: () },
            _wrpr: WRPR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
pub struct Parts {
    /// Opaque ACR register
    pub acr: ACR,

    /// Opaque AR register
    pub(crate) ar: AR,

    /// Opaque CR register
    pub(crate) cr: CR,

    /// Opaque KEYR register
    pub(crate) keyr: KEYR,

    /// Opaque OBR register
    pub(crate) _obr: OBR,

    /// Opaque OPTKEYR register
    pub(crate) _optkeyr: OPTKEYR,

    /// Opaque SR register
    pub(crate) sr: SR,

    /// Opaque WRPR register
    pub(crate) _wrpr: WRPR,
}
impl Parts {
    pub fn writer(&mut self) -> FlashWriter {
        FlashWriter {
            flash: self,
        }
    }
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

#[allow(dead_code)]
impl ACR {
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }
}

/// Opaque AR register
pub struct AR {
    _0: (),
}

#[allow(dead_code)]
impl AR {
    pub(crate) fn ar(&mut self) -> &flash::AR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).ar }
    }
}

/// Opaque CR register
pub struct CR {
    _0: (),
}

#[allow(dead_code)]
impl CR {
    pub(crate) fn cr(&mut self) -> &flash::CR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).cr }
    }
}

/// Opaque KEYR register
pub struct KEYR {
    _0: (),
}

#[allow(dead_code)]
impl KEYR {
    pub(crate) fn keyr(&mut self) -> &flash::KEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).keyr }
    }
}

/// Opaque OBR register
pub struct OBR {
    _0: (),
}

#[allow(dead_code)]
impl OBR {
    pub(crate) fn obr(&mut self) -> &flash::OBR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).obr }
    }
}

/// Opaque OPTKEYR register
pub struct OPTKEYR {
    _0: (),
}

#[allow(dead_code)]
impl OPTKEYR {
    pub(crate) fn optkeyr(&mut self) -> &flash::OPTKEYR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).optkeyr }
    }
}

/// Opaque SR register
pub struct SR {
    _0: (),
}

#[allow(dead_code)]
impl SR {
    pub(crate) fn sr(&mut self) -> &flash::SR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).sr }
    }
}

/// Opaque WRPR register
pub struct WRPR {
    _0: (),
}

#[allow(dead_code)]
impl WRPR {
    pub(crate) fn wrpr(&mut self) -> &flash::WRPR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).wrpr }
    }
}
