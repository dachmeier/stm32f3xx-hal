use core::{fmt::Debug, mem::size_of};

use crate::pac::{flash, FLASH};

pub const FLASH_START: u32 = 0x0800_0000;
pub const FLASH_END_MAX: u32 = 0x0807_FFFF;

pub const SZ_1K: u32 = 1024;


// pub trait FlashWriteErase {
//     type WriteEraseError: Debug;

//     unsafe fn page_erase(&mut self, start_offset: usize) -> Result<(), Self::WriteEraseError>;

//     /// Writes bytes in data slice to flash, starting from offset.
//     /// The implementation has to check that data.len is an integer multiple of the smallest possible write.
//     /// If not it should return LengthNotMultipleSmallest Error
//     unsafe fn write(&mut self, offset: usize, buf: &[u8]) -> Result<(), Self::WriteEraseError>;
// }

pub trait FlashWriteErase {
    /// The smallest chunk writable in amount of bytes
    const WRITESIZE: u32;
    /// The smallest chunk erasable in amount of bytes (Also called erase sector size)
    const ERASESIZE: u32;
    /// The type of error that can occur when writing or erasing
    type WriteEraseError: Debug;

    /// Erases all erase sectors "touched" by the range of the given length starting at the given offset
    ///
    /// Returns 'Ok(())' if the operation was successful and an error if the operation is invalid based on the given arguments or didn't succeed
    ///
    /// # Safety
    ///
    /// When calling this method it must be made sure to not erase part of the running application
    unsafe fn erase(&mut self, offset: u32, length: u32) -> Result<(), Self::WriteEraseError>;

    /// Writes bytes in buffer slice to flash, starting from offset.
    /// If the buffer length is no integer multiple of ['WRITESIZE'](FlashWriteErase::WRITESIZE), the data is padded by the appropriate amount
    ///
    /// # Safety
    ///
    /// When calling this method it must be made sure to not reprogram part of the running application
    unsafe fn write(&mut self, offset: u32, buffer: &[u8]) -> Result<(), Self::WriteEraseError>;
}

// pub trait FlashRead {
//     type ReadError: Debug;
//     unsafe fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), Self::ReadError>;
// }

pub trait FlashRead {
    /// The smallest chunk readable in amount of bytes
    const READSIZE: u32;
    /// The type of error that can occur when reading
    type ReadError: Debug;

    /// Reads data from offset into the provided buffer
    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::ReadError>;
}


pub struct FlashLayout {
    pub flash_size: u32,
    pub sector_size: u32,
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

    fn valid_write_length(&self, offset: u32, length: u32) -> Result<(), Error> {
        if offset + length > FLASH_LAYOUT.flash_size {
            Err(Error::OutOfBounds)
        } else if offset % Self::WRITESIZE != 0 {
            Err(Error::AddressMisaligned)
        } else {
            Ok(())
        }
    }

    fn valid_read_length(&self, offset: u32, length: u32) -> Result<(), Error> {
        if offset + length > FLASH_LAYOUT.flash_size {
            Err(Error::OutOfBounds)
        } else {
            Ok(())
        }
    }

    fn lock(&mut self) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.cr.cr().modify(|_, w| w.lock().lock());

        match self.flash.cr.cr().read().lock().is_locked() {
            true => Ok(()),
            false => Err(Error::LockError),
        }
    }

    fn unlock(&mut self) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        if !self.is_locked() {
            return Ok(());
        }

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

    unsafe fn program_half_word(&mut self, halfword: u16, address: *mut u16) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.cr.cr().modify(|_, w| w.pg().program());

        address.write_volatile(halfword);

        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.cr.cr().modify(|_, w| w.pg().clear_bit());

        if self.flash.sr.sr().read().eop().bit_is_set() {
            self.flash.sr.sr().modify(|_, w| w.eop().clear_bit());
        } else {
            return Err(Error::WriteError);
        }


        Ok(())
    }

    unsafe fn sector_erase(&mut self, start_offset: u32) -> Result<(), Error> {
        if start_offset >= FLASH_LAYOUT.flash_size {
            return Err(Error::OutOfBounds);
        }

        while self.flash.sr.sr().read().bsy().is_active() {}

        self.flash.cr.cr().modify(|_, w| w.per().page_erase());

        self.flash
            .ar
            .ar()
            .write(|w| w.far().bits((FLASH_START + start_offset) as u32));

        self.flash.cr.cr().modify(|_, w| w.strt().start());

        // the stm32f3xx reference manual states having to wait at least one cpu cycle before checking bsy
        cortex_m::asm::delay(1);

        while self.flash.sr.sr().read().bsy().is_active() {}
        self.flash.cr.cr().modify(|_, w| w.per().clear_bit());

        if self.flash.sr.sr().read().eop().is_event() {
            self.flash.sr.sr().modify(|_, w| w.eop().reset());
            Ok(())
        } else {
            Err(Error::EraseError)
        }
    }
}

impl<'a> FlashWriteErase for FlashWriter<'a> {
    type WriteEraseError = Error;
    const WRITESIZE: u32 = 2;
    const ERASESIZE: u32 = FLASH_LAYOUT.sector_size;

    unsafe fn erase(&mut self, offset: u32, length: u32) -> Result<(), Error> {
        let page_offset = offset % Self::ERASESIZE;

        // basically this is supposed to be (page_offset + length) / PAGE_SIZE
        // where the division / is an integer ceiling division (e.g. 3/2 = 2, 1/2 = 1 ...)
        // because there is no obvious other way to implement that, floor division is used and 1 is added in the end
        // if exactly one full sector should be written, number of pages has to be 1 -> subtract 1 from page_offset + size_of::<T>()
        // this works only for length > 0 which is ok to assume as writing something of length 0 is not

        let number_of_pages = (page_offset + length - 1) / Self::ERASESIZE + 1;

        self.unlock()?;

        for i in 0..number_of_pages {
            self.sector_erase(offset + i * Self::ERASESIZE)?;
        }

        self.lock()?;

        Ok(())
    }

    unsafe fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        self.valid_write_length(offset, data.len() as u32)?;

        self.unlock()?;
        let base_address = FLASH_START + offset;

        for idx in (0..(data.len() - 1)).step_by(2) {
            let write_address = (base_address + idx as u32) as *mut u16;

            let to_write: u16 = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

            self.program_half_word(to_write, write_address)?;
        }

        let uneven: bool = data.len() as u32 % Self::WRITESIZE != 0;

        if uneven {
            let to_write = *(data.last().unwrap()) as u16 | (0xFF) << 8;

            // TODO: think about this again, is -1 correct?
            let address = (base_address + data.len() as u32 - 1) as *mut u16;

            self.program_half_word(to_write, address)?;
        }

        self.lock()?;
        Ok(())
    }
}

impl<'a> FlashRead for FlashWriter<'a> {
    type ReadError = Error;
    const READSIZE: u32 = 1;

    fn read(&self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        self.valid_read_length(offset, buf.len() as u32)?;

        let address = (FLASH_START + offset) as *const _;

        unsafe {
            buf.copy_from_slice(core::slice::from_raw_parts(address, buf.len()));
        }
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
