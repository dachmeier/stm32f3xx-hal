//! # Flash memory
//!
//! Abstractions of the internal flash module.

// TODO: verifiy writes/erases or not?
// TODO: what if misaligned, pad before it and write anyways?

use core::{mem::size_of, slice};

use crate::pac::{flash, FLASH};

pub trait Flash {
    type WriteUnit;
    type Error;

    // could be changed to page_erase(&mut self, start_offset: usize, bytes_to_write: usize)
    // with bytes_to_write being the number of bytes which should be possible to write starting from start_offset
    // in this case, this function should erase all pages necessary for that write
    /// erases the page at the given offset
    unsafe fn page_erase(&mut self, offset: usize) -> Result<(), Self::Error>;

    /// Returns the data at the given offset as type T
    /// Checks if reading data with size of T from offset is valid
    unsafe fn read<T: Copy>(&mut self, offset: usize) -> Result<T, Self::Error>;

    /// Writes into flash memory.
    /// If necessary on the platform, this also performs the unlock operations before writing to flash
    /// Checks if writing of type T to offset is valid
    unsafe fn write<T: Copy>(&mut self, offset: usize, data: T) -> Result<(), Self::Error>;
}


pub const FLASH_START: usize = 0x0800_0000;
// pub const FLASH_END_MAX: usize = 0x0807_FFFF;

pub const SZ_1K: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum Error {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum SectorSize {
    Sz1K = 1,
    Sz2K = 2,
    Sz4K = 4,
}
impl SectorSize {
    pub const fn number_of_bytes(self) -> usize {
        SZ_1K * self as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
pub enum FlashSize {
    Sz16K = 16,
    Sz32K = 32,
    Sz64K = 64,
    Sz128K = 128,
    Sz256K = 256,
    Sz384K = 384,
    Sz512K = 512,
    Sz768K = 768,
    Sz1M = 1024,
}
impl FlashSize {
    pub const fn number_of_bytes(self) -> usize {
        SZ_1K as usize * self as usize
    }
}

pub struct FlashWriter<'a> {
    flash: &'a mut Parts,
    pub sector_sz: SectorSize,
    pub flash_sz: FlashSize,
}

impl<'a> FlashWriter<'a> {
    const KEY1: u32 = 0x45670123;
    const KEY2: u32 = 0xCDEF89AB;

    const WRITESIZE: usize = size_of::<<Self as Flash>::WriteUnit>();

    // TODO: think about wheter it is ok to only check if length of T exceeds flash, as more data might be written due to padding
    // -> consider alignment etc
    fn valid_write<T: Sized>(&self, offset: usize) -> Result<(), Error> {
        if offset + size_of::<T>() > self.flash_sz.number_of_bytes() {
            Err(Error::LengthTooLong)
        } else if offset % Self::WRITESIZE != 0 {
            Err(Error::AddressMisaligned)
        // } else if size_of::<T>() % Self::WRITESIZE != 0 {
        //     Err(Error::LengthNotMultipleSmallest)
        } else {
            Ok(())
        }
    }

    fn valid_read<T: Sized>(&self, offset: usize) -> Result<(), Error> {
        if offset + size_of::<T>() > self.flash_sz.number_of_bytes() {
            Err(Error::LengthTooLong)
        } else {
            Ok(())
        }
    }

    pub fn check_erasure(&self, offset: usize) -> bool {
        // rprintln!("check_erasure: self.address = {:x}", self.address);
        let start = FLASH_START + offset - (offset % self.sector_sz.number_of_bytes());
        // rprintln!("check_erasure: start address: {:x}, end address: {:x}", start, start + PAGE_SIZE);
        for idx in (start..(start + self.sector_sz.number_of_bytes())).step_by(size_of::<usize>()) {
            // rprintln!("checking iteration: {:x}", idx);
            let address = idx as *const usize;
            if unsafe { core::ptr::read_volatile(address) } != 0xFFFFFFFF {
                return false;
            }
        }

        true
    }

    fn lock(&mut self) -> Result<(), Error> {
        while self.flash.sr.sr().read().bsy().is_active() {}

        // TODO: why use modify instead of write here? (copied from stm32f1xx hal)
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
}

impl<'a> Flash for FlashWriter<'a> {
    type WriteUnit = u16;
    type Error = Error;


    // TODO: check what kind of addresses work. Only start address of each page? not stated in RM
    unsafe fn page_erase(&mut self, offset: usize) -> Result<(), Self::Error> {
        if offset >= self.flash_sz.number_of_bytes() {
            return Err(Error::OffsetOutOfBounds);
        }

        self.unlock()?;

        self.flash.cr.cr().modify(|_, w| w.per().set_bit());

        self.flash
            .ar
            .ar()
            .write(|w| w.far().bits((FLASH_START + offset) as u32));

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


    // fn read(&self, offset: usize, length: usize) -> Result<&[u8]> {
    //     self.valid_read_length(offset, length)?;

    //     let address = (FLASH_START + offset) as *const _;

    //     Ok(
    //         // NOTE(unsafe) read with no side effects. The data returned will
    //         // remain valid for its lifetime because we take an immutable
    //         // reference to this FlashWriter, and any operation that would
    //         // invalidate the data returned would first require taking a mutable
    //         // reference to this FlashWriter.
    //         unsafe { core::slice::from_raw_parts(address, length) },
    //     )
    // }

    // TODO: is requiring copy the right thing to do? If not, returning *address doesn't work
    unsafe fn read<T:Copy>(&mut self, offset: usize) -> Result<T, Self::Error> {
        self.valid_read::<T>(offset)?;

        let address = (FLASH_START + offset) as *const T;

        Ok(unsafe { *address })
    }

    // think about how to make verification better, i.e. to not use same code in uneven case again
    unsafe fn write<T>(&mut self, offset: usize, data: T) -> Result<(), Self::Error> {
        self.valid_write::<T>(offset)?;

        self.unlock()?;

        let ptr: *const T = &data;

        let write_blocks: usize = size_of::<T>() / Self::WRITESIZE;

        let u16_ptr = ptr as *const u16;

        let writesize_slice = core::slice::from_raw_parts(u16_ptr, write_blocks);

        for idx in 0..writesize_slice.len() {
            let write_address = (FLASH_START + offset + idx * 2) as *mut Self::WriteUnit;

            self.flash.cr.cr().write(|w| w.pg().set_bit());

            core::ptr::write_volatile(write_address, writesize_slice[idx]);

            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            if self.flash.sr.sr().read().eop().bit_is_set() {
                self.flash.sr.sr().write(|w| w.eop().clear_bit());
            } else {
                self.lock()?;
                return Err(Error::ProgrammingError);
            }

            if writesize_slice[idx] != core::ptr::read_volatile(write_address) {
                self.lock()?;
                return Err(Error::VerifyError);
            }
        }

        let uneven: bool = size_of::<T>() % Self::WRITESIZE != 0;

        if uneven {
            let byte_ptr = ptr as *const u8;
            let u8_slice = slice::from_raw_parts(byte_ptr, size_of::<T>());

            let to_write = *(u8_slice.last().unwrap()) as u16 | (0 as u16) << 8;

            // TODO: think about this again, is -1 correct?
            let address = (FLASH_START + offset + size_of::<T>() - 1) as *mut Self::WriteUnit;

            core::ptr::write_volatile(address, to_write);

            while self.flash.sr.sr().read().bsy().bit_is_set() {}

            if self.flash.sr.sr().read().eop().bit_is_set() {
                self.flash.sr.sr().write(|w| w.eop().clear_bit());
            } else {
                self.lock()?;
                return Err(Error::ProgrammingError);
            }

            if to_write != core::ptr::read_volatile(address) {
                self.lock()?;
                return Err(Error::VerifyError);
            }
        }

        self.lock()?;

        Ok(())
    }

    // fn write(&mut self, offset: usize, data: &[u8]) -> Result<()> {
    //     self.valid_write_length(offset, data.len())?;

    //     self.unlock()?;

    //     for idx in (0..data.len()).step_by(2) {
    //         let write_address =
    //             (FLASH_START + offset + idx) as *mut Self::WriteUnit;

    //         self.flash.cr.cr().write(|w| w.pg().set_bit());

    //         let to_write: Self::WriteUnit = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

    //         unsafe {
    //             core::ptr::write_volatile(write_address, to_write);
    //         };

    //         while self.flash.sr.sr().read().bsy().bit_is_set() {}

    //         if self.flash.sr.sr().read().eop().bit_is_set() {
    //             self.flash.sr.sr().write(|w| w.eop().clear_bit());
    //         } else {
    //             self.lock()?;
    //             return Err(Error::ProgrammingError);
    //         }

    //         if to_write != unsafe { core::ptr::read_volatile(write_address) } {
    //             self.lock()?;
    //             return Err(Error::VerifyError);
    //         }
    //     }

    //     self.lock()?;
    //     Ok(())
    // }
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
    pub fn writer(&mut self, sector_sz: SectorSize, flash_sz: FlashSize) -> FlashWriter {
        FlashWriter {
            flash: self,
            sector_sz,
            flash_sz,
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
