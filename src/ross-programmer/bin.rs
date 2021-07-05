#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use panic_itm as _;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use cortex_m::asm::nop;
use cortex_m::iprint;
use cortex_m_rt::entry;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pac::{Peripherals, CorePeripherals, ITM};

const DEBUG: bool = true;
const HEAP_SIZE: usize = 4096;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

static mut ITM_PERIPHERAL: Option<ITM> = None;

macro_rules! debug {
    ($fmt:expr) => {
        if DEBUG {
            iprint!(&mut unsafe { ITM_PERIPHERAL.as_mut().unwrap() }.stim[0], concat!($fmt, "\r\n"));
        }
    };
    ($fmt:expr, $($arg:tt)*) => {
        if DEBUG {
            iprint!(&mut unsafe { ITM_PERIPHERAL.as_mut().unwrap() }.stim[0], concat!($fmt, "\r\n"), $($arg)*);
        }
    };
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    rcc.cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    unsafe {
        ITM_PERIPHERAL = Some(cp.ITM);
    }

    debug!("Firmware initialized.");

    allocate_heap();

    loop {
        nop();
    }
}

fn allocate_heap() {
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, HEAP_SIZE) }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
