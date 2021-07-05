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
use stm32f1xx_hal::can::Can;
use stm32f1xx_hal::i2c::{BlockingI2c, Mode};
use stm32f1xx_hal::time::MonoTimer;
use eeprom24x::{Eeprom24x, SlaveAddr};
use nb::block;
use bxcan::*;
use bxcan::filter::Mask32;

use ross_eeprom::{RossEeprom, RossDeviceInfo};
use ross_protocol::ross_frame::RossFrame;
use ross_protocol::ross_packet::RossPacketBuilder;
use ross_protocol::ross_convert_packet::RossConvertPacket;
use ross_protocol::ross_event::ross_bootloader_event::RossBootloaderHelloEvent;
use ross_protocol::ross_event::ross_programmer_event::RossProgrammerHelloEvent;

const EEPROM_BITRATE: u32 = 400_000;

const CAN_BITRATE: u32 = 50_000;
const CAN_TSEG1: u32 = 13;
const CAN_TSEG2: u32 = 2;
const CAN_SJW: u32 = 1;

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
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
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

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let timer = MonoTimer::new(cp.DWT, cp.DCB, clocks);

    let mut eeprom = {
        let i2c1 = {
            let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
            let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
            // TODO: put better values in thge last 4 arguments
            BlockingI2c::i2c1(dp.I2C1, (scl, sda), &mut afio.mapr, Mode::standard(EEPROM_BITRATE.hz()), clocks, &mut rcc.apb1, 10, 10, 10, 10)
        };

        let eeprom = Eeprom24x::new_24x02(i2c1, SlaveAddr::Alternative(false, false, false));

        RossEeprom::new(eeprom, 32)
    };

    let device_info = eeprom.read_device_info().unwrap();

    let mut can1 = {
        let can = Can::new(dp.CAN1, &mut rcc.apb1, dp.USB);

        let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
        let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
        can.assign_pins((tx, rx), &mut afio.mapr);

        bxcan::Can::new(can)
    };

    can1.configure(|c| {
        c.set_bit_timing(calc_can_btr(clocks.pclk1().0));
        c.set_loopback(false);
        c.set_silent(false);
    });

    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Mask32::accept_all());
    drop(filters);

    block!(can1.enable()).unwrap();

    allocate_heap();

    let (mut tx, mut rx) = can1.split();

    let bootloader_hello_event = loop {
        let programmer_hello_event = transmit_programmer_hello_event(&mut tx, &device_info);

        debug!("Sent 'programmer_hello_event' ({:?}).", programmer_hello_event);

        let bootloader_hello_event_option = wait_for_bootloader_hello_event_with_timeout(&mut rx, 100, &timer);

        if let Some(bootloader_hello_event) = bootloader_hello_event_option {
            break bootloader_hello_event;
        }
    };

    debug!("Received 'bootloader_hello_event' ({:?}).", bootloader_hello_event);

    loop {
        nop();
    }
}

const fn calc_can_btr(clock_rate: u32) -> u32 {
    let brp = clock_rate / CAN_BITRATE / (CAN_TSEG1 + CAN_TSEG2);

    (brp - 1) | ((CAN_TSEG1 - 1) << 16) | ((CAN_TSEG2 - 1) << 20) | ((CAN_SJW - 1) << 24)
}

fn allocate_heap() {
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, HEAP_SIZE) }
}

fn transmit_programmer_hello_event<T: Instance>(
    tx: &mut Tx<T>,
    device_info: &RossDeviceInfo,
) -> RossProgrammerHelloEvent {
    let programmer_hello_event = RossProgrammerHelloEvent {
        programmer_address: device_info.device_address,
        firmware_version: device_info.firmware_version,
    };

    let frames = programmer_hello_event.to_packet().to_frames();

    for frame in frames {
        block!(tx.transmit(&frame.to_bxcan_frame())).unwrap();
    }

    return programmer_hello_event;
}

fn wait_for_bootloader_hello_event_with_timeout<T: Instance>(rx: &mut Rx<T>, timeout_ms: u32, timer: &MonoTimer) -> Option<RossBootloaderHelloEvent> {
    let start_time = timer.now().elapsed();

    let mut packet_builder_option: Option<RossPacketBuilder> = None;

    loop {
        if let Ok(frame) = rx.receive() {
            if let Some(mut packet_builder) = packet_builder_option {
                if packet_builder.frames_left() > 0 {
                    packet_builder
                        .add_frame(RossFrame::from_bxcan_frame(frame).unwrap())
                        .unwrap();

                    packet_builder_option = Some(packet_builder);
                } else {
                    let packet = packet_builder.build().unwrap();

                    if let Ok(bootloader_hello_event) =
                    RossBootloaderHelloEvent::try_from_packet(packet)
                    {
                        return Some(bootloader_hello_event);
                    } else {
                        debug!("Unexpected event.");
                    }

                    if let Ok(packet_builder) =
                        RossPacketBuilder::new(RossFrame::from_bxcan_frame(frame).unwrap())
                    {
                        packet_builder_option = Some(packet_builder)
                    } else {
                        packet_builder_option = None;
                        debug!("Caught a middle frame.");
                    }
                }
            } else {
                if let Ok(packet_builder) =
                    RossPacketBuilder::new(RossFrame::from_bxcan_frame(frame).unwrap())
                {
                    packet_builder_option = Some(packet_builder)
                } else {
                    packet_builder_option = None;
                    debug!("Caught a middle frame.");
                }
            }
        } else if (timer.now().elapsed() - start_time) / timer.frequency().0 * 1000 >= timeout_ms {
            return None;
        }
    }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
