#![no_std]
#![no_main]
#![feature(lazy_cell)]
#![feature(alloc_error_handler)]

use core::alloc::Layout;
use core::borrow::BorrowMut;
use core::convert::TryFrom;
use core::fmt::Write;
use core::num::NonZeroU32;
use core::panic::PanicInfo;
use core::str::FromStr;

extern crate alloc;
use alloc::{boxed::Box, string::ToString};

use cortex_m_rt::entry;

use cortex_m::peripheral::syst::SystClkSource;

use dsf_core::options::Filters;
use embedded_graphics::primitives::Rectangle;
use rand_core::RngCore;

#[cfg(not(feature = "defmt"))]
pub use log::{debug, error, info, trace, warn, Level};

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

extern crate alloc_cortex_m;
use alloc_cortex_m::CortexMHeap;

// Import global logger
use defmt_rtt as _;

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::MODE_0;

//use embedded_hal_compat::ReverseCompat;

use hal::gpio::Speed;
use hal::prelude::*;
use hal::serial;
use hal::{i2c::I2c, serial::Serial, spi::Spi};
use stm32f4xx_hal as hal;

use rand_chacha::ChaChaRng;
use rand_core::SeedableRng;

use radio_sx128x::prelude::*;
use radio_sx128x::Config as Sx128xConfig;

use lpwan::mac_802154::{AssocState, SyncState};
use lpwan::prelude::*;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X12, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_text::{
    alignment::{HorizontalAlignment, VerticalAlignment},
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
use ssd1306::{mode::TerminalMode, prelude::*, I2CDisplayInterface, Ssd1306};

use dsf_core::{
    base::{Decode, Encode},
    prelude::*,
};

use dsf_engine::store::Store;
use dsf_iot::prelude::*;

mod newlib;

mod serial_log;
use serial_log::SerialLogger;

mod timer;
use timer::SystickDelay;

mod common;
use common::{device_keys, HeaplessStore, RadioComms, StaticKeyStore};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const ALLOC_SIZE: usize = 20 * 1024;

// Bind custom OS RNG
getrandom::register_custom_getrandom!(os_rand);

static mut RNG: Option<ChaChaRng> = None;

// OS RNG function
pub fn os_rand(buf: &mut [u8]) -> Result<(), getrandom::Error> {
    unsafe {
        let rng = match RNG.borrow_mut() {
            Some(rng) => rng,
            None => {
                let code = NonZeroU32::new(1).unwrap();
                return Err(getrandom::Error::from(code));
            }
        };

        rng.fill_bytes(buf);
    }

    Ok(())
}

const LOG_FILTERS: &'static [&'static str] = &["radio_sx128x"];

const DISPLAY_UPDATE_MS: u64 = 1000;
const JOIN_TIMEOUT: u64 = 10000;

#[entry]
fn main() -> ! {
    // Initialize the allocator
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, ALLOC_SIZE) }

    let core = cortex_m::peripheral::Peripherals::take().unwrap();
    let device = hal::pac::Peripherals::take().unwrap();

    let gpioa = device.GPIOA.split();
    let gpiob = device.GPIOB.split();
    let _gpioc = device.GPIOC.split();
    let gpiod = device.GPIOD.split();
    let gpioe = device.GPIOE.split();
    let gpiof = device.GPIOF.split();

    // Setup pins
    let mut led0 = gpiob.pb7.into_push_pull_output();
    let mut led1 = gpiob.pb14.into_push_pull_output();
    led1.set_high();

    // Setup system clock
    let rcc = device.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.MHz()).require_pll48clk().freeze();

    // Setup serial (and logging)
    let uart_cfg = serial::config::Config::default().baudrate(115_200.bps());
    let uart_tx = gpiod.pd8.into_alternate();
    let uart_rx = gpiod.pd9.into_alternate();
    let uart = Serial::new(device.USART3, (uart_tx, uart_rx), uart_cfg, &clocks).unwrap();

    SerialLogger::init(Level::Info, Box::new(uart), LOG_FILTERS);

    let mut syst = core.SYST;
    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(18_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    // Fetch unique chip ID for address use
    let chip_id = hal::signature::Uid::get();
    let chip_id =
        (chip_id.waf_num() as u64) << 32 | (chip_id.y() as u64) << 16 | (chip_id.x() as u64);

    info!("hello world");

    info!("Starting subscriber with ID: {}", chip_id);

    let device_keys = device_keys();
    let pub_id = Id::from_str("G3hsdihnnWH5DX5VsNvGrnyEGU3VCHjgcMijRp9RpqZH").unwrap();

    debug!("Configuring RNG");

    // Setup RNG
    let dev_rng = device.RNG.constrain(&clocks);
    let chacha_rng = ChaChaRng::from_rng(dev_rng).unwrap();
    unsafe { RNG = Some(chacha_rng) };

    let mut keystore = StaticKeyStore::new();

    debug!("RNG bound");

    // Setup Pins
    let (mut rf_cs, rf_rst, rf_busy, rf_ready, mut rf_ant) = (
        gpiof.pf13.into_push_pull_output(),
        gpioa.pa3.into_push_pull_output(),
        gpioe.pe13.into_floating_input(),
        gpiod.pd13.into_floating_input(),
        gpiof.pf3.into_push_pull_output(),
    );

    let _ = rf_cs.set_high();
    let _ = rf_ant.set_high();

    // Setup SPI
    let (spi_mosi, spi_miso, spi_sck) = {
        (
            gpioa.pa7.into_alternate().speed(Speed::VeryHigh),
            gpioa.pa6.into_alternate().speed(Speed::VeryHigh),
            gpioa.pa5.into_alternate().speed(Speed::VeryHigh),
        )
    };
    let spi = Spi::new(
        device.SPI1,
        (spi_sck, spi_miso, spi_mosi),
        MODE_0,
        1.MHz().into(),
        &clocks,
    );
    let rf_spi = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, rf_cs);

    // Setup I2C
    let (i2c_scl, i2c_sda) = {
        (
            gpiob
                .pb10
                .into_alternate()
                .speed(Speed::VeryHigh)
                .internal_pull_up(true)
                .set_open_drain(),
            gpiob
                .pb11
                .into_alternate()
                .speed(Speed::VeryHigh)
                .internal_pull_up(true)
                .set_open_drain(),
        )
    };

    let i2c = I2c::new(device.I2C2, (i2c_scl, i2c_sda), 100.kHz(), &clocks);

    // Setup display
    let disp_if = I2CDisplayInterface::new(i2c);
    let mut disp = Ssd1306::new(disp_if, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X12, BinaryColor::On);
    let textbox_style = TextBoxStyleBuilder::new()
        .height_mode(HeightMode::FitToText)
        .build();

    // Setup a timer object
    //let mut timer = Timer::tim1(device.TIM1, 1.khz(), clocks).unwrap();

    // Create delay object
    let delay = SystickDelay {};

    debug!("Initialising radio");

    // Initialise radio
    let rf_config = common::rf_config();
    let mut radio = match Sx128x::spi(rf_spi, rf_busy, rf_ready, rf_rst, delay, &rf_config) {
        Ok(v) => v,
        Err(e) => {
            panic!("Error initialising radio: {:?}", e);
        }
    };

    // Re-set syncword for GFSK mode
    if let Modem::Gfsk(_gfsk) = &rf_config.modem {
        radio
            .set_syncword(1, &[0x11, 0x22, 0x33, 0x44, 0x55])
            .unwrap();
    }

    debug!("Initialising networking");

    // Initialise network stack
    let address = ExtendedAddress(chip_id);
    let mac_config = mac_802154::Config {
        pan_coordinator: false,
        ..Default::default()
    };

    let timer = SystickDelay {};
    let mac = match mac_802154::Mac::new(address, mac_config, radio, timer.clone()) {
        Ok(m) => m,
        Err(e) => {
            panic!("Error initalising MAC: {:?}", e);
        }
    };

    debug!("Initialising 6lo");
    let sixlo_cfg = SixLoConfig {
        ..Default::default()
    };
    let mut sixlo = SixLo::<_, 127>::new(mac, MacAddress::Extended(PanId(1), address), sixlo_cfg);

    let info = IotInfo::<4>::new(&[]).unwrap();

    let comms = RadioComms::new();
    //let store = HeaplessStore::new();

    let keys = device_keys.iter().find(|(c, _k)| *c == chip_id);
    if let Some(k) = &&keys {
        //store.set_ident(&k.1);
    }

    #[cfg(nope)]
    {
        let mut engine = match IotEngine::new(info, &[], comms, store) {
            Ok(e) => e,
            Err(e) => {
                error!("Failed to initialise engine: {:?}", e);
                loop {}
            }
        };
    }

    {
        // TODO: set kind to IotService
        let mut sb = ServiceBuilder::default();

        let keys = device_keys.iter().find(|(c, _k)| *c == chip_id);
        if let Some(k) = keys {
            sb = sb.private_key(k.1.pri_key.clone().unwrap());
        }

        let mut s = sb.body(Body::None).build().unwrap();
        info!("Service ID: {}", s.id().to_string());

        let mut buff = [0u8; 1024];
        let (n, _p) = s.publish_primary(Default::default(), &mut buff).unwrap();

        let primary = &buff[..n];

        info!("Generated primary page of {} bytes", n);

        info!("Starting app");

        let mut rf_buff = [0u8; 512];
        let mut net_state = NetState::None;
        let mut sub_state = SubState::None;
        let mut req_id = rand_core::OsRng {}.next_u32() as u16;

        let mut iot_info = IotInfo::<4>::default();
        let mut iot_data = IotData::<4>::default();
        let mut n1 = 0;

        let mut last_display_update = 0;
        let mut last_state = 0;

        loop {
            let now = SystickDelay {}.ticks_ms();

            // Update the mac
            if let Err(e) = sixlo.tick(now) {
                error!("MAC error: {:?}", e);
            }

            // TODO: check for received packets
            if let Ok(Some((n, addr, _hdr))) = sixlo.receive(now, &mut rf_buff) {
                let payload = &mut rf_buff[..n];

                trace!("Received {} byte packet", n);
                trace!("Page: {:02x?}", &payload);

                let base = match Container::parse(payload, &keystore) {
                    Ok(v) => v,
                    Err(e) => {
                        error!("DSF parsing error: {:?}", e);
                        continue;
                    }
                };

                let id = base.id().clone();
                let header = base.header();

                // Handle messages
                if header.kind().is_message() {
                    match NetMessage::convert(base, &keystore) {
                        Ok(NetMessage::Request(req)) => {
                            // Handle discovery requests
                            if let NetRequestBody::Hello = &req.data {
                                // NOTE: removed for client-based discovery
                            } else if let NetRequestBody::PushData(id, _d) = &req.data {
                                info!("Received push-data for service: {}", id);
                            } else {
                                warn!("Unhandled DSF request");
                            }
                        }
                        Ok(NetMessage::Response(resp)) => {
                            match (&net_state, &sub_state, &resp.data) {
                                (NetState::Joining(gw), _, NetResponseBody::Status(s))
                                    if *s == dsf_core::net::Status::Ok =>
                                {
                                    info!("Received join response");
                                    net_state = NetState::Joined(*gw);
                                }
                                (NetState::Joined(gw), SubState::Subscribing(req_id), _) => {
                                    info!("Received subscribe response");
                                    // TODO: handle subscribe response
                                    sub_state = SubState::Subscribed;
                                }
                                _ => {
                                    warn!("Unhandled DSF response");
                                }
                            }
                        }
                        _ => error!("Message parsing failed"),
                    }

                // Handle page objects
                } else if header.kind().is_page() {
                    let id = base.id();
                    (iot_info, _) = IotInfo::<4>::decode(base.body_raw()).unwrap();

                    // Cache service keys
                    if let Some(pub_key) = base.public_options_iter().pub_key() {
                        keystore.insert(id.clone(), Keys::new(pub_key.clone()));
                    }

                    info!("Subscribed to service ID: {}", id);

                    info!("Endpoints: ");
                    for i in 0..iot_info.descriptors.len() {
                        let e = &iot_info.descriptors[i];

                        info!("  - {:2}: {:?}", i, e.kind);
                    }

                    sub_state = SubState::Subscribed;

                // Handle data objects
                } else if header.kind().is_data() {
                    (iot_data, _) = IotData::decode(base.body_raw()).unwrap();

                    info!("Received data");
                    for i in 0..iot_data.data.len() {
                        let ep_data = &iot_data.data[i];

                        info!("  - {:2}: {:.02}", i, ep_data.value);
                    }
                }
            } else {
                if let Ok(MacState::Synced(gw)) = sixlo.mac().state() {
                    match (&net_state, &sub_state) {
                        (NetState::None, _) => {
                            info!("Issuing register to gw: {:?}", gw);

                            // Transmit encoded primary page
                            if let Err(e) = sixlo.transmit(now, gw, primary) {
                                error!("MAC transmit error: {:?}", e);
                            }
                            // Update net state to joining
                            net_state = NetState::Joining(gw);
                            last_state = now;
                        }
                        (NetState::Joining(_), _) if now > (last_state + JOIN_TIMEOUT) => {
                            info!("Register timeout");
                            net_state = NetState::None;
                            last_state = now;
                        }
                        (NetState::Joined(_), SubState::None) => {
                            // Generate subscribe request
                            let sub_req = NetRequestBody::Subscribe(pub_id.clone());
                            let mut req =
                                NetRequest::new(s.id(), req_id, sub_req, Flags::PUB_KEY_REQUEST);
                            req.set_public_key(s.public_key());

                            let c = s
                                .encode_request(&req, &Keys::default(), &mut rf_buff)
                                .unwrap();

                            info!("Issuing subscribe to gw: {:?}", gw);

                            if let Err(e) = sixlo.transmit(now, gw, c.raw()) {
                                error!("MAC transmit error: {:?}", e);
                            }

                            // Update state to subscribing
                            sub_state = SubState::Subscribing(req_id);
                            req_id = req_id.wrapping_add(1);
                        }
                        _ => (),
                    }
                } else {
                    net_state = NetState::None;
                    sub_state = SubState::None;
                }
            }

            if (now / 500) % 2 == 0 {
                led0.set_high();
            } else {
                led0.set_low();
            }

            if now > (last_display_update + DISPLAY_UPDATE_MS) {
                let mut buff = alloc::string::String::new();

                let dat = &iot_data.data[..];
                let eps = &iot_info.descriptors[..];

                if dat.len() > 0 && eps.len() > 0 {
                    for i in 0..eps.len() {
                        let _ = writeln!(buff, "{:02} {}", dat[i].value, eps[i].kind.unit());
                    }
                } else {
                    match (&net_state, &sub_state) {
                        (NetState::None, _) => writeln!(buff, "Searching").unwrap(),
                        (NetState::Joining(_), _) => writeln!(buff, "Connecting").unwrap(),
                        (_, SubState::None) => writeln!(buff, "Pending").unwrap(),
                        (_, SubState::Subscribing(_)) => writeln!(buff, "Subscribing").unwrap(),
                        (_, SubState::Subscribed) => writeln!(buff, "Subscribed").unwrap(),
                    }
                }

                let bounds = Rectangle::new(Point::zero(), Size::new(128, 0));
                let text_box =
                    TextBox::with_textbox_style(buff.as_str(), bounds, text_style, textbox_style);

                disp.clear(BinaryColor::Off).unwrap();
                text_box.draw(&mut disp).unwrap();
                disp.flush().unwrap();

                last_display_update = now;
            }

            SystickDelay {}.delay_ms(1u32);
        }
    }
}

#[derive(Clone, PartialEq, Debug)]
pub enum NetState {
    None,
    Joining(MacAddress),
    Joined(MacAddress),
}

#[derive(Clone, PartialEq, Debug)]
pub enum SubState {
    None,
    Subscribing(u16),
    Subscribed,
}

use num_traits::float::FloatCore;

fn _roundf(v: f32) -> f32 {
    f32::round(v * 100.0) / 100.0
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    error!("{}", alloc::format!("{}", info));

    loop {}
}
