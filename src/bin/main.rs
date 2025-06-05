#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::i2c::RefCellDevice;

use esp_hal::{
    ram, 
    handler, 
    clock::CpuClock, 
    time::Rate, 
    timer::timg::TimerGroup,
    gpio::{interconnect::OutputSignal, Event, Input, InputConfig, Io, Level, Output, OutputConfig}, 
    ledc::{
        channel::{self, ChannelIFace}, 
        timer::TimerIFace, 
        Ledc
    }, 
    spi::{master::{Config, Spi}, Mode}, 
    i2c::master::{BusTimeout, Config as I2cConfig, I2c},
};
use esp_hal::gpio::interconnect::PeripheralOutput;
use mpu6886::Mpu6886;
use pcf8563::Pcf8563;
use watchy_m5::{
    music::{self, Song},
    pink_panther,
};
use esp_hal::{ledc::timer, main};
use esp_hal::{
    ledc::HighSpeed,
};

use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::{ColorInversion, Orientation};
use mipidsi::Builder;
use {esp_backtrace as _, esp_println as _};

use core::cell::RefCell;
use critical_section::Mutex;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, Primitive, PrimitiveStyle, Triangle},
    text::Text,
};

extern crate alloc;

/// Constants used on this stick
const AXP192_ADDR: u8 = 0x34;
const DISPLAY_WIDTH: u16 = 135;
const DISPLAY_HEIGHT: u16 = 240;


static BUTTON_A: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static BUTTON_B: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static BUTTON_C: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

#[embassy_executor::task]
async fn run() {
    loop {
        info!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task]
async fn sing(ledc: Ledc<'static>, mut buzzer: esp_hal::peripherals::GPIO2<'static>) {
    let song = Song::new(pink_panther::TEMPO);

    for (note, duration_type) in pink_panther::MELODY {
        let note_duration = song.calc_note_duration(duration_type) as u64;
        let pause_duration = note_duration / 10; // 10% of note_duration
        if note == music::REST {
            Timer::after(Duration::from_millis(note_duration)).await;
            continue;
        }
        let freq = Rate::from_hz(note as u32);

        let mut hstimer0 = ledc.timer::<HighSpeed>(timer::Number::Timer0);
        hstimer0
            .configure(timer::config::Config {
                duty: timer::config::Duty::Duty10Bit,
                clock_source: timer::HSClockSource::APBClk,
                frequency: freq,
            })
            .unwrap();

        let mut channel0 = ledc.channel(channel::Number::Channel0, buzzer.reborrow());
        channel0
            .configure(channel::config::Config {
                timer: &hstimer0,
                duty_pct: 50,
                pin_config: channel::config::PinConfig::PushPull,
            })
            .unwrap();

        Timer::after(Duration::from_millis(note_duration - pause_duration)).await;

        channel0.set_duty(0).unwrap();
        Timer::after(Duration::from_millis(pause_duration)).await;
    }
}


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    // let timer1 = TimerGroup::new(peripherals.TIMG0);
    // let _init = esp_wifi::init(
    //     timer1.timer0,
    //     esp_hal::rng::Rng::new(peripherals.RNG),
    //     peripherals.RADIO_CLK,
    // )
    // .unwrap();

    let mut io = Io::new(peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(handler);

    // Set GPIO19 as an output, and set its state low initially.
    let mut led = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());

    // Get the peripherals for the buzzer
    let buzzer = peripherals.GPIO2;
    let ledc = Ledc::new(peripherals.LEDC);

    
    // set up bus for i2c stuff
    let i2c = I2c::new(
        peripherals.I2C1, 
        I2cConfig::default()
            .with_frequency(Rate::from_khz(400))
            .with_timeout(BusTimeout::Maximum),
        )
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22)
        .into_async();
    let i2c_bus = RefCell::new(i2c);
    let mut rtc = Pcf8563::new(RefCellDevice::new(&i2c_bus));
    let mut imu = Mpu6886::new(RefCellDevice::new(&i2c_bus));

    if let Err(_e) = imu.init() {
        error!("unable to init imu");
    }
    
    match rtc.datetime() {
        Ok(dt) => info!("datetime {}", dt.second),
        Err(e) => {
            match e {
                pcf8563::Error::I2cError(error_kind) => error!("i2c error: {}", error_kind),
                pcf8563::Error::Other => error!("other error"),
                _ => error!("mystery error"),
            }
        },
    }

    // Set GPIO37 as an input
    let mut button_a = Input::new(peripherals.GPIO37, InputConfig::default());
    // Set GPIO39 as an input
    let mut button_b = Input::new(peripherals.GPIO39, InputConfig::default());
    // Set GPIO35 as an input
    let mut button_c = Input::new(peripherals.GPIO35, InputConfig::default());

    // ANCHOR: critical_section
    critical_section::with(|cs| {
        button_a.listen(Event::FallingEdge);
        BUTTON_A.borrow_ref_mut(cs).replace(button_a);
        button_b.listen(Event::FallingEdge);
        BUTTON_B.borrow_ref_mut(cs).replace(button_b);
        button_c.listen(Event::FallingEdge);
        BUTTON_C.borrow_ref_mut(cs).replace(button_c);
    });
    // ANCHOR_END: critical_section

    // lets try to get the interface for the display
    let display_dc = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let display_rst = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());
    let backlight = peripherals.GPIO27;
    let mut display_bl = Output::new(backlight, Level::Low, OutputConfig::default());

    
    let display_spi: Spi<'_, esp_hal::Async> = Spi::new(
        peripherals.SPI2, 
        Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0)
    ).unwrap()
    .with_sck(peripherals.GPIO13)
    // .with_cs(peripherals.GPIO5)
    .with_mosi(peripherals.GPIO15)
    .into_async();

    let display_cs: Output<'_> = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());
    let disp_device = ExclusiveDevice::new_no_delay(display_spi, display_cs).unwrap();

    // let mut display_buf = [0_u8; DISPLAY_WIDTH as usize * DISPLAY_HEIGHT as usize];
    let mut display_buf = [0_u8; 512];

    let mut display_delay = esp_hal::delay::Delay::new();
    let di = SpiInterface::new(disp_device, display_dc, &mut display_buf);
    let rotation = Orientation::new().rotate(mipidsi::options::Rotation::Deg0);
    let mut display = Builder::new(ST7789, di)
        .reset_pin(display_rst)
        .invert_colors(ColorInversion::Inverted)
        .display_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .display_offset(52, 40)
        .orientation(rotation)
        .init(&mut display_delay)
        .unwrap();

    // Text
    // let char_w = FONT_10X20.character_size.width; //10;
    // let char_h = FONT_10X20.character_size.height; //20;
    let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    let text = "Hello World ^_^;";
    let mut text_x: i32 = DISPLAY_WIDTH.into();
    let mut text_y: i32 = (DISPLAY_HEIGHT / 2).into();

    // Alternating color
    let colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE];

    // let style = PrimitiveStyleBuilder::new()
    //     .stroke_color(Rgb565::RED)
    //     .stroke_width(3)
    //     .fill_color(Rgb565::GREEN)
    //     .build();
    // let area = Rectangle::new(Point { x: 0, y: 0 }, Size::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into()))
    //     .into_styled(style);
    

    // Clear the display initially
    display.clear(colors[0]).unwrap();

    display_bl.set_high();
    
    // TODO: Spawn some tasks
    // let _ = spawner;
    spawner.spawn(run()).ok();
    spawner.spawn(sing(ledc, buzzer)).ok();

    let mut counter = 0;
    loop {
        counter += 1;
        // Fill the display with alternating colors every 8 frames
        display.clear(colors[(counter / 8) % colors.len()]).unwrap();
        
        // Draw text
        let right = Text::new(text, Point::new(text_x, text_y), text_style)
            .draw(&mut display)
            .unwrap();
        text_x = if right.x <= 0 { DISPLAY_WIDTH.into() } else { text_x - 10 };
        led.toggle();

        // info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#[handler]
#[ram]
fn handler() {
    critical_section::with(|cs| {
        info!("GPIO interrupt");
        if let Some(button_a) = BUTTON_A.borrow_ref_mut(cs).as_mut() {
            if button_a.is_interrupt_set() {
                button_a.clear_interrupt();
                info!("Button A");
            }
        }
        if let Some(button_b) = BUTTON_B.borrow_ref_mut(cs).as_mut() {
            if button_b.is_interrupt_set() {
                button_b.clear_interrupt();
                info!("Button B");
            }
        }
        if let Some(button_c) = BUTTON_C.borrow_ref_mut(cs).as_mut() {
            if button_c.is_interrupt_set() {
                button_c.clear_interrupt();
                info!("Button C");
            }
        }
    });
}