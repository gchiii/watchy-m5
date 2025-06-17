#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::i2c::RefCellDevice;

use esp_hal::{
    clock::CpuClock, 
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig}, 
    handler, 
    i2c::master::{BusTimeout, Config as I2cConfig, I2c}, 
    ledc::Ledc, 
    ram, 
    spi::{master::{Config, Spi}, Mode}, time::Rate, timer::{timg::TimerGroup, AnyTimer}
};
    
use mpu6886::Mpu6886;
use pcf8563::Pcf8563;

use static_cell::StaticCell;
use watchy_m5::music::{Buzzer, BuzzerCommand, BuzzerState};

use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex}, 
    channel::Channel, 
};

use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::{ColorInversion, Orientation};
use mipidsi::Builder;
use {esp_backtrace as _, esp_println as _};

use core::cell::RefCell;
// use critical_section::Mutex as csMutex;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};

// use crossbeam_channel::bounded;

extern crate alloc;

type BuzChannel<'a> = Channel::<CriticalSectionRawMutex, BuzzerCommand<'a>, 3>;

/// Constants used on this stick
const DISPLAY_WIDTH: u16 = 135;
const DISPLAY_HEIGHT: u16 = 240;

// static BUTTON_A: csMutex<RefCell<Option<Input>>> = csMutex::new(RefCell::new(None));
// static BUTTON_B: csMutex<RefCell<Option<Input>>> = csMutex::new(RefCell::new(None));
// static BUTTON_C: csMutex<RefCell<Option<Input>>> = csMutex::new(RefCell::new(None));
static BUTTON_A: CriticalSectionMutex<RefCell<Option<Input>>> = CriticalSectionMutex::new(RefCell::new(None));
static BUTTON_B: CriticalSectionMutex<RefCell<Option<Input>>> = CriticalSectionMutex::new(RefCell::new(None));
static BUTTON_C: CriticalSectionMutex<RefCell<Option<Input>>> = CriticalSectionMutex::new(RefCell::new(None));


// Setup channels
// static  SOUND_CHANNEL: Channel::<CriticalSectionRawMutex, BuzzerCommand, 3> = Channel::new();
// static  SOUND_CHANNEL: Channel::<NoopRawMutex, BuzzerCommand, 3> = Channel::new();

#[embassy_executor::task(pool_size = 4)]
async fn run(tx: embassy_sync::channel::DynamicSender<'static, BuzzerCommand<'static>>) {
    loop {
        info!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
        match tx.try_send(BuzzerCommand::Pause) {
            Ok(_) => {
                info!("sending a message!");
                continue;
            },
            Err(e) => {
                error!("trouble sending: {}", e);
            },
        }
    }
}


#[embassy_executor::task(pool_size = 4)]
async fn sound_task(buzzer: Buzzer<'static>) {
    let mut buzzer_state = BuzzerState::default();
    loop {
        buzzer_state = buzzer.execute(buzzer_state).await;
    }
}


#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    let timer0: AnyTimer = timg1.timer0.into();
    let timer1: AnyTimer = timg1.timer1.into();
    esp_hal_embassy::init([timer0, timer1]);

    info!("Embassy initialized!");

    static SND_CHANNEL: StaticCell<BuzChannel> = StaticCell::new();
    let snd_channel = &*SND_CHANNEL.init(Channel::new());
    

    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timer1.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let mut io = Io::new(peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(handler);

    // Set GPIO19 as an output, and set its state low initially.
    let mut led = Output::new(peripherals.GPIO19, Level::Low, OutputConfig::default());

    // Get the peripherals for the buzzer
    let buzzer_pin = peripherals.GPIO2;
    let ledc = Ledc::new(peripherals.LEDC);

    // set up bus for i2c stuff
    let mut i2c_sda = peripherals.GPIO21;
    let mut i2c_scl = peripherals.GPIO22;
    let mut i2c_bus1 = peripherals.I2C1;
    let i2c1_cfg =  I2cConfig::default()
                .with_frequency(Rate::from_khz(400))
                .with_timeout(BusTimeout::Maximum);
    
    let i2c = I2c::new(
        i2c_bus1.reborrow(),
        i2c1_cfg,
        )
        .unwrap()
        .with_sda(i2c_sda.reborrow())
        .with_scl(i2c_scl.reborrow())
        .into_async();
    
    let i2c_bus = RefCell::new(i2c);
    let mut rtc = Pcf8563::new(RefCellDevice::new(&i2c_bus));
    let mut imu = Mpu6886::new(RefCellDevice::new(&i2c_bus));


    if let Err(_e) = imu.init() {
        error!("unable to init imu");
    }
    
    if let Ok(pwr_loss) = rtc.power_loss() {
        if pwr_loss {
            info!("RTC lost power!");
        } else {
            info!("RTC has kept power.");
        }
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
        BUTTON_A.borrow(cs).replace(Some(button_a));
        button_b.listen(Event::FallingEdge);
        BUTTON_B.borrow(cs).replace(Some(button_b));
        button_c.listen(Event::FallingEdge);
        BUTTON_C.borrow(cs).replace(Some(button_c));
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
    let text_y: i32 = (DISPLAY_HEIGHT / 2).into();

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

    display_bl.set_high();        // let s = self.state;


    // let (blah_tx, blah_rx) = crossbeam_channel::bounded::
    
    let snd_tx = snd_channel.dyn_sender();
    let snd_rx = snd_channel.dyn_receiver();

    let buzzer = Buzzer::new(ledc, buzzer_pin, snd_rx);

    // TODO: Spawn some tasks
    if let Err(e) = spawner.spawn(sound_task(buzzer)) {
        error!("unable to spawn sound_task: {}", e);
    }
    if let Err(e) = spawner.spawn(run(snd_tx)) {
        error!("unable to spawn run_task: {}", e);
    }


    // let ppsong = Song::new(pink_panther::TEMPO, &pink_panther::MELODY);
    // snd_tx.send(BuzzerCommand::Play(ppsong)).await;

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

        match rtc.datetime() {
            Ok(dt) => info!("datetime {:02}:{:02}:{:02} ", dt.hour, dt.minute, dt.second),
            Err(e) => {
                match e {
                    pcf8563::Error::I2cError(error_kind) => error!("i2c error: {}", error_kind),
                    pcf8563::Error::Other => error!("other error"),
                    _ => error!("mystery error"),
                }
            },
        }

        // info!("Hello world!");
        Timer::after(Duration::from_millis(750)).await;
        // match snd_tx.try_send(BuzzerCommand::Pause) {
        //     Ok(_) => {
        //         info!("sending a message!");
        //         continue;
        //     },
        //     Err(e) => {
        //         error!("trouble sending: {}", e);
        //     },
        // }

    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}


#[handler]
#[ram]
fn handler() {
    critical_section::with(|cs| {
        info!("GPIO interrupt");
        // if let Some(button_a) = BUTTON_A.borrow(cs).borrow_mut() {
        if let Some(button_a) = BUTTON_A.borrow(cs).borrow_mut().as_mut() {
            if button_a.is_interrupt_set() {
                button_a.clear_interrupt();
                info!("Button A");
            }
        }
        if let Some(button_b) = BUTTON_B.borrow(cs).borrow_mut().as_mut() {
            if button_b.is_interrupt_set() {
                button_b.clear_interrupt();
                info!("Button B");
            }
        }
        if let Some(button_c) = BUTTON_C.borrow(cs).borrow_mut().as_mut() {
            if button_c.is_interrupt_set() {
                button_c.clear_interrupt();
                info!("Button C");
            }
        }
    });
}