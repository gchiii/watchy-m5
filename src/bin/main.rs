#![no_std]
#![no_main]

use defmt::{error, info, trace};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c::RefCellDevice;

use esp_alloc::HeapStats;
use esp_hal::{
    clock::CpuClock, 
    gpio::{Input, InputConfig, Io, Level, Output, OutputConfig}, 
    handler, 
    i2c::master::{BusTimeout, Config as I2cConfig, I2c}, 
    ledc::Ledc, 
    ram, time::Rate, timer::timg::TimerGroup
};
    
use mpu6886::Mpu6886;
use pcf8563::Pcf8563;

use static_cell::StaticCell;
use watchy_m5::{buttons::{btn_task, initialize_buttons, INPUT_BUTTONS}, display::StickDisplayBuilder};
use watchy_m5::buzzer::{Buzzer, BuzzerChannel, BuzzerState};
use watchy_m5::music::Song;
use watchy_m5::player::{player_task, PlayerCmd, Player};
use watchy_m5::pink_panther;

use embassy_sync::
    channel::Channel 
;

use {esp_backtrace as _, esp_println as _};

use core::{cell::RefCell, ops::DerefMut};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};


extern crate alloc;


/// Constants used on this stick
const DISPLAY_WIDTH: u16 = 135;
const DISPLAY_HEIGHT: u16 = 240;


#[embassy_executor::task(pool_size = 4)]
async fn run() {
    loop {
        info!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn sound_task(mut buzzer: Buzzer<'static>) {
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
    // esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let stats: HeapStats = esp_alloc::HEAP.stats();
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    // let timer0: AnyTimer = timg1.timer0.into();
    // let timer1: AnyTimer = timg1.timer1.into();
    // esp_hal_embassy::init([timer0, timer1]);
    esp_hal_embassy::init(timg1.timer0);
    info!("Embassy initialized!");

    static SND_CHANNEL: StaticCell<BuzzerChannel> = StaticCell::new();
    let snd_channel = &*SND_CHANNEL.init(Channel::new());
    

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
            trace!("RTC lost power!");
        } else {
            trace!("RTC has kept power.");
        }
    }

    // Set GPIO37 as an input
    let button_a: Input<'_> = Input::new(peripherals.GPIO37, InputConfig::default());
    // Set GPIO39 as an input
    let button_b = Input::new(peripherals.GPIO39, InputConfig::default());
    // Set GPIO35 as an input
    let button_c = Input::new(peripherals.GPIO35, InputConfig::default());



    // lets try to get the interface for the display
    let mut display = {
        let backlight = peripherals.GPIO27;
        let sclk = peripherals.GPIO13;
        let mosi = peripherals.GPIO15;
        let dc = peripherals.GPIO14;
        let rst = peripherals.GPIO12;
        let cs = peripherals.GPIO5;

        StickDisplayBuilder::new(peripherals.SPI2, sclk, mosi)
            .with_bl(backlight)
            .with_dma(peripherals.DMA_SPI2)
            .into_async()
            .create_display(dc, rst, cs) 
    };


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
    

    display.on();

    
    let snd_tx = snd_channel.dyn_sender();
    let snd_rx = snd_channel.dyn_receiver();

    let buzzer = Buzzer::new(ledc, buzzer_pin, snd_rx);

    // TODO: Spawn some tasks
    if let Err(e) = spawner.spawn(sound_task(buzzer)) {
        error!("unable to spawn sound_task: {}", e);
    }
    if let Err(e) = spawner.spawn(run()) {
        error!("unable to spawn run_task: {}", e);
    }

    if let Ok(btn_reader) = initialize_buttons(button_a, button_b, button_c) {
        if let Err(e) = spawner.spawn(btn_task(btn_reader)) {
            error!("unable to spawn run_task: {}", e);
        }

    }
    
 
    static CURRENT_SONG: StaticCell<Song> = StaticCell::new();
    let a_song = Song::new(pink_panther::TEMPO, &pink_panther::MELODY);
    let ppsong: &'static mut Song = CURRENT_SONG.init( a_song );

    let player_channel = Player::create_chan();
    let player_tx = player_channel.sender();
    let player_rx = player_channel.receiver();
    let player = Player::new(snd_tx, player_rx);

    if let Err(e) = spawner.spawn(player_task(player)) {
        error!("unable to spawn song task: {}", e);
    }

    if let Err(e) = player_tx.try_send(PlayerCmd::LoadSong(*ppsong)) {
        error!("oops: {}", e);
    }

    info!("{}", stats);
    let mut counter = 0;
    loop {
        counter += 1;

        {
            // let mut display = get_raw_fb(frame_buffer);
            // Fill the display with alternating colors every 8 frames
            display.clear(colors[(counter / 8) % colors.len()]).unwrap();
            
            // Draw text
            let right = Text::new(text, Point::new(text_x, text_y), text_style)
                .draw(display.deref_mut())
                .unwrap();
            text_x = if right.x <= 0 { DISPLAY_WIDTH.into() } else { text_x - 10 };
        }
        led.toggle();

        match rtc.datetime() {
            Ok(dt) => {
                info!("datetime {:02}:{:02}:{:02} ", dt.hour, dt.minute, dt.second);
            },
            Err(e) => {
                match e {
                    pcf8563::Error::I2cError(error_kind) => error!("i2c error: {}", error_kind),
                    pcf8563::Error::Other => error!("other error"),
                    _ => error!("mystery error"),
                }
            },
        }

        // a_display.show_raw_data(0, 0, watchy_m5::display::WIDTH, watchy_m5::display::HEIGHT, frame_buffer).await.unwrap();
        // info!("Hello world!");
        Timer::after(Duration::from_millis(750)).await;

    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}




#[handler]
#[ram]
fn handler() {
    critical_section::with(|cs| {
        info!("GPIO interrupt");
        if let Some(all_buttons) = INPUT_BUTTONS.borrow(cs).borrow_mut().as_mut() {
            all_buttons.interrupt_handler();
        }
    });
}