#![no_std]
#![no_main]
#![feature(slice_as_array)]

use allocator_api2::vec::Vec;
use defmt::{error, info, trace};
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embedded_canvas::Canvas;
use embedded_graphics_framebuf::FrameBuf;
use embedded_hal_bus::i2c::RefCellDevice;

use embedded_layout::{align::{horizontal, vertical, Align}, prelude::Chain, View};
use embedded_physics::{sprites::{Sprite, SpriteContainer}, vectors::{SpriteVector, VecComp, Velocity}};
use esp_alloc::HeapStats;
use esp_hal::{
    clock::CpuClock, gpio::{Input, InputConfig, Io, Level, Output, OutputConfig}, handler, i2c::master::{BusTimeout, Config as I2cConfig, I2c}, ledc::Ledc, ram, time::Rate, timer::timg::TimerGroup
};
    
use mpu6886::Mpu6886;
use pcf8563::Pcf8563;

use rand::{Rng, SeedableRng};
use static_cell::StaticCell;
use watchy_m5::{buttons::{btn_task, initialize_buttons, INPUT_BUTTONS}, display::{DisplayError, SpiBusMutex, StickDisplaySpiDmaBusAsync, StickDisplaySpiDmaBusBlocking, StickDisplayT, StickFrameBuf, StickFrameBuf2}};
use watchy_m5::display::{StickDisplay, StickDisplayBuilder, HEIGHT, WIDTH};
use watchy_m5::buzzer::{Buzzer, BuzzerChannel, BuzzerState};
use watchy_m5::music::Song;
use watchy_m5::player::{player_task, PlayerCmd, Player};
use watchy_m5::pink_panther;

use embassy_sync::{
    channel::Channel, mutex::Mutex} 
;

use {esp_backtrace as _, esp_println as _};

use core::cell::RefCell;
use embedded_graphics::{
    geometry::AnchorX, mono_font::{ascii::FONT_10X20, MonoTextStyle}, pixelcolor::{raw::LittleEndian, Rgb565}, prelude::*, primitives::{Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment, Triangle}, text::Text
};

use embedded_text::{
    alignment::HorizontalAlignment,
    style::{HeightMode, TextBoxStyleBuilder},
    TextBox,
};
// use embedded_layout::{layout::linear::LinearLayout, prelude::*};

// extern crate alloc;


// #[embassy_executor::task]
// async fn run() {
//     loop {
//         info!("Hello world from embassy using esp-hal-async!");
//         Timer::after(Duration::from_millis(1_000)).await;
//     }
// }

#[embassy_executor::task]
async fn sound_task(mut buzzer: Buzzer<'static>) {
    let mut buzzer_state = BuzzerState::default();
    loop {
        buzzer_state = buzzer.execute(buzzer_state).await;
    }
}



esp_bootloader_esp_idf::esp_app_desc!();
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);
    // esp_alloc::heap_allocator!(size: 72 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
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

        let bld = StickDisplayBuilder::new(peripherals.SPI2, sclk, mosi)
            .with_bl(backlight)
            .with_dma(peripherals.DMA_SPI2)
            .into_async()
            // .into_mutex()
            .create_display(dc, rst, cs);
            // .build_async(dc, rst, cs)
            // .await;
        bld
    };

    display.on();

    
    let snd_tx = snd_channel.dyn_sender();
    let snd_rx = snd_channel.dyn_receiver();

    let buzzer = Buzzer::new(ledc, buzzer_pin, snd_rx);

    // TODO: Spawn some tasks
    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    // static WINDOW: StaticCell<CCanvas<Rgb565, 125, 142>> = StaticCell::new();
    // let window = WINDOW.init_with(|| CCanvas::<Rgb565, 125, 142>::new());
    if let Err(e) = spawner.spawn(display_task(display, rng)) {
        error!("unable to spawn display_task: {}", e);
    }
    if let Err(e) = spawner.spawn(sound_task(buzzer)) {
        error!("unable to spawn sound_task: {}", e);
    }
    // if let Err(e) = spawner.spawn(run()) {
    //     error!("unable to spawn run_task: {}", e);
    // }

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
    // let mut counter = 0;
    loop {
        // counter += 1;

        // {
        //     // let mut display = get_raw_fb(frame_buffer);
        //     // Fill the display with alternating colors every 8 frames
        //     display.clear(colors[(counter / 8) % colors.len()]).unwrap();
        //     // Draw text
        //     let right = Text::new(text, Point::new(text_x, text_y), text_style)
        //         .draw(display.deref_mut())
        //         .unwrap();
        //     text_x = if right.x <= 0 { WIDTH.into() } else { text_x - 10 };
        // }
        //     text_box.draw(&mut display).unwrap();
        led.toggle();

        match rtc.datetime() {
            Ok(_dt) => {
                // info!("datetime {:02}:{:02}:{:02} ", dt.hour, dt.minute, dt.second);
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
        Timer::after(Duration::from_millis(1_000)).await;

    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}


type StickDrawTarget<'d> = StickDisplayT<'d, StickDisplaySpiDmaBusAsync<'d>>;
// type StickDrawTarget<'d> = StickDisplay<'d, MySpiInterface<'d>, Output<'d>>;
// type StickDrawTarget<'d> = CrazyDisplay<'d, StickDisplaySpiDmaBusBlocking<'d>>;

#[embassy_executor::task]
async fn display_task(display: StickDrawTarget<'static>, esp_rng: esp_hal::rng::Rng) {
    if let Err(e) = display_task_worker(display, esp_rng).await {
        error!("display error: {}!", e);
        panic!()
    }
}

// static FB_DATA: StaticCell<Vec<Rgb565, esp_alloc::InternalMemory>> = StaticCell::new();

async fn display_task_worker(mut display: StickDrawTarget<'static>, mut esp_rng: esp_hal::rng::Rng) -> Result<(), DisplayError>{

    // Alternating color
    let colors = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE];
    // Create styles used by the drawing operations.
    let thin_stroke = PrimitiveStyle::with_stroke(Rgb565::BLACK, 1);
    let thick_stroke = PrimitiveStyle::with_stroke(Rgb565::BLACK, 3);
    // let border_stroke = PrimitiveStyleBuilder::new()
    //     .stroke_color(Rgb565::BLACK)
    //     .stroke_width(3)
    //     .stroke_alignment(StrokeAlignment::Inside)
    //     .build();
    let ball_bound_sb = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLACK)
        .stroke_width(3)
        .stroke_alignment(StrokeAlignment::Inside)
        .fill_color(Rgb565::CYAN);
    let fill = PrimitiveStyle::with_fill(Rgb565::CYAN);

    let display_area = display.bounding_box().into_styled(thin_stroke);

    let text_box = {
        let text = "Blah blah blah blah!";
        let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Justified)
            .paragraph_spacing(6)
            .build();
        let tl = display_area.bounding_box().top_left + Point::new(3, 3);
        let bounds = Rectangle::new(tl, Size::new((display.bounding_box().size.width * 2)/3, 0));
        let mut t = TextBox::with_textbox_style(text, bounds, character_style, textbox_style);
        let outline = t.bounding_box().offset(3).into_styled(thick_stroke);
        t.align_to_mut(&outline, horizontal::Center, vertical::Center);
        Chain::new(outline)
            .append(t)
            .align_to(&display.bounding_box(), horizontal::Center, vertical::Top)
    };
    let mut hello_box = {
        let text_x: i32 = WIDTH as i32;
        let text_y: i32 = (HEIGHT / 2) as i32;
        let text = "Hello World ^_^;";
        let text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        let bounds = Text::new(text, Point::new(text_x, text_y), text_style)
            .bounding_box()
            .offset(3);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)            
            .alignment(HorizontalAlignment::Justified)
            .paragraph_spacing(6)
            .build();
        TextBox::with_textbox_style(text, bounds, text_style, textbox_style)
    };

    display.on();
    
    hello_box.align_to_mut(&text_box, horizontal::NoAlignment, vertical::TopToBottom);

    let bb_tl = Point::new(0,88);
    // let mut ball_canvas= Canvas::<Rgb565>::new(Size::new(135, 152)).place_at(bb_tl);
    // let bounce_box = ball_canvas.bounding_box();
    let bounce_box = Rectangle::new(bb_tl, Size::new(135, 152));


    let sub_window_r = Rectangle::with_corners(display.bounding_box().top_left, bounce_box.anchor_point(embedded_graphics::geometry::AnchorPoint::TopRight));
    info!("bounce_box: {:?}", bounce_box);

    let circle = Circle::with_center(bounce_box.bounding_box().center(), 15);
    let mut ball = Sprite::<Rgb565>::new("ball1", circle.translate(Point { x: 30, y: 25 }))
        .with_style(PrimitiveStyle::with_fill(Rgb565::YELLOW))
        .with_line_style(PrimitiveStyle::with_stroke(Rgb565::BLACK, 1));
    
    let mut ball2 = Sprite::<Rgb565>::new("ball2", circle.translate(Point { x: -30, y: -25 }))
        .with_style(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
        .with_line_style(thin_stroke);


    let mut fun_rng = rand::rngs::SmallRng::from_rng(&mut esp_rng);

    ball.set_direction_from_angle(Angle::from_degrees(fun_rng.random_range(0..360) as f32));
    ball2.set_direction_from_angle(Angle::from_degrees(fun_rng.random_range(0..360) as f32));
    let mut sprite_container = SpriteContainer::new(bounce_box);
    let _ = sprite_container.add_sprite(ball);
    let _ = sprite_container.add_sprite(ball2);
    // let stationary_shape = Rectangle::with_center(bounce_box.bounding_box().center(), Size::new_equal(25));
    let stationary_shape = Circle::with_center(bounce_box.bounding_box().center(), 25);
    let stationary = Sprite::new("stationary", stationary_shape)
        .with_style(fill)
        .with_line_style(thick_stroke);
    let _ = sprite_container.add_sprite(stationary);
    {
        let t_vertex1 = stationary_shape.top_left + Point::new(-40, -30);
        let mut blah = Sprite::new("circle", Circle::with_center(t_vertex1, 25))
            .with_style(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
            .with_line_style(thick_stroke);
        blah.set_direction_from_angle(Angle::from_degrees(fun_rng.random_range(0..360) as f32));
        let _ = sprite_container.add_sprite(blah);
    }
    {
        let t_vertex1 = stationary_shape.top_left + Point::new(-40, -30);
        let t_v2 = t_vertex1 + Point::new(-10, -25);
        let t_v3 = t_vertex1 + Point::new(10, -25);
        let mut blah = Sprite::new("triangle", Triangle::new(t_vertex1, t_v2, t_v3))
            .with_style(PrimitiveStyle::with_fill(Rgb565::MAGENTA))
            .with_line_style(thick_stroke);
        blah.set_direction_from_angle(Angle::from_degrees(fun_rng.random_range(0..360) as f32));
        let _ = sprite_container.add_sprite(blah);
    }

    let mut counter = 0;
    let mut last_hello_tick = Instant::now();
    let mut last_bounce_tick = Instant::now();
    let mut bb_style: PrimitiveStyle<Rgb565> = ball_bound_sb.build();
    let mut fb_data = StickFrameBuf2::create_vec( bounce_box.size.width as usize, bounce_box.size.height as usize);
    let fun = StickFrameBuf2::new(fb_data.as_mut_slice());
    let mut fb = FrameBuf::new(fun, bounce_box.size.width as usize, bounce_box.size.height as usize);
    // let mut fb = FrameBuf::new_with_origin(fun, bounce_box.size.width as usize, bounce_box.size.height as usize, Point::zero());
    loop {
        
        let bg_color = colors[(counter / 8) % colors.len()];
        if last_bounce_tick.elapsed() >= Duration::from_millis(50) {
            let window_r = bounce_box;
            
            let now = Instant::now();
            last_bounce_tick = now;
            sprite_container.update_positions();
            {
                let mut fb = fb.translated(-bounce_box.top_left);
                let bounce_box = bounce_box.into_styled(bb_style);
                bounce_box.draw(&mut fb)?;    
                sprite_container.draw(&mut fb)?;
            }
            
            let r = Rectangle::new(window_r.top_left, fb.size());
            let blah = fb.data.0.iter();
            if let Err(e) = display.fill_contiguous(&r, blah.copied()) {
                error!("problem: {}", e);
            }
            {
                let elapsed = now.elapsed();
                if elapsed.as_ticks() > 300 { info!("update_position plus draw took {}", elapsed); }
            }
        }
        
        if last_hello_tick.elapsed() >= Duration::from_secs(1) {
            counter += 1;
            let mut sub_window = display.clipped(&sub_window_r);
            // Fill the display with alternating colors every 8 frames
            sub_window.clear(bg_color)?;
            bb_style = ball_bound_sb.fill_color(bg_color).build();
            // Draw text
            if let Err(_e) = hello_box.draw(&mut sub_window) {
                error!("unable to draw hello_box.");
            }

            let translation = {
                if hello_box.bounds.anchor_x(AnchorX::Right) <= 0 {
                    Point::new(WIDTH as i32, 0)
                } else {
                    Point::new(-10, 0)
                }
            };
            embedded_graphics::prelude::Transform::translate_mut(&mut hello_box, translation);
            last_hello_tick = Instant::now();
            text_box.draw(&mut sub_window)?;
        }

        Timer::after(Duration::from_millis(50)).await;
    }
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