use defmt::error;
// use defmt::{error, info, trace};
// use embassy_executor::Spawner;
// use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
// use esp_hal::dma::{DmaChannelFor, DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::interconnect::PeripheralOutput;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::spi::master::{Config, Instance, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ST7789;
use mipidsi::options::{ColorInversion, Orientation};
use mipidsi::Builder;
use {esp_backtrace as _, esp_println as _};

// use embedded_graphics::{
//     mono_font::{ascii::FONT_10X20, MonoTextStyle},
//     pixelcolor::Rgb565,
//     prelude::*,
//     text::Text,
// };
use thiserror_no_std::Error;


#[derive(Debug, Error, defmt::Format)]
pub enum DisplayError {
    ConfigError(#[from] esp_hal::spi::master::ConfigError),
    Infallible(#[from] core::convert::Infallible),
    SpiError(#[from] esp_hal::spi::Error),
    InitError,
}



// These are for vertical orientation
const X_OFFSET: u16 = 52;
const Y_OFFSET: u16 = 40;
const WIDTH: u16 = 135;
const HEIGHT: u16 = 240;

// These are for horizontal orientation
// const X_OFFSET: u16 = 203;
// const Y_OFFSET: u16 = 40;
// const WIDTH: u16 = 240;
// const HEIGHT: u16 = 135;

// Rgb565 uses 2 bytes per pixel
// const PXL_SIZE: usize = 2;

// type DisplayBufType = u8;
// const DISPLAY_BUFFER_SIZE: usize = (WIDTH * HEIGHT) as usize * PXL_SIZE;
// static DISPLAY_BUFFER: StaticCell<[DisplayBufType; DISPLAY_BUFFER_SIZE]> = StaticCell::new();
// static DISPLAY_BUFFER: [DisplayBufType; DISPLAY_BUFFER_SIZE] = [0_u8; DISPLAY_BUFFER_SIZE];
// static DISPLAY_BUFFER: heapless::Vec<DisplayBufType, DISPLAY_BUFFER_SIZE> = heapless::Vec::new();
// static DISPLAY_BUFFER: StaticCell<Vec<DisplayBufType>> = StaticCell::new();

pub type ADisplay<'d> = mipidsi::Display<SpiInterface<'d, ExclusiveDevice<esp_hal::spi::master::Spi<'d, esp_hal::Async>, Output<'d>, embedded_hal_bus::spi::NoDelay>, Output<'d>>, ST7789, Output<'d>>;

pub struct TDisplay<'d> {
    d: ADisplay<'d>
}

impl core::ops::DerefMut for TDisplay<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.d
    }
}
impl<'d> core::ops::Deref for TDisplay<'d> {
    type Target = ADisplay<'d>;

    fn deref(&self) -> &Self::Target {
        &self.d
    }
}

impl<'d> TDisplay<'d> {
    pub fn new(p_spi: impl Instance + 'd, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl PeripheralOutput<'d> + OutputPin + 'd, sclk: impl PeripheralOutput<'d>, mosi: impl PeripheralOutput<'d>, buf: &'d mut [u8] ) -> Self {
        let display_dc = Output::new(dc, Level::Low, OutputConfig::default());
        let display_rst = Output::new(rst, Level::Low, OutputConfig::default());
        
        let display_spi: Spi<'_, esp_hal::Async> = Spi::new(
            p_spi, 
            Config::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0)
        ).unwrap()
        .with_sck(sclk)
        .with_mosi(mosi)
        .into_async();

        let display_cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let disp_device = ExclusiveDevice::new_no_delay(display_spi, display_cs).unwrap();

        let mut display_delay = esp_hal::delay::Delay::new();
        let di = SpiInterface::new(disp_device, display_dc, buf);
        let rotation = Orientation::new().rotate(mipidsi::options::Rotation::Deg0);
        let display = Builder::new(ST7789, di)
            .reset_pin(display_rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH, HEIGHT)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay)
            .unwrap();

        Self { d: display  }
    }

    pub fn create(p_spi: impl Instance + 'd, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl PeripheralOutput<'d> + OutputPin + 'd, sclk: impl PeripheralOutput<'d>, mosi: impl PeripheralOutput<'d>, buf: &'d mut [u8] ) -> Result<Self, DisplayError> {
        let display_dc = Output::new(dc, Level::Low, OutputConfig::default());
        let display_rst = Output::new(rst, Level::Low, OutputConfig::default());
        
        let display_spi: Spi<'_, esp_hal::Async> = Spi::new(
            p_spi, 
            Config::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0)
        )?
        .with_sck(sclk)
        .with_mosi(mosi)
        .into_async();

        let display_cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let disp_device = ExclusiveDevice::new_no_delay(display_spi, display_cs)?;

        let mut display_delay = esp_hal::delay::Delay::new();
        let di = SpiInterface::new(disp_device, display_dc, buf);
        let rotation = Orientation::new().rotate(mipidsi::options::Rotation::Deg0);
        let t_display = Builder::new(ST7789, di)
            .reset_pin(display_rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH, HEIGHT)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay);
        let display = match t_display {
            Ok(d) => Ok(d),
            Err(_e) => {
                error!("Init Error");
                Err(DisplayError::InitError)
            },
        }?;
        Ok(Self { d: display  })
    }

}


// pub fn build_display<'d>(p_spi: impl Instance + 'd, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, bl: impl OutputPin, cs: impl PeripheralOutput<'d> + OutputPin + embedded_hal::digital::OutputPin + 'd, sclk: impl PeripheralOutput<'d>, mosi: impl PeripheralOutput<'d>) -> mipidsi::Display<SpiInterface<'d, ExclusiveDevice<esp_hal::spi::master::Spi<'d, esp_hal::Async>, Output<'d>, embedded_hal_bus::spi::NoDelay>, Output<'d>>, ST7789, Output<'d>> {
pub fn build_display<'d>(p_spi: impl Instance + 'd, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl PeripheralOutput<'d> + OutputPin + 'd, sclk: impl PeripheralOutput<'d>, mosi: impl PeripheralOutput<'d>, display_buf: &'d mut [u8] ) -> ADisplay<'d> {
    // lets try to get the interface for the display
    let display_dc = Output::new(dc, Level::Low, OutputConfig::default());
    let display_rst = Output::new(rst, Level::Low, OutputConfig::default());
    
    let display_spi: Spi<'_, esp_hal::Async> = Spi::new(
        p_spi, 
        Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0)
    ).unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .into_async();

    let display_cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
    let disp_device = ExclusiveDevice::new_no_delay(display_spi, display_cs).unwrap();

    // Initialize the display_buf ONCE.
    // let display_buf = DISPLAY_BUFFER.init([0_u8; DISPLAY_BUFFER_SIZE]);
    // let display_buf = DISPLAY_BUFFER.init_with(|| {
    //     let mut db: Vec<DisplayBufType> = Vec::new();
    //     db.reserve_exact(512);
    //     db
    // });
    // let mut display_buf = DISPLAY_BUFFER;
    // let mut display_buf: Vec<DisplayBufType, _> = Vec::new_in(esp_alloc::ExternalMemory);
    // display_buf.reserve_exact(DISPLAY_BUFFER_SIZE);

    // let mut display_buf = [0_u8; 512];

    let mut display_delay = esp_hal::delay::Delay::new();
    let di = SpiInterface::new(disp_device, display_dc, display_buf);
    let rotation = Orientation::new().rotate(mipidsi::options::Rotation::Deg0);
    Builder::new(ST7789, di)
        .reset_pin(display_rst)
        .invert_colors(ColorInversion::Inverted)
        .display_size(WIDTH, HEIGHT)
        .display_offset(X_OFFSET, Y_OFFSET)
        .orientation(rotation)
        .init(&mut display_delay)
        .unwrap()

}