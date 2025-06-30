
use core::fmt::Debug;

use embedded_graphics::{
    draw_target::DrawTarget, 
    pixelcolor::Rgb565, 
    prelude::Dimensions
};

use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::dma::{DmaChannelFor, DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::spi::master::{AnySpi, Config, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;

use mipidsi::{
    interface,
    Builder,
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
};
use static_cell::StaticCell;

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
    // InitError(#[from] InitError<SpiError<SpiDeviceError<esp_hal::spi::Error, Infallible>, Infallible>, Infallible>),
    OtherError,
}



// These are for vertical orientation
const X_OFFSET: u16 = 52;
const Y_OFFSET: u16 = 40;
pub const WIDTH: u16 = 135;
pub const HEIGHT: u16 = 240;

// These are for horizontal orientation
// const X_OFFSET: u16 = 203;
// const Y_OFFSET: u16 = 40;
// const WIDTH: u16 = 240;
// const HEIGHT: u16 = 135;

// Rgb565 uses 2 bytes per pixel
// const PXL_SIZE: usize = 2;


// type DisplayDelay = embedded_hal_bus::spi::NoDelay;
type DisplayDelay = esp_hal::delay::Delay;

type DisplayBus<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Async>;
type DisplayDevice<'d> = ExclusiveDevice<DisplayBus<'d>, Output<'d>, DisplayDelay>;
// pub type ADisplay<'d> = mipidsi::Display<mipidsi::interface::SpiInterface<'d, ExclusiveDevice<esp_hal::spi::master::Spi<'d, esp_hal::Async>, Output<'d>, DisplayDelay>, Output<'d>>, mipidsi::models::ST7789, Output<'d>>;
pub type ADisplay<'d> = mipidsi::Display<mipidsi::interface::SpiInterface<'d, ExclusiveDevice<esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Async>, Output<'d>, DisplayDelay>, Output<'d>>, mipidsi::models::ST7789, Output<'d>>;
pub type BDisplay<'d> = mipidsi::Display<mipidsi::interface::SpiInterface<'d, DisplayDevice<'d>, Output<'d>>, mipidsi::models::ST7789, Output<'d>>;


// struct Blah<Iface, Rst: embedded_hal::digital::OutputPin> 
// where 
//     Iface: mipidsi::interface::Interface,
//     Rgb565: InterfacePixelFormat<<Iface as mipidsi::interface::Interface>::Word>
// {
//     d: mipidsi::Display<Iface, ST7789, Rst>,
// }

pub struct TDisplay<'d> {
    d: ADisplay<'d>
}

impl Dimensions for TDisplay<'_> {
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        self.d.bounding_box()
    }
}

impl DrawTarget for TDisplay<'_> {
    type Color = Rgb565;
    type Error = mipidsi::interface::SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            self.d.draw_iter(pixels)
    }
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

const BUFLEN: usize = 4096;
static DISPLAY_BUF: StaticCell<[u8; BUFLEN]> = StaticCell::new();

impl<'d> TDisplay<'d> {

    #[allow(clippy::too_many_arguments)]
    pub fn new<SPI, DMACHAN>(p_spi: SPI, dma_channel: DMACHAN, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl OutputPin + 'd, sclk: impl OutputPin + 'd, mosi: impl OutputPin + 'd) -> Self 
    where 
        SPI: esp_hal::spi::master::Instance + 'd,
        DMACHAN: DmaChannelFor<AnySpi<'d>>,
    {
        let display_dc = Output::new(dc, Level::Low, OutputConfig::default());
        let display_rst = Output::new(rst, Level::Low, OutputConfig::default());
        
        // DMA transfers need descriptors and buffers
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, BUFLEN);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let display_spi = Spi::new(
            p_spi, 
            Config::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0)
        ).unwrap()
        .with_sck(sclk)
        .with_mosi(mosi)
        .with_dma(dma_channel)
        .with_buffers(dma_rx_buf, dma_tx_buf)
        .into_async();

        let mut display_delay = esp_hal::delay::Delay::new();
        let display_cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());

        let disp_device = ExclusiveDevice::new(display_spi, display_cs, display_delay).unwrap();

        let disp_buf = DISPLAY_BUF.init([0_u8; BUFLEN]);

        let di = interface::SpiInterface::new(disp_device, display_dc, disp_buf);
        let rotation = Orientation::new().rotate(Rotation::Deg0);
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

    #[allow(clippy::too_many_arguments)]
    pub fn create<SPI, DMACHAN, DC, RST, CS, SCLK, MOSI>(p_spi: SPI, dma_channel: DMACHAN, dc: DC, rst: RST, cs: CS, sclk: SCLK, mosi: MOSI) -> Result<Self, DisplayError> 
    where 
        SPI: esp_hal::spi::master::Instance + 'd,
        DMACHAN: DmaChannelFor<AnySpi<'d>>,
        DC: OutputPin + 'd,
        RST: OutputPin + 'd,
        CS: OutputPin + 'd,
        SCLK: OutputPin + 'd,
        MOSI: OutputPin + 'd
    {
        Ok(Self::new(p_spi, dma_channel, dc, rst, cs, sclk, mosi))
    }


}

