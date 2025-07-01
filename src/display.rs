
use core::convert::Infallible;
use core::fmt::Debug;

use embedded_graphics::{
    draw_target::DrawTarget, 
    pixelcolor::Rgb565, 
    prelude::Dimensions
};

use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use esp_hal::dma::{DmaChannelFor, DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::spi::master::{AnySpi, Config, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;

use mipidsi::interface::{Interface, InterfacePixelFormat, SpiError};
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

const BUFLEN: usize = 4096;
static DISPLAY_BUF: StaticCell<[u8; BUFLEN]> = StaticCell::new();


pub struct StickDisplayBuilder<'d, SBus> {
    dc: Option<Output<'d>>,
    rst: Option<Output<'d>>,
    cs: Option<Output<'d>>,
    bl: Option<Output<'d>>,
    base_spi: SBus,
}

impl<'d> StickDisplayBuilder<'d, Spi<'d, esp_hal::Blocking>> {
    pub fn new(spi_periph: impl esp_hal::spi::master::Instance + 'd, sclk: impl OutputPin + 'd, mosi: impl OutputPin + 'd) -> Self {
        let base_spi = Spi::new(
            spi_periph, 
            Config::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(Mode::_0)
        ).unwrap()
            .with_sck(sclk)
            .with_mosi(mosi);

        Self {
            dc: None,
            rst: None,
            cs: None,
            bl: None,
            base_spi,
        }
    }
}

impl<'d> StickDisplayBuilder<'d, Spi<'d, esp_hal::Blocking>> {
    pub fn into_async(self) -> StickDisplayBuilder<'d, Spi<'d, esp_hal::Async>> {
        let s = self.base_spi.into_async();
        StickDisplayBuilder::<'d, Spi<'d, esp_hal::Async>> {
            base_spi: s,
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl, 
        }
    }
}

impl<'d> StickDisplayBuilder<'d, SpiDmaBus<'d, esp_hal::Blocking>> {
    pub fn into_async(self) -> StickDisplayBuilder<'d, SpiDmaBus<'d, esp_hal::Async>> {
        let s = self.base_spi.into_async();
        StickDisplayBuilder::<'d, SpiDmaBus<'d, esp_hal::Async>> {
            base_spi: s,
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl, 
        }
    }
}

impl<'d> StickDisplayBuilder<'d, Spi<'d, esp_hal::Blocking>> {
    pub fn with_dma(self, dma_channel: impl DmaChannelFor<AnySpi<'d>>) -> StickDisplayBuilder<'d, SpiDmaBus<'d, esp_hal::Blocking>>  {
        // DMA transfers need descriptors and buffers
        #[allow(clippy::manual_div_ceil)]
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, BUFLEN);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let base_spi = self.base_spi
            .with_dma(dma_channel)            
            .with_buffers(dma_rx_buf, dma_tx_buf);

        StickDisplayBuilder::<'d, SpiDmaBus<'d, esp_hal::Blocking>> {
            base_spi,
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl, 
        }
    }
}

impl<'d, SBus: embedded_hal::spi::SpiBus> StickDisplayBuilder<'d, SBus> {

    pub fn with_bl(self, bl: impl OutputPin + 'd) -> StickDisplayBuilder<'d, SBus> {
        let bl = Output::new(bl,Level::Low, OutputConfig::default());
        Self {
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: Some(bl),
            base_spi: self.base_spi,
        }
    }

    pub fn create_display(self, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl OutputPin + 'd) -> StickDisplay<'d, interface::SpiInterface<'d, ExclusiveDevice<SBus, Output<'d>, esp_hal::delay::Delay>, Output<'d>>, Output<'d>>{
        let mut display_delay = esp_hal::delay::Delay::new();

        let dc = Output::new(dc, Level::Low, OutputConfig::default());
        let cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let rst = Output::new(rst, Level::Low, OutputConfig::default());

        let disp_device = ExclusiveDevice::new(self.base_spi, cs, display_delay).unwrap();

        let disp_buf = DISPLAY_BUF.init([0_u8; BUFLEN]);
        let di = interface::SpiInterface::new(disp_device, dc, disp_buf);

        let rotation = Orientation::new().rotate(Rotation::Deg0);

        let display: mipidsi::Display<interface::SpiInterface<'d, ExclusiveDevice<SBus, Output<'d>, esp_hal::delay::Delay>, Output<'d>>, ST7789, Output<'d>> = Builder::new(ST7789, di)
            .reset_pin(rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH, HEIGHT)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay)
            .unwrap();
        StickDisplay::new(display, self.bl)
    }

}

pub struct StickDisplay<'d, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    d: mipidsi::Display<DI, ST7789, OP>,
    bl: Option<Output<'d>>,
}

impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    fn new(d: mipidsi::Display<DI, ST7789, OP>, bl: Option<Output<'d>>) -> Self {
        Self { d, bl }
    }

    pub fn on(&mut self) {
        if let Some(bl) = self.bl.as_mut() {
            bl.set_high();
        }
    }

    pub fn off(&mut self) {
        if let Some(bl) = self.bl.as_mut() {
            bl.set_low();
        }
    }

}

impl<DI, OP> Dimensions for StickDisplay<'_, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        self.d.bounding_box()
    }
}

impl<DI, OP> DrawTarget for StickDisplay<'_, DI, OP> 
where 
    DI: Interface<Error = SpiError<DeviceError<esp_hal::spi::Error, Infallible>, Infallible>>,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    type Color = Rgb565;
    type Error = SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            self.d.draw_iter(pixels)
    }
}

impl<DI, OP> core::ops::DerefMut for StickDisplay<'_, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.d
    }
}
impl<DI, OP> core::ops::Deref for StickDisplay<'_, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    type Target = mipidsi::Display<DI, ST7789, OP>;

    fn deref(&self) -> &Self::Target {
        &self.d
    }
}


