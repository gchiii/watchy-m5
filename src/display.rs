use core::convert::Infallible;
use core::fmt::Debug;
use core::ops::DerefMut;

use allocator_api2::vec::Vec;
use defmt::{error, info};
use embedded_graphics::{
    draw_target::DrawTarget, 
    pixelcolor::Rgb565, 
    prelude::*,
};

use embedded_graphics_framebuf::backends::FrameBufferBackend;
use embedded_graphics_framebuf::FrameBuf;

use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
#[cfg(feature = "mipidsi")] 
use esp_hal::delay::Delay;

use esp_hal::dma::{DmaChannelFor, DmaError, DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
use esp_hal::spi::master::{AnySpi, Config, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;

#[cfg(feature = "mipidsi")] 
use mipidsi::{interface::{self, Interface, InterfacePixelFormat, SpiError, SpiInterface}, models::ST7789, options::{ColorInversion, Orientation, Rotation}, Builder, Display};


#[cfg(feature = "lcdasync")] 
use embassy_time::Delay;
#[cfg(feature = "lcdasync")] 
use lcd_async::{interface::{self, Interface, SpiError, SpiInterface}, models::ST7789, options::{ColorInversion, Orientation, Rotation}, Builder, Display};

use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};

use thiserror_no_std::Error;


#[derive(Debug, Error, defmt::Format)]
pub enum DisplayError {
    ConfigError(#[from] esp_hal::spi::master::ConfigError),
    Infallible(#[from] core::convert::Infallible),
    SpiError(#[from] esp_hal::spi::Error),
    DmaError(#[from] DmaError),
    // Device(#[from] DeviceError<esp_hal::spi::Error, Infallible>),
    // SomeError(#[from] mipidsi::interface::spi::SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>),
    // SomeError(#[from] SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>),
    // InitError(#[from] InitError<SpiError<SpiDeviceError<esp_hal::spi::Error, Infallible>, Infallible>, Infallible>),
    OtherError,
}



// These are for vertical orientation
const X_OFFSET: u16 = 52;
const Y_OFFSET: u16 = 40;
pub const WIDTH: usize = 135;
pub const HEIGHT: usize = 240;

// These are for horizontal orientation
// const X_OFFSET: u16 = 203;
// const Y_OFFSET: u16 = 40;
// const WIDTH: u16 = 240;
// const HEIGHT: u16 = 135;

// Rgb565 uses 2 bytes per pixel
// const PXL_SIZE: usize = 2;
const FRAME_BUFFER_SIZE: usize = WIDTH * HEIGHT * 2;

// Use StaticCell to create a static, zero-initialized buffer.
static FRAME_BUFFER: StaticCell<Vec<Rgb565, esp_alloc::InternalMemory>> = StaticCell::new();

const PIXEL_ROW_LEN: usize = WIDTH;

// const BUFLEN: usize = 4096;
const BUFLEN: usize = 20 * WIDTH * size_of::<Rgb565>();
const DMA_BUFLEN: usize = 16_000;
// static DISPLAY_BUF: StaticCell<[u8; BUFLEN]> = StaticCell::new();
static DISPLAY_BUF: StaticCell<Vec<u8, esp_alloc::InternalMemory>> = StaticCell::new();

#[cfg(all(feature = "lcdasync", feature = "mipidsi"))]
compile_error!("feature \"lcdasyncoo\" and feature \"mipidsi\" cannot be enabled at the same time");
#[cfg(not(any(feature = "lcdasync", feature = "mipidsi")))]
compile_error!("must have either feature \"lcdasyncoo\" or feature \"mipidsi\" defined");

pub type SpiDmaBusAsync = esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>;
pub type StickDisplaySpiDmaBusAsync<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Async>;
pub type StickDisplaySpiDmaBusBlocking<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Blocking>;
#[cfg(feature = "lcdasync")]
pub type EmbassyAsyncSpiDev<'d> = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<'d, NoopRawMutex, SpiDmaBusAsync, Output<'d>>;
#[cfg(feature = "lcdasync")]
pub type AsyncSpiInterface<'d> = SpiInterface<EmbassyAsyncSpiDev<'d>, Output<'d>>;


pub type StickDisplayExclusiveDevice<'d, SBus>  = embedded_hal_bus::spi::ExclusiveDevice<SBus, Output<'d>, Delay>;

#[cfg(feature = "lcdasync")] 
pub type StickDisplaySpiInterface<'d, SBus> = SpiInterface<StickDisplayExclusiveDevice<'d, SBus>, Output<'d>>;
#[cfg(feature = "mipidsi")]
pub type StickDisplaySpiInterface<'d, SBus> = SpiInterface<'d, StickDisplayExclusiveDevice<'d, SBus>, Output<'d>>;
pub type StickDisplayT<'d, SBus> = StickDisplay<'d, StickDisplaySpiInterface<'d, SBus>, Output<'d>>;
// type StickDrawTarget<'d> = CrazyDisplay<'d, StickDisplaySpiDmaBusAsync<'d>>;



#[derive(Debug)]
pub struct StickDisplayBuilder<'d, SBus> {
    dc: Option<Output<'d>>,
    rst: Option<Output<'d>>,
    cs: Option<Output<'d>>,
    bl: Option<Output<'d>>,
    base_spi: SBus,
}

impl<'d> StickDisplayBuilder<'d, Spi<'d, esp_hal::Blocking>> {
    pub fn new(spi_periph: impl esp_hal::spi::master::Instance + 'static, sclk: impl OutputPin + 'd, mosi: impl OutputPin + 'd) -> Self {
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

impl<'d> StickDisplayBuilder<'d, StickDisplaySpiDmaBusBlocking<'d>> {
    pub fn into_async(self) -> StickDisplayBuilder<'d, StickDisplaySpiDmaBusAsync<'d>> {
        let s = self.base_spi.into_async();
        StickDisplayBuilder::<'d, StickDisplaySpiDmaBusAsync<'d>> {
            base_spi: s,
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl, 
        }
    }
}

impl<'d> StickDisplayBuilder<'d, Spi<'d, esp_hal::Blocking>> {
    pub fn with_dma(self, dma_channel: impl DmaChannelFor<AnySpi<'d>>) -> StickDisplayBuilder<'d, StickDisplaySpiDmaBusBlocking<'d>>  {
        // DMA transfers need descriptors and buffers
        #[allow(clippy::manual_div_ceil)]
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, DMA_BUFLEN);
        let dma_rx_buf = match DmaRxBuf::new(rx_descriptors, rx_buffer) {
            Ok(d) => d,
            Err(e) => {
                error!("DmaBufError: {}", e);
                panic!();
            },
        };
        let dma_tx_buf = match DmaTxBuf::new(tx_descriptors, tx_buffer) {
            Ok(d) => d,
            Err(e) => {
                error!("DmaBufError: {}", e);
                panic!();
            },
        };

        let base_spi = self.base_spi
            .with_dma(dma_channel)            
            .with_buffers(dma_rx_buf, dma_tx_buf);

        StickDisplayBuilder::<'d, StickDisplaySpiDmaBusBlocking<'d>> {
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
}

#[cfg(feature = "lcdasync")] 
pub type SpiBusMutex<'d> = Mutex<NoopRawMutex, SpiDmaBus<'d, esp_hal::Async>>;

#[cfg(feature = "lcdasync")] 
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
#[cfg(feature = "lcdasync")] 
use embassy_embedded_hal;

#[cfg(feature = "lcdasync")] 
static SPI_BUS: StaticCell<SpiBusMutex<'static>> = StaticCell::new();

#[cfg(feature = "lcdasync")] 
impl<'d> StickDisplayBuilder<'d, SpiDmaBusAsync> 
{
    pub fn into_mutex(self) -> StickDisplayBuilder<'d, &'static mut SpiBusMutex<'static>> {
        let mtx: Mutex<NoopRawMutex, SpiDmaBus<'static, esp_hal::Async>> = Mutex::new(self.base_spi);
        let spi = SPI_BUS.init(mtx);

        StickDisplayBuilder::<'d, &'static mut SpiBusMutex<'static>> {
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl,
            base_spi: spi,
        }
    }
}

#[cfg(feature = "lcdasync")] 
impl<'d> StickDisplayBuilder<'d, &'static mut SpiBusMutex<'static>> 
{
    pub async fn build_async(self, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl OutputPin + 'd) -> StickDisplay<'d, AsyncSpiInterface<'d>, Output<'d>> {
        // let mut display_delay = esp_hal::delay::Delay::new();

        let dc = Output::new(dc, Level::Low, OutputConfig::default());
        let cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let rst = Output::new(rst, Level::Low, OutputConfig::default());

        let spi_device = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(self.base_spi, cs);

        // Create display interface
        let di = SpiInterface::new(spi_device, dc);
        let mut delay = embassy_time::Delay;

        // Initialize the display
        let d = lcd_async::Builder::new(lcd_async::models::ST7789, di)
            .reset_pin(rst)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .orientation(lcd_async::options::Orientation {
                rotation: lcd_async::options::Rotation::Deg0,
                mirrored: false,
            })
            .display_offset(0, 0)
            .invert_colors(lcd_async::options::ColorInversion::Inverted)
            .init(&mut delay)
            .await;
        
        let display: Display<SpiInterface<embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<'_, NoopRawMutex, SpiDmaBus<'static, esp_hal::Async>, Output<'_>>, Output<'_>>, ST7789, Output<'_>> = match d {
            Ok(d) => d,
            Err(_e) => {
                // error!("{}", e);
                panic!();
            },
        };
        StickDisplay::new(display, self.bl)
    }

}

#[cfg(feature = "mipidsi")]
impl<'d, SBus> StickDisplayBuilder<'d, SBus> 
where 
    SBus: embedded_hal::spi::SpiBus,
    <SBus as embedded_hal::spi::ErrorType>::Error: defmt::Format
{

    pub fn create_display(self, dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl OutputPin + 'd) -> StickDisplay<'d, StickDisplaySpiInterface<'d, SBus>, Output<'d>> {
        let mut display_delay = esp_hal::delay::Delay::new();

        let dc = Output::new(dc, Level::Low, OutputConfig::default());
        let cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let rst = Output::new(rst, Level::Low, OutputConfig::default());

        let disp_device = ExclusiveDevice::new(self.base_spi, cs, display_delay).unwrap();

        let disp_buf = DISPLAY_BUF.init_with(|| {
            let mut blah = Vec::<u8, esp_alloc::InternalMemory>::with_capacity_in(BUFLEN, esp_alloc::InternalMemory);
            blah.resize(BUFLEN, 0);
            blah
        });
        let b = disp_buf.as_mut_slice();
        info!("buf_len = {}", b.len());
        let di = interface::SpiInterface::new(disp_device, dc, b);

        let rotation = Orientation::new().rotate(Rotation::Deg0);

        let d = Builder::new(ST7789, di)
            .reset_pin(rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay);            

        
        let display: mipidsi::Display<interface::SpiInterface<'d, ExclusiveDevice<SBus, Output<'d>, esp_hal::delay::Delay>, Output<'d>>, ST7789, Output<'d>> = match d {
            Ok(d) => d,
            Err(e) => {
                match e {
                    mipidsi::builder::InitError::Interface(e) => {
                        match e {
                            SpiError::Spi(e) => {
                                match e {
                                    DeviceError::Spi(_e) => {
                                        error!("oops: {}", _e);
                                    },
                                    DeviceError::Cs(e) => error!("{}", e),
                                }
                            },
                            SpiError::Dc(e) => error!("{}", e),
                        }
                    },
                    mipidsi::builder::InitError::ResetPin(e) => error!("{}", e),
                };
                panic!();
            },
        };
        StickDisplay::new(display, self.bl)
    }

}



// #[cfg(feature = "mipidsi")]
// pub trait StickDisplayInterface: Interface where Rgb565: InterfacePixelFormat<<DI as Interface>::Word> {}

// #[cfg(feature = "lcdasync")]
// pub trait StickDisplayInterface: Interface {}

pub struct StickDisplay<'d, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>,
{
    pub d: Display<DI, ST7789, OP>,
    bl: Option<Output<'d>>,
    #[cfg(feature = "lcdasync")]
    pub fb: FrameBuf<Rgb565, StickFrameBuf<'d>>,
    // fbuf: RawFrameBuf<Rgb565, &'d mut [u8]>,
}

#[cfg(feature = "lcdasync")]
impl<'d, DI, OP> DerefMut for StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.fb
    }
}
#[cfg(feature = "mipidsi")]
impl<'d, DI, OP> DerefMut for StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.d
    }
}

#[cfg(feature = "lcdasync")]
impl<'d, DI, OP> core::ops::Deref for StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
{
    type Target = FrameBuf<Rgb565, StickFrameBuf<'d>>;

    fn deref(&self) -> &Self::Target {
        &self.fb
    }
}

#[cfg(feature = "mipidsi")]
impl<'d, DI, OP> core::ops::Deref for StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    type Target = Display<DI, ST7789, OP>;

    fn deref(&self) -> &Self::Target {
        &self.d
    }
}


#[derive(Debug)]
pub struct StickFrameBuf<'a>(pub &'a mut [Rgb565]);


impl<'a> FrameBufferBackend for StickFrameBuf<'a> {
    type Color = Rgb565;


    fn set(&mut self, index: usize, color: Self::Color) {
        self.0[index] = color
    }

    fn get(&self, index: usize) -> Self::Color {
        self.0[index]
    }

    fn nr_elements(&self) -> usize {
        self.0.len()
    }
}

impl<'a> StickFrameBuf<'a> {
    pub fn new(buf: &'a mut [Rgb565]) -> Self {
        Self(buf)
    }
    pub fn create_vec(width: usize, height: usize) -> Vec<Rgb565, esp_alloc::InternalMemory> {
        let mut buf = Vec::<Rgb565, esp_alloc::InternalMemory>::with_capacity_in(width * height, esp_alloc::InternalMemory);
        buf.resize(width * height, Rgb565::default());        
        buf
    }
}


#[cfg(feature = "mipidsi")]
impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    fn new(d: Display<DI, ST7789, OP>, bl: Option<Output<'d>>) -> Self {
        Self { 
            d, 
            bl,
        }
    }
}

#[cfg(feature = "lcdasync")]
impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
{
    fn new(d: Display<DI, ST7789, OP>, bl: Option<Output<'d>>) -> Self {
    // let bb_width = bb_size.width as usize;
    // let bb_height = bb_size.height as usize;
    // let mut fb_data = StickFrameBuf::create_vec( bb_width, bb_height);
    // let fun = StickFrameBuf::new(fb_data.as_mut_slice());
    // let mut fb = FrameBuf::new(fun, bb_width, bb_height);

        let buf = FRAME_BUFFER.init_with(|| {
            let mut blah = Vec::<Rgb565, esp_alloc::InternalMemory>::with_capacity_in(FRAME_BUFFER_SIZE/2, esp_alloc::InternalMemory);
            blah.resize(FRAME_BUFFER_SIZE/2, Rgb565::default());
            blah
        });
        let buffer = StickFrameBuf::new(buf.as_mut_slice());
        let fb = FrameBuf::new(buffer, WIDTH, HEIGHT);
        Self { 
            d, 
            bl,
            fb
        }
    }

    // pub async fn draw_region(&mut self, r: &Rectangle) -> Result<(), SpiError<embassy_embedded_hal::shared_bus::SpiDeviceError<esp_hal::spi::Error, Infallible>, Infallible>> {
    //     self.d.show_raw_data(r.top_left.x.try_into().unwrap(), r.top_left.y.try_into().unwrap(), r.size.width.try_into().unwrap(), r.size.height.try_into().unwrap(), self.fbuf.as_bytes()).await
    // }
    
    // pub async fn draw_stuf(&mut self) -> Result<(), SpiError<embassy_embedded_hal::shared_bus::SpiDeviceError<esp_hal::spi::Error, Infallible>, Infallible>> {
    //     self.d.show_raw_data(0, 0, WIDTH as u16, HEIGHT as u16, self.fbuf.as_bytes())
    //         .await
    // }
}


impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
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

#[cfg(feature = "mipidsi")]
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

#[cfg(feature = "lcdasync")]
impl<DI, OP> Dimensions for StickDisplay<'_, DI, OP> 
where 
    DI: Interface,
    OP: embedded_hal::digital::OutputPin,
{
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        self.fb.bounding_box()
    }
}

#[cfg(feature = "mipidsi")]
impl<DI, OP> DrawTarget for StickDisplay<'_, DI, OP> 
where 
    DI: Interface<Error = SpiError<DeviceError<esp_hal::spi::Error, Infallible>, Infallible>>,
    OP: embedded_hal::digital::OutputPin,
    Rgb565: InterfacePixelFormat<<DI as Interface>::Word>
{
    type Color = Rgb565;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            if let Err(e) = self.d.draw_iter(pixels) {
                // return Err(DisplayError::Infallible(e));
                match e {
                    SpiError::Spi(e) => {
                        match e {
                            DeviceError::Spi(e) => {
                                error!("draw error! {}", e);
                                panic!();
                                return Err(DisplayError::SpiError(e))
                            },
                            DeviceError::Cs(e) => return Err(DisplayError::Infallible(e)),
                        }
                    },
                    SpiError::Dc(e) => return Err(DisplayError::Infallible(e)),
                }
            }
            Ok(())
    }
}

#[cfg(feature = "lcdasync")]
impl<DI, OP> DrawTarget for StickDisplay<'_, DI, OP> 
where 
    DI: Interface<Error = SpiError<DeviceError<esp_hal::spi::Error, Infallible>, Infallible>>,
    OP: embedded_hal::digital::OutputPin,
{
    type Color = Rgb565;
    type Error = DisplayError;
    // type Error = SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            if let Err(e) = self.fb.draw_iter(pixels) {
                return Err(DisplayError::Infallible(e));
            }
            Ok(())
    }
}



