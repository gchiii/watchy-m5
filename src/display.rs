use core::fmt::Debug;

use allocator_api2::vec::Vec;
use defmt::error;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    draw_target::DrawTarget, 
    pixelcolor::Rgb565, 
    prelude::*,
};

use embedded_graphics_framebuf::backends::{DMACapableFrameBufferBackend, FrameBufferBackend};
use embedded_graphics_framebuf::FrameBuf;
#[cfg(feature = "mipidsi")] 
use embedded_graphics_framebuf::FrameBuf;

#[cfg(feature = "mipidsi")] 
use embedded_hal_async::spi::SpiBus;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
#[cfg(feature = "mipidsi")] 
use esp_hal::delay::Delay;

use esp_hal::dma::{DmaBufError, DmaError, DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::gpio::{Level, Output, OutputConfig, OutputPin};
#[cfg(feature = "lcdasync")]
use esp_hal::spi::master::SpiDmaBus;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;

use lcd_async::raw_framebuf::RawBufferBackendMut;
#[cfg(feature = "lcdasync")]
use lcd_async::raw_framebuf::RawFrameBuf;
#[cfg(feature = "mipidsi")] 
use mipidsi::{interface::{self, Interface, SpiError, SpiInterface}, models::ST7789, options::{ColorInversion, Orientation, Rotation}, Builder, Display};


#[cfg(feature = "lcdasync")] 
use embassy_time::Delay;
#[cfg(feature = "lcdasync")] 
use lcd_async::{interface::{self, Interface, SpiError, SpiInterface}, models::ST7789, options::{ColorInversion, Orientation, Rotation}, Builder, Display};
#[cfg(feature = "lcdasync")] 
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};

use thiserror_no_std::Error;

#[cfg(feature = "lcdasync")] 
pub type SpiBusMutex<'d> = Mutex<CriticalSectionRawMutex, SpiDmaBus<'d, esp_hal::Async>>;


#[cfg(feature = "lcdasync")] 
static SPI_BUS: StaticCell<SpiBusMutex<'static>> = StaticCell::new();


#[derive(Debug, Error, defmt::Format)]
pub enum DisplayError {
    ConfigError(#[from] esp_hal::spi::master::ConfigError),
    Infallible(#[from] core::convert::Infallible),
    SpiError(#[from] esp_hal::spi::Error),
    DmaError(#[from] DmaError),
    DmaBufError(#[from] DmaBufError),
    Digital(#[from] embedded_hal::digital::ErrorKind),
    OtherError,
}


impl From<interface::SpiError<esp_hal::spi::Error, embedded_hal::digital::ErrorKind>> for DisplayError {
    fn from(value: interface::SpiError<esp_hal::spi::Error, embedded_hal::digital::ErrorKind>) -> Self {
        
        match value {
            SpiError::Spi(e) => e.into(),
            SpiError::Dc(e) => e.into(),
        }
    }
}

impl From<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>> for DisplayError {
    fn from(value: embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>) -> Self {
        match value {
            DeviceError::Spi(e) => e.into(),
            DeviceError::Cs(e) => e.into(),
        }
    }
}

impl From<interface::SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>> for DisplayError {
    fn from(value: interface::SpiError<embedded_hal_bus::spi::DeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>) -> Self {
        match value {
            SpiError::Spi(e) => e.into(),
            SpiError::Dc(e) => e.into(),
        }
    }
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
#[cfg(feature = "lcdasync")]
const FRAME_BUFFER_SIZE: usize = WIDTH * HEIGHT * 2;

// Use StaticCell to create a static, zero-initialized buffer.
#[cfg(feature = "lcdasync")]
static FRAME_BUFFER: StaticCell<Vec<u8, esp_alloc::InternalMemory>> = StaticCell::new();
#[cfg(feature = "lcdasync")]
fn frame_buffer_init() -> &'static mut Vec<u8, esp_alloc::InternalMemory> {
    let buf: &'static mut Vec<u8, esp_alloc::InternalMemory> = FRAME_BUFFER.init_with(|| {
        let mut fb_vec = Vec::<u8, esp_alloc::InternalMemory>::with_capacity_in(FRAME_BUFFER_SIZE, esp_alloc::InternalMemory);
        fb_vec.resize(FRAME_BUFFER_SIZE, u8::default());
        fb_vec
    });
    buf
}

const DMA_BUFLEN: usize = 16_000;
// const BUFLEN: usize = 4096;
#[cfg(feature = "mipidsi")]
const BUFLEN: usize = 20 * WIDTH * size_of::<Rgb565>();
#[cfg(feature = "mipidsi")]
static DISPLAY_BUF: StaticCell<Vec<u8, esp_alloc::InternalMemory>> = StaticCell::new();

#[cfg(all(feature = "lcdasync", feature = "mipidsi"))]
compile_error!("feature \"lcdasyncoo\" and feature \"mipidsi\" cannot be enabled at the same time");
#[cfg(not(any(feature = "lcdasync", feature = "mipidsi")))]
compile_error!("must have either feature \"lcdasyncoo\" or feature \"mipidsi\" defined");


pub type EspSpiBusBlock<'d> = esp_hal::spi::master::Spi<'d, esp_hal::Blocking>;
pub type EspSpiDmaBusBlock<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Blocking>;
pub type EspSpiBusAsync<'d> = esp_hal::spi::master::Spi<'d, esp_hal::Async>;
pub type EspSpiDmaBusAsync<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Async>;
type ExclSpiDev<'d, SBus> = ExclusiveDevice<SBus, Output<'d>, Delay>;

pub type SpiDmaBusAsync = esp_hal::spi::master::SpiDmaBus<'static, esp_hal::Async>;
pub type StickDisplaySpiDmaBusAsync<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Async>;
pub type StickDisplaySpiDmaBusBlocking<'d> = esp_hal::spi::master::SpiDmaBus<'d, esp_hal::Blocking>;
#[cfg(feature = "lcdasync")]
pub type EmbassyAsyncSpiDev<'d> = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice<'d, CriticalSectionRawMutex, SpiDmaBusAsync, Output<'d>>;
// #[cfg(feature = "lcdasync")]
// pub type AsyncSpiInterface<'d> = SpiInterface<EmbassyAsyncSpiDev<'d>, Output<'d>>;

pub type StickDisplayExclusiveDevice<'d, SBus>  = embedded_hal_bus::spi::ExclusiveDevice<SBus, Output<'d>, Delay>;

#[cfg(feature = "lcdasync")] 
pub type StickDisplaySpiInterface<'d, SBus> = SpiInterface<StickDisplayExclusiveDevice<'d, SBus>, Output<'d>>;
#[cfg(feature = "mipidsi")]
pub type StickDisplaySpiInterface<'d, SBus> = SpiInterface<'d, StickDisplayExclusiveDevice<'d, SBus>, Output<'d>>;
pub type StickDisplayT<'d, SBus> = StickDisplay<'d, StickDisplaySpiInterface<'d, SBus>, Output<'d>>;
// type StickDrawTarget<'d> = CrazyDisplay<'d, StickDisplaySpiDmaBusAsync<'d>>;



#[derive(Debug)]
pub struct StickRawFrameBuf(pub &'static mut Vec<u8, esp_alloc::InternalMemory>);

impl StickRawFrameBuf {
    pub fn new(data: &'static mut Vec<u8, esp_alloc::InternalMemory>) -> Self {
        Self(data)
    }
}



impl RawBufferBackendMut for StickRawFrameBuf {
    fn as_mut_u8_slice(&mut self) -> &mut [u8] {
        self.0.as_mut_slice()            
    }

    fn as_u8_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    fn u8_len(&self) -> usize {
        self.0.len()
    }
}

#[derive(Debug)]
pub struct StickExtraFrameBuf<C: PixelColor + Default>(pub Vec<C, esp_alloc::InternalMemory>);

impl<C: PixelColor + Default> StickExtraFrameBuf<C> {
    pub fn new(width: usize, height: usize) -> Self {
        let data = Self::create_vec(width, height);
        Self(data)
    }
    
    fn create_vec(width: usize, height: usize) -> Vec<C, esp_alloc::InternalMemory> {
        let mut buf = Vec::<C, esp_alloc::InternalMemory>::with_capacity_in(width * height, esp_alloc::InternalMemory);
        buf.resize(width * height, C::default());        
        buf
    }

    fn rgb565_vec_to_byte_slice(rgb_vec: &Vec<C, esp_alloc::InternalMemory>) -> &[u8] {
        let len = rgb_vec.len() * core::mem::size_of::<C>();
        let ptr = rgb_vec.as_ptr() as *const u8;
        unsafe {
            core::slice::from_raw_parts(ptr, len)
        }
    }

    fn rgb565_vec_to_byte_slice_mut(rgb_vec: &mut Vec<C, esp_alloc::InternalMemory>) -> &mut [u8] {
        let len = rgb_vec.len() * core::mem::size_of::<C>();
        let ptr = rgb_vec.as_mut_ptr() as *mut u8;
        unsafe {
            core::slice::from_raw_parts_mut(ptr, len)
        }
    }
    
    pub fn data(&self) -> &Vec<C, esp_alloc::InternalMemory> {
        &self.0
    }

}

pub fn byte_slice_to_pixels<C: PixelColor>(bytes: &[u8]) -> &[C] {
    let len = bytes.len() / core::mem::size_of::<C>();
    let ptr = bytes.as_ptr() as *const C;
    unsafe {
        core::slice::from_raw_parts(ptr, len)
    }
}


impl<C: PixelColor + Default> RawBufferBackendMut for StickExtraFrameBuf<C> {
    fn as_mut_u8_slice(&mut self) -> &mut [u8] {
        Self::rgb565_vec_to_byte_slice_mut(&mut self.0)
    }

    fn as_u8_slice(&self) -> &[u8] {
        Self::rgb565_vec_to_byte_slice(&self.0)
    }

    fn u8_len(&self) -> usize {
        let len = self.0.len() * core::mem::size_of::<C>();
        len
    }
}

impl<C: PixelColor + Default> FrameBufferBackend for StickExtraFrameBuf<C> {
    type Color = C;

    fn set(&mut self, index: usize, color: Self::Color) {
        self.0[index] = color;
    }

    fn get(&self, index: usize) -> Self::Color {
        self.0[index]
    }

    fn nr_elements(&self) -> usize {
        self.0.len()
    }
}

unsafe impl<C: PixelColor + Default> DMACapableFrameBufferBackend for StickExtraFrameBuf<C> {
    fn data_ptr(&self) -> *const Self::Color {
        self.0.as_ptr()
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

pub struct StickDisplay<'d, DI, OP> 
where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
{
    pub d: Display<DI, ST7789, OP>,
    bl: Option<Output<'d>>,
    #[cfg(feature = "lcdasync")]
    pub fb: RawFrameBuf<Rgb565, StickRawFrameBuf>,
    // buffer: &'static mut Vec<Rgb565, esp_alloc::InternalMemory>,
    bounding_box: Rectangle,
}



#[cfg(feature = "mipidsi")]
impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
{
    fn new(d: Display<DI, ST7789, OP>, bl: Option<Output<'d>>) -> Self {
        // let buf: &'static mut Vec<Rgb565, esp_alloc::InternalMemory> = frame_buffer_init();
        let bounding_box = d.bounding_box();
        Self { 
            d, 
            bl,
            // buffer: buf,
            bounding_box,
        }
    }
}


#[cfg(feature = "lcdasync")]
impl<'d, DI, OP> StickDisplay<'d, DI, OP>
where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
{
    fn new(d: Display<DI, ST7789, OP>, bl: Option<Output<'d>>) -> Self {

        let bounding_box = Rectangle::new(Point::zero(), Size::new(WIDTH as u32, HEIGHT as u32));

        let buf: &'static mut Vec<u8, esp_alloc::InternalMemory> = frame_buffer_init();
        let buffer = StickRawFrameBuf::new(buf);
        let fb = RawFrameBuf::<Rgb565, _>::new(buffer, WIDTH.into(), HEIGHT.into());
        Self { 
            d, 
            bl,
            fb,
            bounding_box,
        }
    }
}

pub trait DrawAsync {

    // pub async fn draw_region(&mut self, r: &Rectangle) -> Result<(), SpiError<embassy_embedded_hal::shared_bus::SpiDeviceError<esp_hal::spi::Error, Infallible>, Infallible>> {
    fn draw_sub_region(&mut self, r: &Rectangle) -> impl core::future::Future<Output = Result<(), DisplayError>>;

    fn draw_screen(&mut self) -> impl core::future::Future<Output = Result<(), DisplayError>>;

    fn draw_area(&mut self, region: &Rectangle, data: &[u8]) -> impl core::future::Future<Output = Result<(), DisplayError>>;
}

#[cfg(feature = "lcdasync")]
impl<'d, DI, OP> DrawAsync for StickDisplay<'d, DI, OP>
    where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
    {
    
    async fn draw_sub_region(&mut self, region: &Rectangle) -> Result<(), DisplayError> {
        const BYTES_PER_PIXEL: usize = size_of::<Rgb565>();
        const BYTES_PER_ROW: usize = WIDTH * BYTES_PER_PIXEL;

        let fb_bytes = self.fb.as_bytes();
        if region.size < self.bounding_box.size {
            let (all_rows, _remainder) = fb_bytes.as_chunks::<BYTES_PER_ROW>();    
            let first_row = region.top_left.y as usize;
            let last_row = first_row + region.size.height as usize;
            let desired_rows = &all_rows[first_row..last_row];
            
            self.d.show_raw_data(
                0, 
                region.top_left.y as u16, 
                self.bounding_box.size.width as u16, 
                region.size.height as u16, 
                desired_rows.as_flattened()
            ).await.or(Err(DisplayError::OtherError))
        } else {
            self.d.show_raw_data(
                region.top_left.x as u16, 
                region.top_left.y as u16, 
                region.size.width as u16, 
                region.size.height as u16, 
                &fb_bytes
            ).await.or(Err(DisplayError::OtherError))
        }
    }
    
    async fn draw_screen(&mut self) -> Result<(), DisplayError> {
        let region = self.bounding_box;
        let fb_bytes = self.fb.as_bytes();
        self.d.show_raw_data(
            region.top_left.x as u16, 
            region.top_left.y as u16, 
            region.size.width as u16, 
            region.size.height as u16, 
            &fb_bytes
        ).await.or(Err(DisplayError::OtherError))
    }

    async fn draw_area(&mut self, region: &Rectangle, data: &[u8]) -> Result<(), DisplayError> {
        self.d.show_raw_data(
            region.top_left.x as u16, 
            region.top_left.y as u16, 
            region.size.width as u16, 
            region.size.height as u16, 
            &data
        ).await.or(Err(DisplayError::OtherError))
    }
}


pub trait Backlight {
    fn on(&mut self);

    fn off(&mut self);

}

impl<'d, DI, OP> Backlight for StickDisplay<'d, DI, OP>
where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
{
    fn on(&mut self) {
        if let Some(bl) = self.bl.as_mut() {
            bl.set_high();
        }
    }

    fn off(&mut self) {
        if let Some(bl) = self.bl.as_mut() {
            bl.set_low();
        }
    }

}

// impl<'d, DI: Interface<Word = u8>, OP: embedded_hal::digital::OutputPin> StickDisplay<'d, DI, OP> {
//     pub async fn fill_contiguous_a<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), DisplayError>
//     where
//         I: IntoIterator<Item = Rgb565>,
//     {
//         self.draw_iter(
//             area.points()
//                 .zip(colors)
//                 .map(|(pos, color)| Pixel(pos, color)),
//         )
//     }
// }


impl<DI, OP> Dimensions for StickDisplay<'_, DI, OP> 
where 
    DI: Interface<Word = u8>,
    OP: embedded_hal::digital::OutputPin,
{
    fn bounding_box(&self) -> embedded_graphics::primitives::Rectangle {
        self.bounding_box
    }
}


#[cfg(feature = "mipidsi")]
impl<DI, OP> DrawTarget for StickDisplay<'_, DI, OP> 
where 
    // DI: Interface<Word = u8, Error = SpiError<DeviceError<esp_hal::spi::Error, Infallible>, Infallible>>,
    DI: Interface<Word = u8>,
    // DI: Interface<Word = u8, Error = SpiError<esp_hal::spi::Error, embedded_hal::digital::ErrorKind>>,
    OP: embedded_hal::digital::OutputPin,
{
    type Color = Rgb565;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            // let mut fb = FrameBuf::new(self.buffer.as_mut_array::<{ WIDTH*HEIGHT }>().expect("really bad"), WIDTH, HEIGHT);
            // fb.draw_iter(pixels)?;
            if let Err(_e) = self.d.draw_iter(pixels) {
                return Err(DisplayError::OtherError);
                // match e {
                //     SpiError::Spi(e) => {
                //         return Err(e.into());
                //     },
                //     SpiError::Dc(e) => return Err(e.into()),
                // }
            }
            Ok(())
    }
}


#[cfg(feature = "lcdasync")]
impl<DI, OP> DrawTarget for StickDisplay<'_, DI, OP> 
where 
    DI: Interface<Word = u8, Error = SpiError<embassy_embedded_hal::shared_bus::SpiDeviceError<esp_hal::spi::Error, core::convert::Infallible>, core::convert::Infallible>>,
    OP: embedded_hal::digital::OutputPin,
{
    type Color = Rgb565;
    type Error = DisplayError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>> {
            if let Err(e) = self.fb.draw_iter(pixels) {
                return Err(DisplayError::Infallible(e));
            }
            Ok(())
    }
}



#[derive(Debug)]
pub enum EspSpiBlockingBus<'d> {
    NoDma(EspSpiBusBlock<'d>),
    Dma(EspSpiDmaBusBlock<'d>),
    None
}

impl<'d> EspSpiBlockingBus<'d> {
    pub fn try_into_no_dma(self) -> Result<EspSpiBusBlock<'d>, Self> {
        if let Self::NoDma(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }

    pub fn try_into_dma(self) -> Result<EspSpiDmaBusBlock<'d>, Self> {
        if let Self::Dma(v) = self {
            Ok(v)
        } else {
            Err(self)
        }
    }
}

impl<'d> From<EspSpiBusBlock<'d>> for EspSpiBlockingBus<'d> {
    fn from(value: EspSpiBusBlock<'d>) -> Self {
        EspSpiBlockingBus::NoDma(value)
    }
}

impl<'d> From<EspSpiDmaBusBlock<'d>> for EspSpiBlockingBus<'d> {
    fn from(value: EspSpiDmaBusBlock<'d>) -> Self {
        EspSpiBlockingBus::Dma(value)
    }
}

pub struct DisplayComponents<'d> {
    // pub spi_periph: esp_hal::spi::master::AnySpi<'static>,
    pub dc: Output<'d>,
    pub rst: Output<'d>,
    pub cs: Option<Output<'d>>,
    pub bl: Option<Output<'d>>,
    spi_bus: Option<EspSpiBusBlock<'d>>,
    dma_channel: Option<esp_hal::dma::AnySpiDmaChannel<'d>>,
}

impl<'d> DisplayComponents<'d> {

    pub fn with_spi(self, spi_periph: esp_hal::spi::master::AnySpi<'static>, sclk: impl OutputPin + 'd, mosi: impl OutputPin + 'd) -> Result<Self, DisplayError> {
        let config = Config::default()
            .with_frequency(Rate::from_mhz(20))
            .with_mode(Mode::_0);

        let spi_bus = Spi::new(spi_periph, config)?
            .with_sck(sclk)
            .with_mosi(mosi);        

        let dma = self.dma_channel;
        Ok(Self {
            spi_bus: Some(spi_bus),
            dma_channel: dma,
            ..self
        })
    }
    
    pub fn new(dc: impl OutputPin + 'd, rst: impl OutputPin + 'd, cs: impl OutputPin + 'd) -> Self {
        let dc = Output::new(dc, Level::Low, OutputConfig::default());
        let cs: Output<'_> = Output::new(cs, Level::High, OutputConfig::default());
        let rst = Output::new(rst, Level::Low, OutputConfig::default());
        Self {
            dc, 
            rst, 
            cs: Some(cs),
            bl: None,
            spi_bus: None,
            dma_channel: None,
        }
    }

    pub fn with_bl(self, bl: impl OutputPin + 'd) -> Self {
        let bl = Output::new(bl,Level::Low, OutputConfig::default());
        Self {
            bl: Some(bl),
            ..self
        }
    }

    pub fn with_dma(self, dma_channel: esp_hal::dma::AnySpiDmaChannel<'d>) -> Self {
        Self {
            dma_channel: Some(dma_channel),
            ..self
        }
    }

}


#[derive(Debug, defmt::Format)]
pub struct DisplayBuilder<'d, SBus> {
    // pub spi_periph: esp_hal::spi::master::AnySpi<'static>,
    dc: Output<'d>,
    rst: Output<'d>,
    cs: Option<Output<'d>>,
    bl: Option<Output<'d>>,
    dma_channel: Option<esp_hal::dma::AnySpiDmaChannel<'d>>,
    spi_instance: SBus,
}

impl<'d> DisplayBuilder<'d, EspSpiBusBlock<'d>> {
    pub fn from_components(components: DisplayComponents<'d>) -> Self {
        let dc = components.dc;
        let rst = components.rst;
        let cs = components.cs;
        let bl = components.bl;
        let spi_bus = components.spi_bus.expect("why no spi bus?");

        Self {
            dc,
            rst,
            cs,
            bl,
            spi_instance: spi_bus,
            dma_channel: components.dma_channel,
        }
    }

    pub fn try_into_dma(self) -> Result<DisplayBuilder<'d, EspSpiDmaBusBlock<'d>>, DisplayError> {
        let dc = self.dc;
        let rst = self.rst;
        let cs = self.cs;
        let bl = self.bl;
        let spi_instance = self.spi_instance;        
        let dma = self.dma_channel;

        let spi_bus = match dma {
            Some(dma_channel) => {
                let (dma_rx_buf, dma_tx_buf) = create_dma_buffers()?;
                spi_instance
                    .with_dma(dma_channel)            
                    .with_buffers(dma_rx_buf, dma_tx_buf)
            },
            None => return Err(DisplayError::OtherError),
        };

        
        Ok(DisplayBuilder::<'d, EspSpiDmaBusBlock<'d>> {
            dc,
            rst,
            cs,
            bl,
            spi_instance: spi_bus,
            dma_channel: None,
        })
    }
    
}

impl<'d> DisplayBuilder<'d, EspSpiBusBlock<'d>> {
    pub fn into_async(self) -> DisplayBuilder<'d, EspSpiBusAsync<'d>> {
        DisplayBuilder::<'d, EspSpiBusAsync<'d>> {
            spi_instance: self.spi_instance.into_async(),
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl,
            dma_channel: self.dma_channel,
        }
    }
}

impl<'d> DisplayBuilder<'d, EspSpiDmaBusBlock<'d>> {
    pub fn into_async(self) -> DisplayBuilder<'d, EspSpiDmaBusAsync<'d>> {
        DisplayBuilder::<'d, EspSpiDmaBusAsync<'d>> {
            spi_instance: self.spi_instance.into_async(),
            dc: self.dc,
            rst: self.rst,
            cs: self.cs,
            bl: self.bl,
            dma_channel: self.dma_channel,
        }
    }
}

impl<'d, SBus> DisplayBuilder<'d, SBus> {

    pub fn with_bl(self, bl: impl OutputPin + 'd) -> Self {
        let bl = Output::new(bl,Level::Low, OutputConfig::default());
        Self {
            bl: Some(bl),
            ..self
        }
    }
}

impl<'d, SBus> DisplayBuilder<'d, SBus> {
    pub fn to_exclusive_device(self) -> Result<DisplayBuilder<'d, ExclSpiDev<'d, SBus>>, DisplayError> {
        let dc = self.dc;
        let rst = self.rst;
        let cs = self.cs.unwrap();
        let bl = self.bl;
        let spi_instance = self.spi_instance;
        #[cfg(feature = "lcdasync")]
        let display_delay = embassy_time::Delay;
        #[cfg(feature = "mipidsi")]
        let display_delay = Delay::new();

        let spi_instance: ExclSpiDev<'d, SBus> = match ExclusiveDevice::new(spi_instance, cs, display_delay) {
            Ok(d) => d,
            Err(_e) => {
                error!("unable to make exclusive device");
                return Err(DisplayError::OtherError);
                // panic!("oops");bl
            },
        };
        Ok(
            DisplayBuilder::<'d, ExclSpiDev<'d, SBus>> {
                dc,
                rst,
                cs: None,
                bl,
                spi_instance,
                dma_channel: None,
            }
        )
    }
}



impl<'d> DisplayBuilder<'d, EspSpiDmaBusAsync<'static>> {
    pub fn to_mutex_dev(self) -> DisplayBuilder<'d, EmbassyAsyncSpiDev<'d>> {
        let dc = self.dc;
        let rst = self.rst;
        let cs = self.cs.unwrap();
        let bl = self.bl;
        // let spi_instance = self.spi_instance;

        // Create shared SPI bus
        // static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, EspSpiDmaBusAsync<'static>>> = StaticCell::new();
        let spi_bus: SpiBusMutex<'static> = Mutex::new(self.spi_instance);
        let spi_bus: &'static mut SpiBusMutex<'static> = SPI_BUS.init(spi_bus);
        let spi_instance: EmbassyAsyncSpiDev<'d> = embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(spi_bus, cs);

        DisplayBuilder::<'d, EmbassyAsyncSpiDev<'d>> {
            dc,
            rst,
            cs: None,
            bl,
            spi_instance,
            dma_channel: None,
        }
    }
}

#[cfg(feature = "lcdasync")]
impl<'d> DisplayBuilder<'d, EmbassyAsyncSpiDev<'d>> {
    pub async fn build(self) -> StickDisplay<'d, SpiInterface<EmbassyAsyncSpiDev<'d>, Output<'d>>, Output<'d>> {
        let dc = self.dc;
        let rst = self.rst;
        let bl = self.bl;
        // let spi_device = self.spi_instance;
        
        let di = interface::SpiInterface::new(self.spi_instance, dc);

        let rotation = Orientation::new().rotate(Rotation::Deg0);
        let mut display_delay = embassy_time::Delay;
        let d = Builder::new(ST7789, di)
            .reset_pin(rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay)
            .await;

        let display = match d {
            Ok(d) => d,
            Err(_e) => {
                error!("InitError");
                panic!();
            },
        };
        StickDisplay::new(display, bl)
        
    }
}

#[cfg(feature = "mipidsi")]
impl<'d, SBus: embedded_hal::spi::SpiBus> DisplayBuilder<'d, ExclSpiDev<'d, SBus>> {
    pub fn build_mipidsi(self) -> StickDisplay<'d, SpiInterface<'d, ExclSpiDev<'d, SBus>, Output<'d>>, Output<'d>> {
        let dc = self.dc;
        let rst = self.rst;
        let bl = self.bl;
        let spi_device = self.spi_instance;

        let di_buf = create_di_buf();
        
        let di = interface::SpiInterface::new(spi_device, dc, di_buf);

        let rotation = Orientation::new().rotate(Rotation::Deg0);
        let mut display_delay = Delay::new();
        let d = Builder::new(ST7789, di)
            .reset_pin(rst)
            .invert_colors(ColorInversion::Inverted)
            .display_size(WIDTH as u16, HEIGHT as u16)
            .display_offset(X_OFFSET, Y_OFFSET)
            .orientation(rotation)
            .init(&mut display_delay);            

        let display = match d {
            Ok(d) => d,
            Err(_e) => {
                error!("InitError");
                panic!();
            },
        };
        StickDisplay::new(display, bl)
        
    }
}

// fn create_exclusive<SBus>(spi: SBus, cs: esp_hal::gpio::Output<'_>) -> Result<ExclusiveDevice<SBus, esp_hal::gpio::Output<'_>, Delay>, DisplayError> {
//     let display_delay = Delay::new();

//     let exclusive_spidev = match ExclusiveDevice::new(spi, cs, display_delay) {
//         Ok(d) => d,
//         Err(_e) => {
//             error!("unable to make exclusive device");
//             return Err(DisplayError::OtherError);
//             // panic!("oops");
//         },
//     };
//     Ok(exclusive_spidev)
// }

fn create_dma_buffers() -> Result<(DmaRxBuf, DmaTxBuf), DisplayError> {
    // DMA transfers need descriptors and buffers
    #[allow(clippy::manual_div_ceil)]
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4, DMA_BUFLEN);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer)?;
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer)?;
    Ok((dma_rx_buf, dma_tx_buf))
}

#[cfg(feature = "mipidsi")]
fn create_di_buf<'d>() -> &'d mut [u8] {
    let disp_buf = DISPLAY_BUF.init_with(|| {
        let mut blah = Vec::<u8, esp_alloc::InternalMemory>::with_capacity_in(BUFLEN, esp_alloc::InternalMemory);
        blah.resize(BUFLEN, 0);
        blah
    });
    let di_buf = disp_buf.as_mut_slice();
    #[cfg(feature = "defmt")]
    defmt::trace!("buf_len = {}", di_buf.len());
    di_buf
}

