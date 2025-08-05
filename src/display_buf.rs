use esp_alloc as _;
use allocator_api2::alloc::{alloc_zeroed, handle_alloc_error, Allocator};
use allocator_api2::boxed::Box;
use allocator_api2::vec::Vec;
use defmt::{error, info};
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{
    draw_target::DrawTarget, 
    pixelcolor::Rgb565, 
    prelude::*,
};

use embedded_graphics_framebuf::backends::FrameBufferBackend;
use embedded_graphics_framebuf::FrameBuf;



use static_cell::StaticCell;

use crate::display::{HEIGHT, WIDTH};

use {esp_backtrace as _, esp_println as _};

use thiserror_no_std::Error;


/// create a vector of type 'T' with 'L' elements to be used as framebuffer storage
fn create_fb_vec<T: Default + Clone, const L: usize>() -> Vec<T, esp_alloc::InternalMemory> {
    info!("alloc vec");
    let mut fb_vec = Vec::<T, esp_alloc::InternalMemory>::with_capacity_in(L, esp_alloc::InternalMemory);
    info!("resize vec");
    fb_vec.resize(L, T::default());
    info!("return vec");
    fb_vec
}

fn create_heap_array<T: Default + Clone, const L: usize>() -> Box<[T; L], esp_alloc::InternalMemory> {
    // Create a Vec and convert it to a boxed slice
    let mut v = Vec::<T, esp_alloc::InternalMemory>::with_capacity_in(L, esp_alloc::InternalMemory);
    v.resize(L, T::default());
    let boxed_slice: Box<[T], esp_alloc::InternalMemory> = v.into_boxed_slice();

    // If you specifically need a Box<[T; N]>, you can transmute (use with caution)
    // This requires `unsafe` and careful handling to ensure the size matches.
    let boxed_array: Box<[T; L], esp_alloc::InternalMemory> = unsafe {
        let raw_ptr = Box::into_raw(boxed_slice) as *mut [T; L];
        Box::<[T; L], esp_alloc::InternalMemory>::from_raw_in(raw_ptr, esp_alloc::InternalMemory)
    };
    boxed_array
}

fn alloc_heap_array<T: Default + Clone, const L: usize>() -> Box<[T; L], esp_alloc::InternalMemory> {
    let b = Box::<[T; L], esp_alloc::InternalMemory>::new_uninit_in(esp_alloc::InternalMemory);
    unsafe {
        b.assume_init()
    }
    // let layout = Layout::from_size_align(size_of::<Rgb565>() * L, 4).expect("msg");
    // unsafe {
    //     // let ptr = alloc_zeroed(layout)
    //     let ptr = esp_alloc::InternalMemory.allocate_zeroed(layout).expect("very bad");
    //     let raw_u8_ptr = ptr.as_ptr();
    //     let len_u8 = ptr.len();
    //     defmt::info!("len_u8 = {}", len_u8);
    //     let raw_u16_ptr = raw_u8_ptr as *mut [u16];
    //     let len_u16 = len_u8 / 2;
    //     // let ptr = NonNull::<[u16]>::fr(raw_u16_ptr, len_u16);
    //     defmt::info!("len_T = {}", ptr.len());
        
        
    //     // if ptr.is_null() {
    //     //     handle_alloc_error(layout);
    //     // }
    //     // Box::<[T; L]>::from_raw_in(ptr, esp_alloc::InternalMemory)
    //     Box::<[Rgb565; L]>:: fr(ptr, esp_alloc::InternalMemory)
    // }
}


pub struct StickDisplayBuffer {
    buffer: Box<[Rgb565; WIDTH * HEIGHT], esp_alloc::InternalMemory>,
    bounding_box: Rectangle,
}

impl Dimensions for StickDisplayBuffer {
    fn bounding_box(&self) -> Rectangle {
        self.bounding_box
    }
}

impl StickDisplayBuffer {
    pub fn create() -> Self {
        const LENGTH: usize = WIDTH * HEIGHT;
        defmt::info!("creating stick buffer");
        let mut blah = create_fb_vec::<Rgb565, LENGTH>();
        let x: &mut [Rgb565; LENGTH] = blah.as_mut_array().expect("msg");
        let buf: Box<[Rgb565; LENGTH], esp_alloc::InternalMemory> = unsafe { Box::from_raw_in(x, esp_alloc::InternalMemory) };
        // let buf = alloc_heap_array::<Rgb565, LENGTH>();
        defmt::info!("got memory for stick buffer");
        let bounding_box = Rectangle::new(Point::zero(), Size::new(WIDTH as u32, HEIGHT as u32));
        
        Self { 
            buffer: buf,
            bounding_box,
        }
    }

    pub fn get_framebuffer(&mut self) -> FrameBuf<Rgb565, &mut [Rgb565; WIDTH * HEIGHT]> {
        FrameBuf::new(self.buffer.as_mut(), WIDTH, HEIGHT)
    }

}
