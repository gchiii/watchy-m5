use embedded_graphics::prelude::Point;
use no_std_compat2::f32;
use thiserror_no_std::Error;

extern crate micromath as mm;

extern crate nalgebra as na;
use na::Vector2;

pub mod vectors;
pub mod sprites;
pub mod geometry;

#[derive(Debug, Error, defmt::Format)]
pub enum SpriteError {
    ConfigError(#[from] esp_hal::spi::master::ConfigError),
    Infallible(#[from] core::convert::Infallible),
    SpiError(#[from] esp_hal::spi::Error),
    OtherError,
}

pub struct UnitVector<T> {
    pub value: T,
}


#[derive(Clone, Copy, Debug)]
pub struct SpritePoint(pub Vector2<f32>);

impl From<Point> for SpritePoint {
    fn from(value: Point) -> Self {
        let mut p = Vector2::zeros();
        p.x = value.x as f32;
        p.y = value.y as f32;
        SpritePoint::new(p)
    }
}

impl core::ops::DerefMut for SpritePoint {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for SpritePoint {
    type Target = Vector2<f32>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl SpritePoint {
    pub fn new(value: Vector2<f32>) -> Self {
        Self(value)
    }
}

impl defmt::Format for SpritePoint {
    fn format(&self, fmt: defmt::Formatter) {        
        defmt::write!(fmt, "{{ x: {}, y: {} }}", self.x, self.y)
    }
}

