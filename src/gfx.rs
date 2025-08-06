// use core::{marker::PhantomData, ops::Deref};



use embedded_graphics::primitives::Line;
use embedded_graphics::prelude::*;
use micromath::F32;
use thiserror_no_std::Error;

use embedded_physics::{self, geometry::SurfaceNormal};




pub fn normal_angle_from_line(l: &Line) -> Angle {
    // Calculate the vector representing the line
    let delta = l.delta();

    // Calculate a perpendicular vector
    let normal_delta = Point::new(-delta.y, delta.x);

    // Calculate the angle in radians using atan2
    let angle_radians = F32(normal_delta.y as f32).atan2(F32(normal_delta.x as f32));

    // Create an Angle object from radians
    Angle::from_radians(angle_radians.into())
} 

pub enum Side {
    Top,
    Bottom,
    Left,
    Right,
}



#[derive(Debug, Error, defmt::Format)]
pub enum SpriteError {
    ConfigError(#[from] esp_hal::spi::master::ConfigError),
    Infallible(#[from] core::convert::Infallible),
    SpiError(#[from] esp_hal::spi::Error),
    OtherError,
}

pub trait Movable: Primitive + SurfaceNormal + Transform {
}
