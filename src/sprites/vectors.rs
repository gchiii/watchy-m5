use core::ops::{Add, Div, Mul, Sub, Neg};
use core::fmt::Debug;
use embedded_graphics::prelude::Point;

use micromath::vector::{Vector, Vector2d};
use num_traits::{self, Float, FromPrimitive, NumCast, Pow, ToPrimitive};

// use defmt::info;
// use thiserror_no_std::Error;

extern crate micromath as mm;
extern crate nalgebra as na;

use na::Vector2;

use crate::sprites::geometry::PointExt;

#[derive(Clone, Copy, Debug)]
pub struct GfxVector(pub Vector2<f32>);

impl GfxVector {
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vector2::<f32>::new(x, y))
    }
}

// impl From<SpriteVector> for GfxVec {
//     fn from(value: SpriteVector) -> Self {
//         GfxVec::new(value.x, value.y)
//     }
// }
// impl From<GfxVec> for SpriteVector {
//     fn from(value: GfxVec) -> Self {
//         SpriteVector::new(value.x, value.y)
//     }
// }

impl From<Vector2<f32>> for GfxVector {
    fn from(value: Vector2<f32>) -> Self {
        GfxVector(value)
    }
}

impl From<GfxVector> for Point {
    fn from(value: GfxVector) -> Self {
        Point { x: value.x as i32, y: value.y as i32 }
    }
}

impl defmt::Format for GfxVector {
    fn format(&self, fmt: defmt::Formatter) {        
        defmt::write!(fmt, "{{ x: {}, y: {} }}", self.x, self.y)
    }
}

impl core::ops::DerefMut for GfxVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for GfxVector {
    type Target = Vector2<f32>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}


/// Let's define some stuff for handling vector manipulations for object collisions
pub trait VectorComponent:
    Copy
    + Debug
    + Default
    + PartialEq
    + PartialOrd
    + Send
    + Sized
    + Sync
    // + NumOps
    + Pow<u8>
    + NumCast
    + Neg
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + ToPrimitive
    + FromPrimitive
    // + Neg<Output = Self>
{
}
impl VectorComponent for i8 {}
impl VectorComponent for i16 {}
impl VectorComponent for i32 {}
impl VectorComponent for f32 {}

// `i32: core::convert::From<f32>`
// `f32: core::convert::From<i32>`

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default, defmt::Format)]
pub struct SpriteVector<T: VectorComponent> {    
    pub x: T,
    pub y: T,
}

impl<T: VectorComponent + core::convert::From<i32>> From<Point> for SpriteVector<T> {
    fn from(value: Point) -> Self {
        SpriteVector {
            x: value.x.into(),
            y: value.y.into(),
        }
    }
}

impl<T: VectorComponent> From<SpriteVector<T>> for Point where i32: From<T> {
    fn from(value: SpriteVector<T>) -> Self {
        Point { x: value.x.into(), y: value.y.into() }
    }
}

impl<T: VectorComponent> Mul<SpriteVector<T>> for f32 {
    type Output = SpriteVector<T>;

    fn mul(self, rhs: SpriteVector<T>) -> Self::Output {
        let x = <f32 as num_traits::NumCast>::from(rhs.x).unwrap() * self;
        let y = <f32 as num_traits::NumCast>::from(rhs.y).unwrap() * self;
        SpriteVector {
            x: <T as num_traits::NumCast>::from(x).unwrap(),
            y: <T as num_traits::NumCast>::from(y).unwrap(),
        }
    }
}

impl<T: VectorComponent> Mul<T> for SpriteVector<T> {
    type Output = Self;

    fn mul(self, rhs: T) -> Self::Output {
        SpriteVector {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T: VectorComponent> Sub<SpriteVector<T>> for &SpriteVector<T> {
    type Output = SpriteVector<T>;

    fn sub(self, rhs: SpriteVector<T>) -> Self::Output {
        SpriteVector { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}
impl<T: VectorComponent> Sub<&SpriteVector<T>> for SpriteVector<T> {
    type Output = Self;

    fn sub(self, rhs: &Self) -> Self::Output {
        Self { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}
impl<T: VectorComponent> Sub for SpriteVector<T> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self { x: self.x - rhs.x, y: self.y - rhs.y }
    }
}
impl<T: VectorComponent> Add for SpriteVector<T> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self { x: self.x + rhs.x, y: self.y + rhs.y }
    }
}


trait NewTrait<T: VectorComponent + Neg<Output = T>> where SpriteVector<T>: Neg<Output = SpriteVector<T>> {
    fn new(x: T, y: T) -> Self;

    fn magnitude(&self) -> T;

    fn dot_product(&self, other: &Self) -> T;

    // fn distance_squared(&self, other: &Self) -> T {
    //     let delta_x = self.x - other.x;
    //     let delta_y = self.y - other.y;
    //     (delta_x * delta_x) + (delta_y * delta_y)
    // }

    // fn distance(&self, other: &Self) -> T {
    //     let d = self.distance_squared(other);

    //     let d = <f32 as num_traits::NumCast>::from(d).unwrap().sqrt();
    //     <T as num_traits::NumCast>::from(d).unwrap()
    // }
    
    fn normalize(self) -> Self;

    fn rotate90(self) -> Self;

    fn calculate_reflection_vector(
        incoming_velocity: &Self,
        collision_normal: &Self,
        coefficient_of_restitution: f32,
    ) -> Self;    

}

impl<T: VectorComponent + Neg<Output = T>> NewTrait<T> for SpriteVector<T> where SpriteVector<T>: Neg<Output = SpriteVector<T>> {
    fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    fn magnitude(&self) -> T {
        let m: T = (self.x * self.x) + (self.y * self.y);
        let m = <f32 as num_traits::NumCast>::from(m).unwrap().sqrt();
        <T as num_traits::NumCast>::from(m).unwrap()
    }

    fn dot_product(&self, other: &Self) -> T {
        (self.x * other.x) + (self.y * other.y)
    }

    // fn distance_squared(&self, other: &Self) -> T {
    //     let delta_x = self.x - other.x;
    //     let delta_y = self.y - other.y;
    //     (delta_x * delta_x) + (delta_y * delta_y)
    // }

    // fn distance(&self, other: &Self) -> T {
    //     let d = self.distance_squared(other);

    //     let d = <f32 as num_traits::NumCast>::from(d).unwrap().sqrt();
    //     <T as num_traits::NumCast>::from(d).unwrap()
    // }
    
    fn normalize(self) -> Self {
        let mag = self.magnitude();
        let mut norm = self.clone();
        norm.x = self.x / mag;
        norm.y = self.y / mag;
        norm
    }

    fn rotate90(self) -> Self {
        let x = -self.y;
        let y = self.x;
        Self { x, y }
    }

    fn calculate_reflection_vector(
        incoming_velocity: &Self,
        collision_normal: &Self,
        coefficient_of_restitution: f32,
    ) -> Self {
        // Ensure the normal is normalized (unit length)
        let normal = collision_normal.normalize();

        // Calculate the component of the incoming velocity perpendicular to the collision surface
        let perpendicular_velocity = normal * incoming_velocity.dot_product(&normal);

        // Calculate the component of the incoming velocity parallel to the collision surface
        let parallel_velocity = incoming_velocity - perpendicular_velocity;

        // The reflected perpendicular velocity is reversed and scaled by the COR
        let reflected_perpendicular_velocity = coefficient_of_restitution * (-perpendicular_velocity);

        // The reflected velocity is the sum of the reflected perpendicular and parallel components
        reflected_perpendicular_velocity + parallel_velocity
    }    

}

pub trait VecNormalize {
    fn normalize(&self) -> Self;
}

impl<T> VecNormalize for Vector2d<T> 
where 
    T: micromath::vector::Component + core::convert::From<f32>,
    f32: core::convert::From<T>,
{
    fn normalize(&self) -> Self {
        let mag = self.magnitude();
        let x = <f32 as core::convert::From<T>>::from(self.x) / mag;
        let y = <f32 as core::convert::From<T>>::from(self.y) / mag;
        Vector2d { x: x.into(), y: y.into()}
    }
}

impl VecNormalize for Point {
    fn normalize(&self) -> Self {
        let mag = (self.x.pow(2) + self.y.pow(2)).isqrt();
        Self { x: self.x / mag, y: self.y / mag }
    }
}


fn normalize_vector2d_f32(a_vec: micromath::vector::Vector2d<f32>) -> micromath::vector::Vector2d<f32> {
    let mag = micromath::vector::Vector::magnitude(a_vec);
    micromath::vector::Vector2d { x: a_vec.x / mag, y: a_vec.y / mag }
}

fn calculate_reflection_vector<T>(
    incoming_velocity: &Vector2d<T>,
    collision_normal: &Vector2d<T>,
    coefficient_of_restitution: f32,
) -> Vector2d<T> 
where 
    Vector2d<T>: VecNormalize + Mul<f32, Output = Vector2d<T>> + Mul<T, Output = Vector2d<T>>,
    T: micromath::vector::Component + core::ops::Neg<Output = T>, 
    <T as Neg>::Output: Mul<f32>, 
    T: From<f32> + Mul<f32, Output = T>, 
{
    // Ensure the normal is normalized (unit length)
    // let mag = collision_normal.magnitude()
    let normal: Vector2d<T> = collision_normal.normalize();

    // Calculate the component of the incoming velocity perpendicular to the collision surface
    let perpendicular_velocity: Vector2d<T> = normal * incoming_velocity.dot(normal);
    let neg_x = -perpendicular_velocity.x;
    let neg_y = -perpendicular_velocity.y;
    let neg_perpendicular_velocity: Vector2d<T> = Vector2d { x: neg_x , y: neg_y };

    // Calculate the component of the incoming velocity parallel to the collision surface
    let parallel_velocity: Vector2d<T> = *incoming_velocity - perpendicular_velocity;

    // The reflected perpendicular velocity is reversed and scaled by the COR
    let reflected_perpendicular_velocity: Vector2d<T> = neg_perpendicular_velocity * coefficient_of_restitution;

    // The reflected velocity is the sum of the reflected perpendicular and parallel components
    reflected_perpendicular_velocity + parallel_velocity
}    

pub fn calculate_reflection_vector_p(
    incoming_velocity: &Point,
    collision_normal: &Point,
    coefficient_of_restitution: f32,
) -> Point {
    // Ensure the normal is normalized (unit length)
    // let mag = collision_normal.magnitude()
    let normal = collision_normal.normalize();

    // Calculate the component of the incoming velocity perpendicular to the collision surface
    let perpendicular_velocity = normal * incoming_velocity.dot_product(normal);
    let neg_x = -perpendicular_velocity.x;
    let neg_y = -perpendicular_velocity.y;
    let neg_perpendicular_velocity = Point { x: neg_x , y: neg_y };

    // Calculate the component of the incoming velocity parallel to the collision surface
    let parallel_velocity = *incoming_velocity - perpendicular_velocity;

    // The reflected perpendicular velocity is reversed and scaled by the COR
    let reflected_perpendicular_velocity = neg_perpendicular_velocity * coefficient_of_restitution as i32;

    // The reflected velocity is the sum of the reflected perpendicular and parallel components
    reflected_perpendicular_velocity + parallel_velocity
}    

