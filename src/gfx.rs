// use core::{marker::PhantomData, ops::Deref};

use core::{marker::PhantomData, ops::{Add, Mul, Neg}};


use allocator_api2::{boxed::Box, vec::Vec};
use defmt::info;
use embassy_time::Instant;
use embedded_graphics::{
    geometry::{AnchorPoint, AnchorX, AnchorY}, 
    pixelcolor::{BinaryColor, Rgb565}, 
    primitives::{self, Circle, Line, Polyline, PrimitiveStyle, Rectangle, Styled, StyledDrawable, Triangle}, 
    Drawable
};
use embedded_graphics::prelude::*;
use micromath::{vector::{Vector, Vector2d}, F32};
use micromath::F32Ext;
use no_std_compat2::i32;
use thiserror_no_std::Error;

use crate::sprites::{geometry::{Area, PointExt, SurfaceNormal}, sprites::SpritePrimitive, vectors::{calculate_reflection_vector_p, SpriteVector, VecNormalize, VectorComponent}};


// /// Converts a polar coordinate (angle/distance) into an (X, Y) coordinate centered around the
// /// center of the circle.
// ///
// /// The angle is relative to the 12 o'clock position and the radius is relative to the edge of the
// /// clock face.
// fn polar(circle: &Circle, angle: &Angle, radius_delta: i32) -> Point {
//     let radius = circle.diameter as f32 / 2.0 + radius_delta as f32;
//     let angle = F32::from(angle.to_radians());

//     circle.center()
//         + Point::new(
//             (angle.sin().0 * radius) as i32,
//             -(angle.cos().0 * radius) as i32,
//         )
// }



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


    // pub fn normalize(self) -> GfxVec {
    //     let x = self.x * self.x;
    //     let y = self.y * self.y;
    //     let sum = F32((x + y) as f32);
    //     let mag = sum.sqrt();
    //     GfxVec(Vector2d { x: (self.x as f32) / mag.0, y: (self.y as f32) / mag.0 })
    // }


#[derive(Copy, Clone, PartialEq, Debug)]
pub struct GfxVec(Point);

impl From<Point> for GfxVec {
    fn from(value: Point) -> Self {
        Self(value)
    }
}
impl Mul<f32> for GfxVec {
    type Output = GfxVec;
    
    fn mul(self, rhs: f32) -> Self::Output {
        Self::new((self.x as f32 * rhs) as i32, (self.y as f32 * rhs) as i32)
    }    
}

impl defmt::Format for GfxVec {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{{ x: {}, y: {} }}", self.0.x, self.0.y)
    }
}

impl GfxVec {
    pub fn new(x: i32, y:i32) -> Self {
        GfxVec(Point { x: x, y: y })
    }

    fn magnitude(&self) -> i32 {
        // self.iter()
        //     .map(|n| {
        //         let n = n.into();
        //         n * n
        //     })
        //     .sum::<f32>()
        //     .sqrt()
        (self.x * self.x) + (self.y * self.y).isqrt()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        let mut norm = self.clone();
        norm.x = self.x / mag;
        norm.y = self.y / mag;
        norm
    }

    pub fn rotate90(self) -> Self {
        Self(Point { x: -self.y, y: self.x })
    }

    pub fn cross_product(&self, other: &Self) -> i32 {
        (self.x * other.y) - (self.y * other.x)
    }
}

impl core::ops::DerefMut for GfxVec {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for GfxVec {
    type Target = Point;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

// impl From<GfxVec> for Point {
//     fn from(value: GfxVec) -> Self {
//         Point { x: value.x.round() as i32, y: value.y.round() as i32 }
//     }
// }

// impl From<GfxVec> for Vector2d<f32> {
//     fn from(value: GfxVec) -> Self {
//         value.0
//     }
// }

// impl From<Vector2d<f32>> for GfxVec {
//     fn from(value: Vector2d<f32>) -> Self {
//         Self(value)
//     }
// }

// impl From<Point> for GfxVec {
//     fn from(value: Point) -> Self {
//         Self(Vector2d { x: value.x as f32, y: value.y as f32 })
//     }
// }





pub enum LineRelation {
    Parallel,
    Colinear,
    Intersect(Point),
    None,
}

pub fn intersection(l1: &Line, l2: &Line) -> LineRelation {
    let a = l1.start;
    let b = l1.end;
    let c = l2.start;
    let d = l2.end;

    let t_num = (c - a).cross_product(d - c);
    let t_den = (b - a).cross_product(d - c);

    let u_num = (a - c).cross_product(b - a);
    let u_den = (c - d).cross_product(b - a);

    let t = t_num as f32 / t_den as f32;
    let u = u_num as f32 / u_den as f32;

    if t_num == 0 && t_den == 0 && u_num == 0 && u_den == 0 {
        LineRelation::Colinear
    } else if t_den == 0 && u_den == 0 {
        LineRelation::Parallel
    } else if ( 0.0 <= t && t <= 1.0 ) && ( 0.0 <= u && u <= 1.0 ) {
        LineRelation::Intersect(a + GfxVec(GfxVec::from(b - a).0 * t as i32).0)
    } else {
        LineRelation::None
    }
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


#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Sprite<'a, C: PixelColor, T: VectorComponent>
{
    name: &'a str,
    style: PrimitiveStyle<C>,
    line_style: PrimitiveStyle<C>,
    shape: SpritePrimitive<'a>,
    velocity: SpriteVector<T>,
}

impl<'a, C, T> SurfaceNormal for Sprite<'a, C, T> 
where 
    i32: From<T>, 
    C: PixelColor, 
    T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
    {
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        let point: Point = point.into();
        self.center() - point
    }

    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        let point: Point = point.into();
        self.center().distance(point) as f32
    }
}

impl From<GfxVec> for Point {
    fn from(value: GfxVec) -> Self {
        value.0
    }
}
impl<'a, C, T> Drawable for Sprite<'a, C, T>  
    where 
        i32: From<T>, 
        C: PixelColor, 
        T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        self.shape.draw_styled(&self.style, target)?;
        let center: Point = self.center();
        let delta: Point = self.velocity().into();
        Line::with_delta(center, delta).draw_styled(&self.line_style, target)?;
        Ok(())
    }
}

impl<'a, C, T> Sprite<'a, C, T> 
    where 
        i32: From<T>, 
        C: PixelColor, 
        T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    pub fn new(name: &'a str, style: PrimitiveStyle<C>, line_style: PrimitiveStyle<C>, shape: impl Into<SpritePrimitive<'a>>) -> Self {
        Self {name, style, line_style, shape: shape.into(), velocity: Point::zero().into() }
    }
}

impl<'a, C, T> Sprite<'a, C, T> 
    where 
        i32: From<T>, 
        C: PixelColor, 
        T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    pub fn name(&self) -> &str {
        self.name
    }

    #[inline]
    pub fn style(&self) -> PrimitiveStyle<C> {
        self.style
    }
    
    #[inline]
    pub fn style_mut(&mut self) -> &mut PrimitiveStyle<C> {
        &mut self.style
    }
    
    #[inline]
    pub fn set_style(&mut self, style: PrimitiveStyle<C>) {
        self.style = style;
    }
    
    pub fn set_line_style(&mut self, line_style: PrimitiveStyle<C>) {
        self.line_style = line_style;
    }
    
    pub fn line_style_mut(&mut self) -> &mut PrimitiveStyle<C> {
        &mut self.line_style
    }
    
    pub fn line_style(&self) -> PrimitiveStyle<C> {
        self.line_style
    }
    
    pub fn velocity(&self) -> SpriteVector<T> {
        self.velocity
    }
    
    pub fn velocity_mut(&mut self) -> &mut SpriteVector<T> {
        &mut self.velocity
    }
    
    pub fn set_velocity(&mut self, velocity: SpriteVector<T>) {
        self.velocity = velocity;
    }

    pub fn center(&self) -> Point {
        self.shape.bounding_box().center()
    }

    fn half_width(&self) -> u32 {
        self.shape.bounding_box().size.width / 2
    }

    pub fn simple_distance(&self, other: &Self) -> i32 {
        let mut simple_distance = self.center().distance(other.center());
        simple_distance -= (self.half_width()+other.half_width()) as i32;
        simple_distance
    }

    pub fn set_direction_from_angle(&mut self, direction: Angle) {
        let radius = (self.half_width() as f32) + 1.0;
        let angle = direction.to_radians();
        let (mut x, mut y) = angle.sin_cos();
        x *= radius;
        y *= radius;
        // self.set_velocity(GfxVec((x, y).into()));
        self.set_velocity(Point::new(x as i32, y as i32).into());
    }

    // incident vector is the vector before collision
    fn incident_vector(&self) -> SpriteVector<T> {
        self.velocity()
    }
    

    // compute the reflection vector
    fn reflected_vector(&self, surface_normal: SpriteVector<T>) -> SpriteVector<T> {
        // let incoming_velocity = Point::from(self.incident_vector());
        // let incoming_velocity = Vector2d { x: incoming_velocity.x, y: incoming_velocity.y };
        // let collision_normal = Point::from(surface_normal);
        // let collision_normal = Vector2d { x: collision_normal.x, y: collision_normal.y };
        // let v = calculate_reflection_vector::<i32>(&incoming_velocity, &collision_normal, 1.0);

        // let v = calculate_reflection_vector(&self.incident_vector().0, &surface_normal.0, 1.0);
        let v = calculate_reflection_vector_p(&self.incident_vector().into(), &surface_normal.into(), 1.0);

        // let (speed1, speed2) = (self.incident_vector().magnitude(), v.magnitude());
        // if (speed1 - speed2).abs() > 0.1 {
        //     info!("{} speed change: old = {}, new = {}", self.name(), speed1, speed2);
        // }
        v.into()
    }

    pub fn distance_between(&self, other: &Self) -> i32 {
        self.shape.distance_between(&other.shape)
    }

    pub fn about_to_collide(&self, other: &Self) -> bool {
        let next_self = self.shape.translate(self.velocity.into());
        let next_other = other.shape.translate(other.velocity.into());
        next_self.distance_between(&next_other) < 1
    }
    
    pub fn is_collision(sprite1: &Sprite<'a, C, T>, sprite2: &Sprite<'a, C, T>) -> bool {
        let distance = sprite1.distance_between(sprite2);
        distance < 1
        // let r1 = sprite1.shape.bounding_box().offset(1);
        // let r2 = sprite2.shape.bounding_box().offset(1);
        // let i1 = r1.intersection(&r2);
        // !i1.is_zero_sized()
        // sprite1.box_distance(sprite2) < 1
    }

    pub fn box_distance(&self, other: &Self) -> i32 {
        let distance_line = Line::new(self.center(), other.center());
        let mut distance = distance_line.start.distance_squared(distance_line.end);
        let size1 = self.shape.bounding_box().size;
        let size2 = other.shape.bounding_box().size;

        distance -= (size1.area() + size2.area()) as i32;
        distance
    }

    fn wall_bounce(&self, boundary: & impl Dimensions) -> SpriteVector<T> {
        let mut delta = self.velocity;
        let center = self.center();
        let width = self.half_width() as i32;

        // check against bounds
        let r = boundary.bounding_box();
        let top_left = r.top_left;
        let bottom_right = Point::new(top_left.x + r.size.width as i32, top_left.y + r.size.height as i32);

        if (center.x - top_left.x) <= width || (bottom_right.x - center.x) <= width {
            // bounce off x axis
            delta.x = -delta.x;
        }
        if (center.y - top_left.y) <= width || (bottom_right.y - center.y) <= width {
            // bounce off y axis
            delta.y = -delta.y;
        }
        delta
    }

    pub fn update_velocity(&mut self, other: &Self) -> bool{
        let mut bounced = false;
        if Self::is_collision(self, other) && self.is_moving() {
            let velocity = self.reflected_vector(other.shape.surface_normal(self.center()).into());
            self.set_velocity(velocity);
            bounced = true;
        }
        bounced
    }

    #[inline]
    pub fn is_moving(&self) -> bool {
        // self.velocity.magnitude() > 0.0
        Point::from(self.velocity) != Point::zero()
    }

    /// move object by applying the direction vector, while checking against containing rectangle
    pub fn move_object_bounded(&mut self, boundary: & impl Dimensions) {
        let mut delta = self.velocity();
        if self.is_moving() {
            let center = self.center();
            let width = self.half_width() as i32;
    
            // check against bounds
            let r = boundary.bounding_box();
            let top_left = r.top_left;
            let bottom_right = Point::new(top_left.x + r.size.width as i32, top_left.y + r.size.height as i32);
    
            if (center.x - top_left.x) <= width || (bottom_right.x - center.x) <= width {
                // bounce off x axis
                delta.x = -delta.x;
            }
            if (center.y - top_left.y) <= width || (bottom_right.y - center.y) <= width {
                // bounce off y axis
                delta.y = -delta.y;
            }
    
            self.set_velocity(delta);
            self.shape.translate_mut(delta.into());
        }
    }

    /// apply the velocity to the position of the shape (ignoring any collisions)
    pub fn move_object(&mut self) {
        let delta = self.velocity();
        if self.is_moving() {
            self.shape.translate_mut(delta.into());
        }
    }

}


// #[derive(Debug, Eq, PartialEq, Ord, PartialOrd)]
#[derive(Clone, Debug, Default, defmt::Format)]
pub struct SpriteContainer<'a, C: PixelColor, T> 
    where 
        i32: From<T>, 
        C: PixelColor, 
        T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    boundary: Rectangle,
    sprites: Vec<Sprite<'a, C, T>>,
    _phantom: PhantomData<C>,
}

impl<'a, C: PixelColor, T> SpriteContainer<'a, C, T> 
where 
    i32: From<T>, 
    C: PixelColor, 
    T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    pub fn new(boundary: Rectangle) -> Self {
        let sprites = Vec::new();
        Self { boundary, sprites, _phantom: PhantomData }
    }

    pub fn add_sprite(&mut self, sprite: Sprite<'a, C, T>) {
        self.sprites.push(sprite);
    }

    pub fn update_positions(&mut self) {
        let max_idx = self.sprites.len();
        for i in 0..max_idx {
            let (left, right) = self.sprites.split_at_mut(i);
            if let Some((current_sprite, right)) = right.split_first_mut() {
                if current_sprite.is_moving() {
                    let now = Instant::now();
                    // Iterate over the remaining parts to find other elements matching the predicate
                    for other_sprite in left.iter().chain(right.iter()) {
                        if current_sprite.about_to_collide(other_sprite) {
                        // if current_sprite.box_distance(other_sprite) < 1 {
                            if current_sprite.update_velocity(other_sprite) {
                                // info!("{} bounced off of {}", current_sprite.name(), other_sprite.name());
                            }
                        }
                    }
                    info!("calculation took {}", now.elapsed());
                }
            }
        }
        for current_sprite in self.sprites.iter_mut() {
            if current_sprite.is_moving() {
                current_sprite.move_object_bounded(&self.boundary);
            }
        }
    }
}

impl<'a, C: PixelColor, T> Drawable for SpriteContainer<'a, C, T> 
    where 
        i32: From<T>, 
        C: PixelColor, 
        T: VectorComponent + core::convert::From<i32> + Neg<Output = T> 
{
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        for sprite in self.sprites.iter() {
            sprite.draw(target)?;
        }
        Ok(())
    }
}

