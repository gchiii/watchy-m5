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

use crate::sprites::{geometry::{Area, PointExt, SurfaceNormal}, sprites::SpritePrimitive};


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
pub struct Sprite<'a, C: PixelColor>
{
    name: &'a str,
    style: PrimitiveStyle<C>,
    line_style: PrimitiveStyle<C>,
    shape: SpritePrimitive<'a>,
    velocity: GfxVec,
}

impl<'a, C: PixelColor> SurfaceNormal for Sprite<'a, C> {
    fn surface_normal(&self, point: impl Into<GfxVec> + Into<Point> + Copy) -> GfxVec {
        let point: GfxVec = point.into();
        let sn = GfxVec(GfxVec::from(self.center()).0 - point.0);
        sn.normalize()
    }

    fn distance(&self, point: impl Into<GfxVec> + Into<Point> + Copy) -> f32 {
        let point: GfxVec = point.into();
        let center: GfxVec = self.center().into();
        center.distance(*point) as f32
    }
}

impl From<GfxVec> for Point {
    fn from(value: GfxVec) -> Self {
        value.0
    }
}
impl<'a, C: PixelColor> Drawable for Sprite<'a, C> {
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        self.shape.draw_styled(&self.style, target)?;
        Line::with_delta(self.center(), self.velocity().into()).draw_styled(&self.line_style, target)?;
        Ok(())
    }
}

impl<'a, C: PixelColor> Sprite<'a, C> {
    pub fn new(name: &'a str, style: PrimitiveStyle<C>, line_style: PrimitiveStyle<C>, shape: impl Into<SpritePrimitive<'a>>) -> Self {
        Self {name, style, line_style, shape: shape.into(), velocity: Point::zero().into() }
    }

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
    
    pub fn velocity(&self) -> GfxVec {
        self.velocity
    }
    
    pub fn velocity_mut(&mut self) -> &mut GfxVec {
        &mut self.velocity
    }
    
    pub fn set_velocity(&mut self, velocity: GfxVec) {
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
        self.set_velocity(GfxVec(Point::new(x as i32, y as i32)));
    }

    // incident vector is the vector before collision
    fn incident_vector(&self) -> GfxVec {
        self.velocity().into()
    }
    

    // compute the reflection vector
    fn reflected_vector(&self, surface_normal: GfxVec) -> GfxVec {
        // let incoming_velocity = Point::from(self.incident_vector());
        // let incoming_velocity = Vector2d { x: incoming_velocity.x, y: incoming_velocity.y };
        // let collision_normal = Point::from(surface_normal);
        // let collision_normal = Vector2d { x: collision_normal.x, y: collision_normal.y };
        // let v = calculate_reflection_vector::<i32>(&incoming_velocity, &collision_normal, 1.0);

        // let v = calculate_reflection_vector(&self.incident_vector().0, &surface_normal.0, 1.0);
        let v = calculate_reflection_vector_p(&self.incident_vector().0, &surface_normal.0, 1.0);

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
    
    pub fn is_collision(sprite1: &Sprite<'a, C>, sprite2: &Sprite<'a, C>) -> bool {
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

    fn wall_bounce(&self, boundary: & impl Dimensions) -> GfxVec {
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
            let velocity: GfxVec = self.reflected_vector(other.shape.surface_normal(self.center())).into();
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
        let x = f32::from(self.x) / mag;
        let y = f32::from(self.y) / mag;
        Vector2d { x: x.into(), y: y.into()}
    }
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

impl VecNormalize for Point {
    fn normalize(&self) -> Self {
        let mag = (self.x.pow(2) + self.y.pow(2)).isqrt();
        Self { x: self.x / mag, y: self.y / mag }
    }
}

fn calculate_reflection_vector_p(
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

// #[derive(Debug, Eq, PartialEq, Ord, PartialOrd)]
#[derive(Clone, Debug, Default, defmt::Format)]
pub struct SpriteContainer<'a, C: PixelColor> {
    boundary: Rectangle,
    sprites: Vec<Sprite<'a, C>>,
    _phantom: PhantomData<C>,
}

impl<'a, C: PixelColor> SpriteContainer<'a, C> {
    pub fn new(boundary: Rectangle) -> Self {
        let sprites = Vec::new();
        Self { boundary, sprites, _phantom: PhantomData }
    }

    pub fn add_sprite(&mut self, sprite: Sprite<'a, C>) {
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

impl<'a, C: PixelColor> Drawable for SpriteContainer<'a, C> {
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


// #[derive(Copy, Clone, PartialEq, Debug)]
// pub struct LinearEquation {
//     /// Normal vector, perpendicular to the line.
//     ///
//     normal_vector: Vector2d<f32>,
    
//     /// Distance from the origin.
//     /// 
//     origin_distance: f32,
// }

// impl LinearEquation {
//     /// Creates a new linear equation from a line.
//     pub fn from_line(line: &Line) -> Self {
//         let normal_vector = GfxVec::from(line.delta()).rotate90().normalize();
//         let origin_distance = GfxVec::from(line.start).dot(normal_vector.into());

//         Self {
//             normal_vector: normal_vector.into(),
//             origin_distance,
//         }
//     }

//     /// Returns the distance between the line and a point.
//     ///
//     /// The scaling of the returned value depends on the length of the normal vector.
//     /// Positive values will be returned for points on the left side of the line and negative
//     /// values for points on the right.
//     pub fn distance(&self, point: Point) -> f32 {
//         GfxVec::from(point).dot(self.normal_vector) - self.origin_distance
//     }

// }

fn normalize_vector2d_f32(a_vec: Vector2d<f32>) -> Vector2d<f32> {
    let mag = a_vec.magnitude();
    Vector2d { x: a_vec.x / mag, y: a_vec.y / mag }
}


pub trait HasVertices {
    fn get_vertices(&self) -> &[Point];

    fn get_edges(&self) -> Vec<Line> {
        let vertices = self.get_vertices();
        let mut edges: Vec<Line> = vertices.windows(2).map(|p| {
            Line::new(p[0], p[1])
        }).collect();
        if let (Some(p1), Some(p2)) = (vertices.first(), vertices.last()) {
            edges.push(Line::new(*p1, *p2));
        }
        edges
    }

    fn center(&self) -> Point;

    fn surface_point(&self, target: Point) -> Option<Point> {
        let edges = self.get_edges();
        let target_line = Line::new(self.center(), target);
        let mut found: Option<Point> = None;
        for l in edges {
            let p = match intersection(&l, &target_line) {
                LineRelation::Intersect(point) => point,
                _ => continue,
            };
            match found {
                Some(fp) => {
                    if fp.distance(target) > p.distance(target) {
                        found = Some(p);
                    } else {
                        continue;
                    }
                },
                None => found = Some(p),
            }
        }
        found
    }
}

// fn surface_point() {
//     intersection
// }


pub fn intersect_lines(first_line: Line, other_line: Line) -> Option<Point> {        
    let a1 = F32((first_line.end.y - first_line.start.y) as f32);
    let b1 = F32((first_line.start.x - first_line.end.x) as f32);
    let c1 = a1 * first_line.start.x as f32 + b1 * first_line.start.y as f32;

    let a2 = F32((other_line.end.y - other_line.start.y) as f32);
    let b2 = F32((other_line.start.x - other_line.end.x) as f32);
    let c2 = a2 * other_line.start.x as f32 + b2 * other_line.start.y as f32;

    let delta = a1 * b2 - a2 * b1;

    if delta == 0.0 {
        return None;
    }

    let x = (b2 * c1 - b1 * c2) / delta;
    let y = (a1 * c2 - a2 * c1) / delta;
    Some(Point {
        x: x.round().0 as i32,
        y: y.round().0 as i32,
    })
}


