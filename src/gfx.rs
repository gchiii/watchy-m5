// use core::{marker::PhantomData, ops::Deref};

use core::marker::PhantomData;

use allocator_api2::{boxed::Box, vec::Vec};
use defmt::info;
use embedded_graphics::{
    geometry::{AnchorPoint, AnchorX, AnchorY}, 
    pixelcolor::{BinaryColor, Rgb565}, 
    primitives::{self, Circle, Line, Polyline, PrimitiveStyle, Rectangle, Styled, StyledDrawable, Triangle}, 
    Drawable
};
use embedded_graphics::prelude::*;
use micromath::{vector::{Vector, Vector2d}, F32};
use thiserror_no_std::Error;

/// Converts a polar coordinate (angle/distance) into an (X, Y) coordinate centered around the
/// center of the circle.
///
/// The angle is relative to the 12 o'clock position and the radius is relative to the edge of the
/// clock face.
fn polar(circle: &Circle, angle: &Angle, radius_delta: i32) -> Point {
    let radius = circle.diameter as f32 / 2.0 + radius_delta as f32;
    let angle = F32::from(angle.to_radians());

    circle.center()
        + Point::new(
            (angle.sin().0 * radius) as i32,
            -(angle.cos().0 * radius) as i32,
        )
}


#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default, defmt::Format)]
pub enum CollisionSide {
    #[default]
    None,
    Left(Point),
    Top(Point),
    Right(Point),
    Bottom(Point),
}

pub trait Perimeter: Primitive {
    fn perimeter(&self) -> Vec<Point>;
}

impl Perimeter for Circle {
    fn perimeter(&self) -> Vec<Point> {
        let style = PrimitiveStyle::<BinaryColor>::with_stroke(BinaryColor::On, 1);
        let styled= self.into_styled(style);
        styled.pixels().map(|p| p.0).collect()
    }
}

impl Perimeter for Rectangle {
    fn perimeter(&self) -> Vec<Point> {
        let style = PrimitiveStyle::<BinaryColor>::with_stroke(BinaryColor::On, 1);
        let styled= self.into_styled(style);
        styled.pixels().map(|p| p.0).collect()
    }
}


#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default, defmt::Format)]
pub struct Edges {
    pub top: Line,
    pub bottom: Line,
    pub left: Line,
    pub right: Line,
}

impl Edges {
    pub fn new(shape: &impl Dimensions) -> Self {
        let r = shape.bounding_box();    
        Self { 
            top: Line::new(r.top_left, r.anchor_point(AnchorPoint::TopRight)),
            bottom: Line::new(r.anchor_point(AnchorPoint::BottomLeft), r.anchor_point(AnchorPoint::BottomRight)),
            left: Line::new(r.top_left, r.anchor_point(AnchorPoint::BottomLeft)),
            right: Line::new(r.anchor_point(AnchorPoint::TopRight), r.anchor_point(AnchorPoint::BottomRight)) 
        }
    }

    pub fn intersect(first_line: Line, other_line: Line) -> Option<Point> {        
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
}



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

pub trait PointExt {
    /// dot product of two points, self and other
    fn dot_product(&self, other: Point) -> i32;

    /// distance squared between two points
    fn distance_squared(&self, other: Point) -> i32;

    /// distance betwen points
    fn distance(&self, other: Point) -> i32;

    fn cross_product(&self, point: Point) -> i32;
}

impl PointExt for Point {
    fn cross_product(&self, point: Point) -> i32 {
        (self.x * point.y) - (self.y * point.x)
    }
    fn dot_product(&self, other: Point) -> i32 {
        (self.x * other.x) + (self.y * other.y)
    }

    fn distance_squared(&self, other: Point) -> i32 {
        (self.x - other.x).pow(2) + (self.y - other.y).pow(2)
    }

    fn distance(&self, other: Point) -> i32 {
        self.distance_squared(other).isqrt()
    }
}

pub trait SurfaceNormal {
    /// compute the normalized surface normal between self and point
    fn surface_normal(&self, point: Point) -> GfxVec;

    /// distance from nearest surface to point
    fn distance(&self, point: Point) -> i32;
}

impl ClosestEdge for Rectangle {
    fn closest_edge(&self, point: Point) -> Line {
        let vertices = [
            self.anchor_point(AnchorPoint::TopLeft),
            self.anchor_point(AnchorPoint::TopRight),
            self.anchor_point(AnchorPoint::BottomRight),
            self.anchor_point(AnchorPoint::BottomLeft)
        ];
        let mut closest1 = vertices[0];
        let mut closest2 = vertices[1];

        if closest1.distance_squared(point) > closest2.distance_squared(point) {
            core::mem::swap(&mut closest1, &mut closest2);
        }

        for &p in vertices.iter().skip(2) {
            let dist = p.distance_squared(point);
            if dist < closest1.distance_squared(point) {
                closest2 = closest1;
                closest1 = p;
            } else if dist < closest2.distance_squared(point) {
                closest2 = p;
            }
        }
        Line::new(closest1, closest2)
    }
}

impl SurfaceNormal for Rectangle {
    fn surface_normal(&self, point: Point) -> GfxVec {
        self.closest_edge(point).surface_normal(point)
    }
    
    fn distance(&self, point: Point) -> i32 {
        self.closest_edge(point).distance(point)
    }
}

impl SurfaceNormal for Line {

    fn surface_normal(&self, point: Point) -> GfxVec {
        let closest_point = {
            let line_vec = self.delta();
            let point_vec = point - self.start;
            let line_len_sq = self.start.distance_squared(self.end) as f32;

            if line_len_sq == 0.0 { 
                self.start
            } else {              
                let t = point_vec.dot_product(line_vec) as f32 / line_len_sq;
                let t = t.max(0.0).min(1.0);
    
                Point {
                    x: (self.start.x as f32 + t * line_vec.x as f32) as i32,
                    y: (self.start.y as f32 + t * line_vec.y as f32) as i32,
                }
            }
        };
        GfxVec::from(closest_point - point).normalize()
    }
    
    fn distance(&self, point: Point) -> i32 {
        let closest_point = {
            let line_vec = self.delta();
            let point_vec = point - self.start;
            let line_len_sq = self.start.distance_squared(self.end) as f32;

            if line_len_sq == 0.0 { 
                self.start
            } else {              
                let t = point_vec.dot_product(line_vec) as f32 / line_len_sq;
                let t = t.max(0.0).min(1.0);
    
                Point {
                    x: (self.start.x as f32 + t * line_vec.x as f32) as i32,
                    y: (self.start.y as f32 + t * line_vec.y as f32) as i32,
                }
            }
        };
        closest_point.distance(point)
    }    
}


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
        LineRelation::Intersect(a + Point::from(GfxVec(GfxVec::from(b - a).0 * t)))
    } else {
        LineRelation::None
    }
}

pub trait ClosestEdge {
    fn closest_edge(&self, point: Point) -> Line;
}

impl ClosestEdge for Triangle {
    fn closest_edge(&self, point: Point) -> Line {

        let mut closest1 = self.vertices[0];
        let mut closest2 = self.vertices[1];

        if closest1.distance_squared(point) > closest2.distance_squared(point) {
            core::mem::swap(&mut closest1, &mut closest2);
        }

        for &p in self.vertices.iter().skip(2) {
            let dist = p.distance_squared(point);
            if dist < closest1.distance_squared(point) {
                closest2 = closest1;
                closest1 = p;
            } else if dist < closest2.distance_squared(point) {
                closest2 = p;
            }
        }
        Line::new(closest1, closest2)
    }
}

impl SurfaceNormal for Triangle {
    fn surface_normal(&self, point: Point) -> GfxVec {
        self.closest_edge(point).surface_normal(point)
    }
    
    fn distance(&self, point: Point) -> i32 {
        self.closest_edge(point).distance(point)
    }
}

impl<'a> ClosestEdge for Polyline<'a> {
    fn closest_edge(&self, point: Point) -> Line {
        let mut closest1 = self.vertices[0];
        let mut closest2 = self.vertices[1];

        if closest1.distance_squared(point) > closest2.distance_squared(point) {
            core::mem::swap(&mut closest1, &mut closest2);
        }

        for &p in self.vertices.iter().skip(2) {
            let dist = p.distance_squared(point);
            if dist < closest1.distance_squared(point) {
                closest2 = closest1;
                closest1 = p;
            } else if dist < closest2.distance_squared(point) {
                closest2 = p;
            }
        }
        Line::new(closest1, closest2)
    }
}

impl<'a> SurfaceNormal for Polyline<'a> {
    fn surface_normal(&self, point: Point) -> GfxVec {
        self.closest_edge(point).surface_normal(point)
    }
    
    fn distance(&self, point: Point) -> i32 {
        self.closest_edge(point).distance(point)
    }
}

impl SurfaceNormal for Circle {
    // compute the normalized surface normal between self and point
    fn surface_normal(&self, point: Point) -> GfxVec {
        GfxVec::from(self.center() - point).normalize()
    }
    
    fn distance(&self, point: Point) -> i32 {
        self.center().distance(point) - (self.diameter / 2) as i32
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
    // fn move_by(&mut self, by: Point);
}

// impl Movable for Circle {
//     fn move_by(&mut self, by: Point) {
//         self.top_left += by;
//     }
// }

// impl Movable for Rectangle {
//     fn move_by(&mut self, by: Point) {
//         self.top_left += by;
//     }
// }


#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum SpritePrimitive<'a> 
// where 
//     C: PixelColor,
{
    Line(primitives::Line),
    Circle(primitives::Circle),
    Rectangle(primitives::Rectangle),
    Polyline(primitives::Polyline<'a>),
}

impl<'a> Dimensions for SpritePrimitive<'a> {
    fn bounding_box(&self) -> Rectangle {
        match self {
            SpritePrimitive::Line(line) => line.bounding_box(),
            SpritePrimitive::Circle(circle) => circle.bounding_box(),
            SpritePrimitive::Rectangle(rectangle) => rectangle.bounding_box(),
            SpritePrimitive::Polyline(polyline) => polyline.bounding_box(),
        }
    }
}

impl<'a> SurfaceNormal for SpritePrimitive<'a> {
    fn surface_normal(&self, point: Point) -> GfxVec {
        match self {
            SpritePrimitive::Line(line) => line.surface_normal(point),
            SpritePrimitive::Circle(circle) => circle.surface_normal(point),
            SpritePrimitive::Rectangle(rectangle) => rectangle.surface_normal(point),
            SpritePrimitive::Polyline(polyline) => polyline.surface_normal(point),
        }
    }

    fn distance(&self, point: Point) -> i32 {
        match self {
            SpritePrimitive::Line(line) => line.distance(point),
            SpritePrimitive::Circle(circle) => circle.distance(point),
            SpritePrimitive::Rectangle(rectangle) => rectangle.distance(point),
            SpritePrimitive::Polyline(polyline) => polyline.distance(point),
        }
    }
}

impl<'a> From<Line> for SpritePrimitive<'a> {
    fn from(value: Line) -> Self {
        Self::Line(value)
    }
}
impl<'a> From<Circle> for SpritePrimitive<'a> {
    fn from(value: Circle) -> Self {
        Self::Circle(value)
    }
}
impl<'a> From<Rectangle> for SpritePrimitive<'a> {
    fn from(value: Rectangle) -> Self {
        Self::Rectangle(value)
    }
}
impl<'a> From<Polyline<'a>> for SpritePrimitive<'a> {
    fn from(value: Polyline<'a>) -> Self {
        Self::Polyline(value)
    }
}


impl<'a> Transform for SpritePrimitive<'a> {
    fn translate(&self, by: Point) -> Self {
        match self {
            SpritePrimitive::Line(line) => line.translate(by).into(),
            SpritePrimitive::Circle(circle) => circle.translate(by).into(),
            SpritePrimitive::Rectangle(rectangle) => rectangle.translate(by).into(),
            SpritePrimitive::Polyline(polyline) => polyline.translate(by).into(),
        }
    }

    fn translate_mut(&mut self, by: Point) -> &mut Self {
        *self = self.translate(by);
        self
    }
}

#[derive(Clone, Copy, Hash, Debug, defmt::Format)]
pub struct Sprite<'a, C: PixelColor>
{
    style: PrimitiveStyle<C>,
    line_style: PrimitiveStyle<C>,
    shape: SpritePrimitive<'a>,
    velocity: Point,
}

impl<'a, C: PixelColor> Drawable for Sprite<'a, C> {
    type Color = C;

    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self.shape {
            SpritePrimitive::Line(line) => line.into_styled(self.style()).draw(target)?,
            SpritePrimitive::Circle(circle) => circle.into_styled(self.style()).draw(target)?,
            SpritePrimitive::Rectangle(rectangle) => rectangle.into_styled(self.style()).draw(target)?,
            SpritePrimitive::Polyline(polyline) => polyline.into_styled(self.style()).draw(target)?,
        };
        Line::with_delta(self.center(), self.velocity()).draw_styled(&self.line_style, target)?;
        Ok(())
    }
}

impl<'a, C: PixelColor> Sprite<'a, C> {
    pub fn new(style: PrimitiveStyle<C>, line_style: PrimitiveStyle<C>, shape: impl Into<SpritePrimitive<'a>>) -> Self {
        Self { style, line_style, shape: shape.into(), velocity: Point::zero() }
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
    
    #[inline]
    pub fn set_line_style(&mut self, line_style: PrimitiveStyle<C>) {
        self.line_style = line_style;
    }
    
    #[inline]
    pub fn line_style_mut(&mut self) -> &mut PrimitiveStyle<C> {
        &mut self.line_style
    }
    
    #[inline]
    pub fn line_style(&self) -> PrimitiveStyle<C> {
        self.line_style
    }
    
    #[inline]
    pub fn velocity(&self) -> Point {
        self.velocity
    }
    
    #[inline]
    pub fn velocity_mut(&mut self) -> &mut Point {
        &mut self.velocity
    }
    
    #[inline]
    pub fn set_velocity(&mut self, velocity: Point) {
        self.velocity = velocity;
    }

    #[inline]
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
        let radius = F32(self.half_width() as f32) + 1.0;
        let angle = F32::from(direction.to_radians());
        let delta = Point::new(
            (angle.sin().0 * radius).0 as i32,
            -(angle.cos().0 * radius).0 as i32,
        );
        self.set_velocity(delta);
    }

    // incident vector is the vector before collision
    fn incident_vector(&self) -> GfxVec {
        GfxVec::from(self.velocity())
    }

    // compute the reflection vector
    fn reflected_vector(&self, surface_normal: GfxVec) -> GfxVec {
        let incident_vector = self.incident_vector();
        let twice_dot_product = 2.0 * incident_vector.dot(surface_normal.into());
        let x = incident_vector.x - (twice_dot_product * surface_normal.x);
        let y = incident_vector.y - (twice_dot_product * surface_normal.y);
        let v = GfxVec(Vector2d { x, y });
        info!("new speed: {}", v.0.magnitude());
        v
    }

    fn collission_update(sprite1: &mut Sprite<'a, C>, sprite2: &mut Sprite<'a, C>) -> bool {
        let collision_distance = 1;
        if sprite1.shape != sprite2.shape && sprite1.simple_distance(sprite2) <= collision_distance {
            if sprite1.is_moving() {
                let my_reflection: Point = sprite1.reflected_vector(sprite2.shape.surface_normal(sprite1.center())).into();
                info!("Collision Sprite1: vel1: {} vel2: {}", sprite1.velocity, my_reflection);
                sprite1.set_velocity(my_reflection);
                sprite1.move_object();
            }
            if sprite2.is_moving() {
                let other_reflection: Point = sprite2.reflected_vector(sprite1.shape.surface_normal(sprite2.center())).into();
                info!("Collision Sprite2: vel1: {} vel2: {}", sprite2.velocity, other_reflection);
                sprite2.set_velocity(other_reflection);
                sprite2.move_object();
            }
            true
        } else {
            false
        }
    }
    pub fn collission_update_bounded(sprite1: &mut Sprite<'a, C>, sprite2: &mut Sprite<'a, C>, boundary: & impl Dimensions) -> bool {
        let collision_distance = 1;
        if sprite1.shape != sprite2.shape && sprite1.simple_distance(sprite2) <= collision_distance {
            if sprite1.is_moving() {
                let my_reflection: Point = sprite1.reflected_vector(sprite2.shape.surface_normal(sprite1.center())).into();
                info!("Collision Sprite1: vel1: {} vel2: {}", sprite1.velocity, my_reflection);
                sprite1.set_velocity(my_reflection);
                sprite1.move_object_bounded(boundary);
            }
            if sprite2.is_moving() {
                let other_reflection: Point = sprite2.reflected_vector(sprite1.shape.surface_normal(sprite2.center())).into();
                info!("Collision Sprite2: vel1: {} vel2: {}", sprite2.velocity, other_reflection);
                sprite2.set_velocity(other_reflection);
                sprite2.move_object_bounded(boundary);
            }
            true
        } else {
            false
        }
    }

    #[inline]
    pub fn is_moving(&self) -> bool {
        self.velocity != Point::zero()
    }
    #[inline]
    pub fn is_stationary(&self) -> bool {
        self.velocity == Point::zero()
    }
    /// move object by applying the direction vector, while checking against containing rectangle
    pub fn move_object_bounded(&mut self, boundary: & impl Dimensions) {
        let mut delta = self.velocity();
        if !(delta.x == 0 && delta.y == 0 ) {
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
            self.shape.translate_mut(delta);
        }
    }

    /// apply the velocity to the position of the shape (ignoring any collisions)
    pub fn move_object(&mut self) {
        let delta = self.velocity();
        if !(delta.x == 0 && delta.y == 0 ) {
            self.shape.translate_mut(delta);
        }
    }

}

// #[derive(Debug, Eq, PartialEq, Ord, PartialOrd)]
#[derive(Clone, Hash, Debug, Default, defmt::Format)]
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
        let sprites = self.sprites.as_mut_slice();
        for i in 0..max_idx {
            let (_a, b) = sprites.split_at_mut(i);
            if i + 1 < max_idx {
                let (b_first, b_second) = b.split_at_mut(1);
                let first = &mut b_first[0];
                let second = &mut b_second[0];                    
                if Sprite::collission_update(first, second) {
                    // first.move_object_bounded(&self.boundary);
                }
            } 
            if i == max_idx {
                let last = &mut b[0];
                let first = &mut _a[0];
                if Sprite::collission_update(last, first) {
                    // last.move_object_bounded(&self.boundary);
                }
            }
            b[0].move_object_bounded(&self.boundary);
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


#[derive(Copy, Clone, PartialEq, Debug)]
pub struct LinearEquation {
    /// Normal vector, perpendicular to the line.
    ///
    normal_vector: Vector2d<f32>,
    
    /// Distance from the origin.
    /// 
    origin_distance: f32,
}

impl LinearEquation {
    /// Creates a new linear equation from a line.
    pub fn from_line(line: &Line) -> Self {
        let normal_vector = GfxVec::from(line.delta()).rotate90().normalize();
        let origin_distance = GfxVec::from(line.start).dot(normal_vector.into());

        Self {
            normal_vector: normal_vector.into(),
            origin_distance,
        }
    }

    /// Returns the distance between the line and a point.
    ///
    /// The scaling of the returned value depends on the length of the normal vector.
    /// Positive values will be returned for points on the left side of the line and negative
    /// values for points on the right.
    pub fn distance(&self, point: Point) -> f32 {
        GfxVec::from(point).dot(self.normal_vector) - self.origin_distance
    }

}

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct GfxVec(Vector2d<f32>);

impl defmt::Format for GfxVec {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{{ x: {}, y: {} }}", self.0.x, self.0.y)
    }
}

impl GfxVec {
    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        let normal_vector = Vector2d { x: self.x / mag, y: self.y / mag };
        normal_vector.into()
    }

    pub fn rotate90(self) -> Self {
        Self(Vector2d { x: -self.y, y: self.x })
    }

    pub fn cross_product(&self, other: &Self) -> f32 {
        (self.x * other.y) - (self.y*other.x)
    }
}

impl core::ops::DerefMut for GfxVec {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for GfxVec {
    type Target = Vector2d<f32>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl From<GfxVec> for Point {
    fn from(value: GfxVec) -> Self {
        Point { x: value.x as i32, y: value.y as i32 }
    }
}

impl From<GfxVec> for Vector2d<f32> {
    fn from(value: GfxVec) -> Self {
        value.0
    }
}

impl From<Vector2d<f32>> for GfxVec {
    fn from(value: Vector2d<f32>) -> Self {
        Self(value)
    }
}

impl From<Point> for GfxVec {
    fn from(value: Point) -> Self {
        Self(Vector2d { x: value.x as f32, y: value.y as f32 })
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, Default, defmt::Format)]
struct GfxPoint(Point);

impl GfxPoint {
    pub fn normalize(self) -> GfxVec {
        let x = self.x * self.x;
        let y = self.y * self.y;
        let sum = F32((x + y) as f32);
        let mag = sum.sqrt();
        GfxVec(Vector2d { x: (self.x as f32) / mag.0, y: (self.y as f32) / mag.0 })
    }

}

impl core::ops::DerefMut for GfxPoint {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for GfxPoint {
    type Target = Point;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl From<Point> for GfxPoint {
    fn from(value: Point) -> Self {
        Self(value)
    }
}

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


