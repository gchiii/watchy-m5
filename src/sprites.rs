use core::num;
use core::f32::consts::PI;

use defmt::info;
use embedded_graphics::{
    prelude::*, 
    primitives::{self, Circle, Line, Polyline, PrimitiveStyle, Rectangle, StyledDrawable, Triangle}
};
use nalgebra::SimdComplexField;
use no_std_compat2::f32;
use thiserror_no_std::Error;

extern crate micromath as mm;

extern crate nalgebra as na;
use na::{Vector2, Reflection2};
use na::Matrix;
use crate::gfx::{Area, GfxVec, PointExt};


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
pub struct SpriteVector(pub Vector2<f32>);

impl From<SpriteVector> for GfxVec {
    fn from(value: SpriteVector) -> Self {
        GfxVec::new(value.x, value.y)
    }
}
impl From<GfxVec> for SpriteVector {
    fn from(value: GfxVec) -> Self {
        SpriteVector::new(value.x, value.y)
    }
}

impl From<Vector2<f32>> for SpriteVector {
    fn from(value: Vector2<f32>) -> Self {
        SpriteVector(value)
    }
}

impl From<SpriteVector> for Point {
    fn from(value: SpriteVector) -> Self {
        Point { x: value.x as i32, y: value.y as i32 }
    }
}

impl SpriteVector {
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vector2::<f32>::new(x, y))
    }
}

impl defmt::Format for SpriteVector {
    fn format(&self, fmt: defmt::Formatter) {        
        defmt::write!(fmt, "{{ x: {}, y: {} }}", self.x, self.y)
    }
}

impl core::ops::DerefMut for SpriteVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl core::ops::Deref for SpriteVector {
    type Target = Vector2<f32>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
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




#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
pub enum SpritePrimitive<'a> 
{
    Line(primitives::Line),
    Circle(primitives::Circle),
    Rectangle(primitives::Rectangle),
    Polyline(primitives::Polyline<'a>),
    Triangle(primitives::Triangle),
}

impl<'a, C: PixelColor> StyledDrawable<PrimitiveStyle<C>> for SpritePrimitive<'a> {
    type Color = C;

    type Output = ();

    fn draw_styled<D>(&self, style: &PrimitiveStyle<C>, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color> {
        match self {
            SpritePrimitive::Line(line) => line.draw_styled(style, target),
            SpritePrimitive::Circle(circle) => circle.draw_styled(style, target),
            SpritePrimitive::Rectangle(rectangle) => rectangle.draw_styled(style, target),
            SpritePrimitive::Polyline(polyline) => polyline.draw_styled(style, target),
            SpritePrimitive::Triangle(triangle) => triangle.draw_styled(style, target),
        }
    }
}

impl<'a> Area for SpritePrimitive<'a> {
    fn area(&self) -> u32 {
        match self {
            SpritePrimitive::Line(_line) => 0,
            SpritePrimitive::Rectangle(rectangle) => rectangle.size.area(),
            SpritePrimitive::Circle(circle) => {
                        // Area = (π/4) × d^2, where 'd' is the diameter
                        let d_squared = circle.diameter * circle.diameter;
                        let area = (PI/4.0) * d_squared as f32;
                        area as u32
                    },
            SpritePrimitive::Polyline(polyline) => {
                        // just use the bounding box for now, we can revisit later
                        polyline.points().map(|p| info!("{}", p));
                        polyline.bounding_box().size.area()
                    },
            SpritePrimitive::Triangle(triangle) => todo!(),
        }
    }
}

impl<'a> SpritePrimitive<'a> {
    pub fn center(&self) -> Point {
        match self {
            SpritePrimitive::Line(line) => line.bounding_box().center(),
            SpritePrimitive::Circle(circle) => circle.center(),
            SpritePrimitive::Rectangle(rectangle) => rectangle.center(),
            SpritePrimitive::Polyline(polyline) => polyline.bounding_box().center(),
            SpritePrimitive::Triangle(triangle) => triangle.bounding_box().center(),
        }
    }

    pub fn distance_between(&self, other: &Self) -> i32 {
        let area_of_distance = self.center().distance_squared(other.center());
        let area_of_objects = self.area() + other.area();
        area_of_distance - area_of_objects as i32
    }

    pub fn box_distance(&self, other: &Self) -> i32 {
        let distance_line = Line::new(self.center(), other.center());
        let mut distance = distance_line.start.distance_squared(distance_line.end);
        distance -= (self.area() + other.area()) as i32;
        distance
    }

    pub fn calculate_reflection_vector(
        incoming_velocity: &SpriteVector,
        collision_normal: &SpriteVector,
        coefficient_of_restitution: f32,
    ) -> SpriteVector {
        // Ensure the normal is normalized (unit length)
        // let mag = collision_normal.magnitude()
        let normal = collision_normal.normalize();

        // Calculate the component of the incoming velocity perpendicular to the collision surface
        let perpendicular_velocity = incoming_velocity.dot(&normal) * normal;

        // Calculate the component of the incoming velocity parallel to the collision surface
        let parallel_velocity = incoming_velocity.0 - perpendicular_velocity;

        // The reflected perpendicular velocity is reversed and scaled by the COR
        let reflected_perpendicular_velocity = -perpendicular_velocity * coefficient_of_restitution;

        // The reflected velocity is the sum of the reflected perpendicular and parallel components
        (reflected_perpendicular_velocity + parallel_velocity).into()
    }    

}

impl<'a> Dimensions for SpritePrimitive<'a> {
    fn bounding_box(&self) -> Rectangle {
        match self {
            SpritePrimitive::Line(line) => line.bounding_box(),
            SpritePrimitive::Circle(circle) => circle.bounding_box(),
            SpritePrimitive::Rectangle(rectangle) => rectangle.bounding_box(),
            SpritePrimitive::Polyline(polyline) => polyline.bounding_box(),
            SpritePrimitive::Triangle(triangle) => triangle.bounding_box(),
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
impl<'a> From<Triangle> for SpritePrimitive<'a> {
    fn from(value: Triangle) -> Self {
        Self::Triangle(value)
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
            SpritePrimitive::Triangle(triangle) => triangle.translate(by).into(),
        }
    }

    fn translate_mut(&mut self, by: Point) -> &mut Self {
        *self = self.translate(by);
        self
    }
}

// #[derive(Clone, Copy, Debug, defmt::Format)]
// pub struct Sprite<'a, C: PixelColor>
// {
//     name: &'a str,
//     style: PrimitiveStyle<C>,
//     line_style: PrimitiveStyle<C>,
//     shape: SpritePrimitive<'a>,
//     velocity: SpriteVector,
// }


// impl<'a, C: PixelColor> Drawable for Sprite<'a, C> {
//     type Color = C;

//     type Output = ();

//     fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
//     where
//         D: DrawTarget<Color = Self::Color> {
//         match self.shape {
//             SpritePrimitive::Line(line) => line.into_styled(self.style()).draw(target)?,
//             SpritePrimitive::Circle(circle) => circle.into_styled(self.style()).draw(target)?,
//             SpritePrimitive::Rectangle(rectangle) => rectangle.into_styled(self.style()).draw(target)?,
//             SpritePrimitive::Polyline(polyline) => polyline.into_styled(self.style()).draw(target)?,
//         };
//         Line::with_delta(self.center(), self.velocity().into()).draw_styled(&self.line_style, target)?;
//         Ok(())
//     }
// }

// impl<'a, C: PixelColor> Sprite<'a, C> {
//     pub fn new(name: &'a str, style: PrimitiveStyle<C>, line_style: PrimitiveStyle<C>, shape: impl Into<SpritePrimitive<'a>>) -> Self {
//         Self {name, style, line_style, shape: shape.into(), velocity: SpriteVector::new(0.0, 0.0)  }
//     }

//     pub fn name(&self) -> &str {
//         self.name
//     }

//     #[inline]
//     pub fn style(&self) -> PrimitiveStyle<C> {
//         self.style
//     }
    
//     #[inline]
//     pub fn style_mut(&mut self) -> &mut PrimitiveStyle<C> {
//         &mut self.style
//     }
    
//     #[inline]
//     pub fn set_style(&mut self, style: PrimitiveStyle<C>) {
//         self.style = style;
//     }
    
//     #[inline]
//     pub fn set_line_style(&mut self, line_style: PrimitiveStyle<C>) {
//         self.line_style = line_style;
//     }
    
//     #[inline]
//     pub fn line_style_mut(&mut self) -> &mut PrimitiveStyle<C> {
//         &mut self.line_style
//     }
    
//     #[inline]
//     pub fn line_style(&self) -> PrimitiveStyle<C> {
//         self.line_style
//     }
    
//     #[inline]
//     pub fn velocity(&self) -> SpriteVector {
//         self.velocity
//     }
    
//     #[inline]
//     pub fn velocity_mut(&mut self) -> &mut SpriteVector {
//         &mut self.velocity
//     }
    
//     #[inline]
//     pub fn set_velocity(&mut self, velocity: SpriteVector) {
//         self.velocity = velocity;
//     }

//     #[inline]
//     pub fn center(&self) -> Point {
//         self.shape.bounding_box().center()
//     }

//     fn half_width(&self) -> u32 {
//         self.shape.bounding_box().size.width / 2
//     }

//     pub fn simple_distance(&self, other: &Self) -> f32 {
//         let center = SpritePoint::from(self.center());
//         let other_ceneter = SpritePoint::from(other.center());
//         let d = center.metric_distance(&other_ceneter);
//         d
//         // let mut simple_distance = self.center().distance(other.center());
//         // simple_distance -= (self.half_width()+other.half_width()) as i32;
//         // simple_distance
//     }

//     // pub fn set_direction_from_angle(&mut self, direction: Angle) {
//     //     let radius = (self.half_width() as f32) + 1.0;
//     //     let angle = direction.to_radians();
//     //     let (mut x, mut y) = angle.sin_cos();
//     //     x *= radius;
//     //     y *= radius;
//     //     self.set_velocity(GfxVec((x, y).into()));
//     // }

//     // // incident vector is the vector before collision
//     // fn incident_vector(&self) -> GfxVec {
//     //     self.velocity().into()
//     // }

//     // // compute the reflection vector
//     // fn reflected_vector(&self, surface_normal: GfxVec) -> GfxVec {
//     //     let incident_vector = self.incident_vector();
//     //     let twice_dot_product = 2.0 * incident_vector.dot(surface_normal.into());
//     //     let x = incident_vector.x - (twice_dot_product * surface_normal.x);
//     //     let y = incident_vector.y - (twice_dot_product * surface_normal.y);
//     //     let v = GfxVec(Vector2d { x, y });
//     //     let (speed1, speed2) = (incident_vector.magnitude(), v.magnitude());
//     //     if (speed1 - speed2).abs() > 0.1 {
//     //         info!("{} speed change: old = {}, new = {}", self.name(), speed1, speed2);
//     //     }
//     //     v
//     // }

//     // pub fn is_collision(sprite1: &Sprite<'a, C>, sprite2: &Sprite<'a, C>) -> bool {
//     //     // let r1 = sprite1.shape.bounding_box().offset(1);
//     //     // let r2 = sprite2.shape.bounding_box().offset(1);
//     //     // let i1 = r1.intersection(&r2);
//     //     // !i1.is_zero_sized()
//     //     sprite1.box_distance(sprite2) < 1
//     // }

//     // pub fn box_distance(&self, other: &Self) -> i32 {
//     //     let distance_line = Line::new(self.center(), other.center());
//     //     let mut distance = distance_line.start.distance_squared(distance_line.end);
//     //     let size1 = self.shape.bounding_box().size;
//     //     let size2 = other.shape.bounding_box().size;

//     //     distance -= (size1.area() + size2.area()) as i32;
//     //     distance
//     // }

//     // fn wall_bounce(&self, boundary: & impl Dimensions) -> GfxVec {
//     //     let mut delta = self.velocity;
//     //     let center = self.center();
//     //     let width = self.half_width() as i32;

//     //     // check against bounds
//     //     let r = boundary.bounding_box();
//     //     let top_left = r.top_left;
//     //     let bottom_right = Point::new(top_left.x + r.size.width as i32, top_left.y + r.size.height as i32);

//     //     if (center.x - top_left.x) <= width || (bottom_right.x - center.x) <= width {
//     //         // bounce off x axis
//     //         delta.x = -delta.x;
//     //     }
//     //     if (center.y - top_left.y) <= width || (bottom_right.y - center.y) <= width {
//     //         // bounce off y axis
//     //         delta.y = -delta.y;
//     //     }
//     //     delta
//     // }

//     // pub fn update_velocity(&mut self, other: &Self) -> bool{
//     //     let mut bounced = false;
//     //     if Self::is_collision(self, other) && self.is_moving() {
//     //         let velocity: GfxVec = self.reflected_vector(other.shape.surface_normal(self.center())).into();
//     //         self.set_velocity(velocity);
//     //         bounced = true;
//     //     }
//     //     bounced
//     // }

//     // #[inline]
//     // pub fn is_moving(&self) -> bool {
//     //     // self.velocity.magnitude() > 0.0
//     //     Point::from(self.velocity) != Point::zero()
//     // }

//     // /// move object by applying the direction vector, while checking against containing rectangle
//     // pub fn move_object_bounded(&mut self, boundary: & impl Dimensions) {
//     //     let mut delta = self.velocity();
//     //     if self.is_moving() {
//     //         let center = self.center();
//     //         let width = self.half_width() as i32;
    
//     //         // check against bounds
//     //         let r = boundary.bounding_box();
//     //         let top_left = r.top_left;
//     //         let bottom_right = Point::new(top_left.x + r.size.width as i32, top_left.y + r.size.height as i32);
    
//     //         if (center.x - top_left.x) <= width || (bottom_right.x - center.x) <= width {
//     //             // bounce off x axis
//     //             delta.x = -delta.x;
//     //         }
//     //         if (center.y - top_left.y) <= width || (bottom_right.y - center.y) <= width {
//     //             // bounce off y axis
//     //             delta.y = -delta.y;
//     //         }
    
//     //         self.set_velocity(delta);
//     //         self.shape.translate_mut(Point::new(delta.x.round() as i32, delta.y.round() as i32));
//     //     }
//     // }

//     // /// apply the velocity to the position of the shape (ignoring any collisions)
//     // pub fn move_object(&mut self) {
//     //     let delta = self.velocity();
//     //     if self.is_moving() {
//     //         self.shape.translate_mut(Point::new(delta.x.round() as i32, delta.y.round() as i32));
//     //     }
//     // }

// }
