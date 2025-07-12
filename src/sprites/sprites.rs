use core::f32::consts::PI;

use defmt::info;
use embedded_graphics::{prelude::{Dimensions, DrawTarget, PixelColor, Point, Transform}, primitives::{self, PrimitiveStyle, StyledDrawable}};

use crate::{gfx::GfxVec, sprites::geometry::{PointExt, SurfaceNormal}}; 
use crate::sprites::{geometry::Area, vectors::GfxVector};

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
                        // polyline.points().map(|p| info!("{}", p));
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
        let mut distance = self.center().distance_squared(other.center());
        distance -= (self.area() + other.area()) as i32;
        distance
    }

    pub fn calculate_reflection_vector(
        incoming_velocity: &GfxVector,
        collision_normal: &GfxVector,
        coefficient_of_restitution: f32,
    ) -> GfxVector {
        // Ensure the normal is normalized (unit length)
        // let mag = collision_normal.magnitude()
        let normal = collision_normal.0.normalize();

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
    fn bounding_box(&self) -> primitives::Rectangle {
        match self {
            SpritePrimitive::Line(line) => line.bounding_box(),
            SpritePrimitive::Circle(circle) => circle.bounding_box(),
            SpritePrimitive::Rectangle(rectangle) => rectangle.bounding_box(),
            SpritePrimitive::Polyline(polyline) => polyline.bounding_box(),
            SpritePrimitive::Triangle(triangle) => triangle.bounding_box(),
        }
    }
}

impl<'a> From<primitives::Line> for SpritePrimitive<'a> {
    fn from(value: primitives::Line) -> Self {
        Self::Line(value)
    }
}
impl<'a> From<primitives::Circle> for SpritePrimitive<'a> {
    fn from(value: primitives::Circle) -> Self {
        Self::Circle(value)
    }
}
impl<'a> From<primitives::Rectangle> for SpritePrimitive<'a> {
    fn from(value: primitives::Rectangle) -> Self {
        Self::Rectangle(value)
    }
}
impl<'a> From<primitives::Triangle> for SpritePrimitive<'a> {
    fn from(value: primitives::Triangle) -> Self {
        Self::Triangle(value)
    }
}
impl<'a> From<primitives::Polyline<'a>> for SpritePrimitive<'a> {
    fn from(value: primitives::Polyline<'a>) -> Self {
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

impl<'a> SurfaceNormal for SpritePrimitive<'a> {
    fn surface_normal(&self, point: impl Into<GfxVec> + Into<Point> + Copy) -> GfxVec {
        match self {
            SpritePrimitive::Line(line) => line.surface_normal(point),
            SpritePrimitive::Circle(circle) => circle.surface_normal(point),
            SpritePrimitive::Rectangle(rectangle) => rectangle.surface_normal(point),
            SpritePrimitive::Polyline(polyline) => polyline.surface_normal(point),
            SpritePrimitive::Triangle(triangle) => triangle.surface_normal(point),
        }
    }
    
    fn distance(&self, point: impl Into<GfxVec> + Into<Point> + Copy) -> f32 {
        match self {
            SpritePrimitive::Line(line) => line.distance(point),
            SpritePrimitive::Circle(circle) => circle.distance(point),
            SpritePrimitive::Rectangle(rectangle) => rectangle.distance(point),
            SpritePrimitive::Polyline(polyline) => polyline.distance(point),
            SpritePrimitive::Triangle(triangle) => triangle.distance(point),
        }
    }
}
