use embedded_graphics::{geometry::AnchorPoint, prelude::{Point, Size}, primitives::{Circle, Line, Polyline, Rectangle, Triangle}};

use crate::sprites::vectors::{SpriteVector, VectorComponent};

// use crate::gfx::GfxVec;


pub trait PointExt {
    fn magnitude(&self) -> i32;
    
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
    
    fn magnitude(&self) -> i32 {
        (self.x * self.x) + (self.y * self.y).isqrt()
    }
}

pub trait Area {
    fn area(&self) -> u32;
}

pub trait SurfaceNormal {
    /// compute the Vector Normal to the Surface between self and point as a Point
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point;

    /// distance from nearest surface to point
    fn distance(&self, point: impl Into<Point> + Copy) -> f32;
}

pub trait ClosestEdge {
    fn closest_edge(&self, point: Point) -> Line;
}





impl SurfaceNormal for Rectangle {
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        self.closest_edge(point.into()).surface_normal(point)
    }
    
    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        self.closest_edge(point.into()).distance(point)
    }
}

impl SurfaceNormal for Line {

    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        let point: Point = point.into();
        let closest_point = {
            let line_vec = self.delta();
            let point_vec = point - self.start;
            let line_len_sq = self.start.distance_squared(self.end) as f32;

            if line_len_sq == 0.0 { 
                self.start
            } else {              
                let t = point_vec.dot_product(line_vec) as f32 / line_len_sq;
                let t = t.clamp(0.0, 1.0);
    
                Point {
                    x: (self.start.x as f32 + t * line_vec.x as f32) as i32,
                    y: (self.start.y as f32 + t * line_vec.y as f32) as i32,
                }
            }
        };
        closest_point - point
    }
    
    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        let point: Point = point.into();
        let closest_point = {
            let line_vec = self.delta();
            let point_vec = point - self.start;
            let line_len_sq = self.start.distance_squared(self.end) as f32;

            if line_len_sq == 0.0 { 
                self.start
            } else {              
                let t = point_vec.dot_product(line_vec) as f32 / line_len_sq;
                let t = t.clamp(0.0, 1.0);
    
                Point {
                    x: (self.start.x as f32 + t * line_vec.x as f32) as i32,
                    y: (self.start.y as f32 + t * line_vec.y as f32) as i32,
                }
            }
        };
        closest_point.distance(point) as f32
    }    
}


impl Area for Size {
    fn area(&self) -> u32 {
        self.height * self.width
    }
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
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        let point: Point = point.into();
        self.closest_edge(point).surface_normal(point)
    }
    
    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        let point: Point = point.into();
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
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        let point: Point = point.into();
        self.closest_edge(point).surface_normal(point)
    }
    
    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        let point: Point = point.into();
        self.closest_edge(point).distance(point)
    }
}

impl SurfaceNormal for Circle {
    // compute the normalized surface normal between self and point
    fn surface_normal(&self, point: impl Into<Point> + Copy) -> Point {
        let point: Point = point.into();
        self.center() - point
    }
    
    fn distance(&self, point: impl Into<Point> + Copy) -> f32 {
        (self.center().distance(point.into()) as u32 - (self.diameter / 2)) as f32
    }
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


fn intersect_lines(first_line: Line, other_line: Line) -> Option<Point> {        
    let a1 = micromath::F32((first_line.end.y - first_line.start.y) as f32);
    let b1 = micromath::F32((first_line.start.x - first_line.end.x) as f32);
    let c1 = a1 * first_line.start.x as f32 + b1 * first_line.start.y as f32;

    let a2 = micromath::F32((other_line.end.y - other_line.start.y) as f32);
    let b2 = micromath::F32((other_line.start.x - other_line.end.x) as f32);
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


