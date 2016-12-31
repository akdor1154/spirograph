use std::f64::consts::PI;
use std::vec::Vec;
use std::ops::Add;
use std::ops::Sub;
use std::ops::Mul;
use std::cell::Cell;
use std::fmt;



fn main() {

	let origin_attachment = OriginGearAttachment{};

	let origin_gear = CircularGear{
		inner_radius: 0.5,
		outer_radius: 0.8,
		parent_attachment: &origin_attachment
	};

	let child_attachment = GearOnInsideEdgeOfParentAttachment{
		theta: Cell::new(0.0),
		parent_gear: &origin_gear
	};

	let child_gear = CircularGear{
		inner_radius: 0.0,
		outer_radius: 0.2,
		parent_attachment:
	&child_attachment};

	let child_axle = SharedAxleAttachment{
		parent_gear: &child_gear,
		theta: Cell::new(0.0)
	};

	let point_on_gear = PointOnGear{
		radius: 0.1,
		theta: 0.125 * PI,
		parent_attachment: &child_axle
	};

	/*
    let (start, step, end) = (0.0, 1.0/16.0, 1.0);
    for i in (0..).map(|x| start+step*(x as f64)).take_while(|&x| x<end) {
    	let theta = i * 2.0*PI;
        println!("setting theta to {:?}", theta);
        child_attachment.theta.set(theta);
		println!("point_on_gear is at {0:?} and {1:?} rev. .", point_on_gear.get_centre(), point_on_gear.get_angle() / (2.0*PI) );
    };


    for i in (0..).map(|x| start+step*(x as f64)).take_while(|&x| x<end) {
    	let theta = i * 2.0*PI;
        child_attachment.theta.set(theta);
        let centre = point_on_gear.get_centre();
		println!("{0}, {1}, {2}", centre.x, centre.y, child_gear.get_angle()  );
    };
	*/

	let offset_d = get_bezier_path_string(|theta| {

       		child_attachment.theta.set(theta);
	        let centre = point_on_gear.get_centre();
	        let derivative = point_on_gear.get_centre_d_theta();
	        (centre, derivative)
	}, 1024);

	let centre_d = get_bezier_path_string(|theta| {
       		child_attachment.theta.set(theta);
	        let centre = child_gear.get_centre();
	        let derivative = child_gear.get_centre_d_theta();
	        (centre, derivative)
	}, 64);

	let offset_path = format!("<path style='fill: none; stroke-width: 1; stroke: black' d='{}' />", offset_d);

	let centre_path = format!("<path style='fill: none; stroke-width: 1; stroke: red' d='{}' />", centre_d);



	println!("<svg viewBox='-100 -100 200 200'>{}{}</svg>" , offset_path, centre_path);
}

#[derive(Clone, Debug)]
struct Point {
	x: f64,
	y: f64
}

impl Point {
	fn rotated_by(&self, theta: f64) -> Point {
		let (sin_theta, cos_theta) = theta.sin_cos();
		Point {
			x: cos_theta*self.x - sin_theta*self.y,
			y: sin_theta*self.x + cos_theta*self.y
		}
	}
}


impl<'a, 'b> Add<&'b Point> for &'a Point {
	type Output = Point;
	fn add(self, p2: &'b Point) -> Point {
		Point {
			x: self.x + p2.x,
			y: self.y + p2.y
		}
	}
}

impl<'b> Add<&'b Point> for Point {
	type Output = Point;
	fn add(self, p2: &'b Point) -> Point { &self + p2 }
}

impl<'a, 'b> Sub<&'b Point> for &'a Point {
	type Output = Point;
	fn sub(self, p2: &'b Point) -> Point {
		Point {
			x: self.x - p2.x,
			y: self.y - p2.y
		}
	}
}

impl<'a> Sub<Point> for &'a Point {
	type Output = Point;
	fn sub(self, p2: Point) -> Point { self - &p2 }
}

impl<'a> Mul<f64> for &'a Point {
	type Output = Point;
	fn mul(self, c: f64) -> Point {
		Point {
			x: self.x * c,
			y: self.y * c
		}
	}
}

impl Mul<f64> for Point {
	type Output = Point;
	fn mul(self, c: f64) -> Point { &self * c }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}, {}", self.x, self.y)
    }
}


trait Attachment<'c> {
	type Child: 'c;
	fn get_child_centre(&self, child: &Self::Child) -> Point;
	fn get_child_centre_d_theta(&self, child: &Self::Child) -> Point;
	fn get_child_angle(&self, child: &Self::Child) -> f64;
	fn get_child_angle_d_theta(&self, child: &Self::Child) -> f64;
}

struct CircularGear<'a> {
	inner_radius: f64,
	outer_radius: f64,
	parent_attachment: &'a Attachment<'a, Child=CircularGear<'a>>
}

trait Positioned {
	fn get_centre_d_t(&self, d_theta_d_t: f64) -> Point;
	fn get_centre(&self) -> Point;
	fn get_angle(&self) -> f64;
}

impl<'a> Positioned for CircularGear<'a> {
	fn get_centre_d_t(&self, d_theta_d_t: f64) -> Point {
		self.parent_attachment.get_child_centre_d_theta(self)*d_theta_d_t
	}
	fn get_centre(&self) -> Point { self.parent_attachment.get_child_centre(self) }
	fn get_angle(&self) -> f64 { self.parent_attachment.get_child_angle(self) }
}

struct GearOnInsideEdgeOfParentAttachment<'a> {
	parent_gear: &'a CircularGear<'a>,
	theta: Cell<f64> // angle from parent's centre
}


impl<'p, 'c> Attachment<'c> for GearOnInsideEdgeOfParentAttachment<'p> {
	type Child = CircularGear<'c>;
	fn get_child_centre(&self, child: & Self::Child) -> Point {
		let absolute_angle = self.parent_gear.get_angle() + self.theta.get();
		let attachment_point_relative_to_parent = Point{
			x: self.parent_gear.inner_radius - child.outer_radius,
			y: 0.0
		}.rotated_by(absolute_angle);

		let attachment_point = &attachment_point_relative_to_parent + &self.parent_gear.get_centre();
		return attachment_point;

	}

	fn get_child_centre_d_theta(&self, child: &Self::Child) -> Point {
		let derivative = Point{
			x: self.parent_gear.inner_radius - child.outer_radius,
			y: 0.0
		}.rotated_by(self.theta.get() + PI*0.5);


		let parent_derivative = self.parent_gear.get_centre_d_theta();
		return derivative;

	}

	fn get_child_angle(&self, child: &Self::Child) -> f64 {
		let child_angle_relative_to_parent =
			-self.theta.get() * self.parent_gear.inner_radius / child.outer_radius;
		let child_angle = child_angle_relative_to_parent + self.parent_gear.get_angle();
		return child_angle
	}

	fn get_child_angle_d_theta(&self, child: &Self::Child) -> f64 {
		-self.parent_gear.inner_radius / child.outer_radius
	}
}

struct PointOnGear<'a> {
	radius: f64,
	theta: f64,
	parent_attachment: &'a Attachment<'a, Child=PointOnGear<'a>>
}

impl<'a> Positioned for PointOnGear<'a> {
	fn get_centre(&self) -> Point {
		let gear_centre = self.parent_attachment.get_child_centre(self);
		let gear_angle = self.parent_attachment.get_child_angle(self);
		let point_relative_to_parent = Point{
			x: self.radius,
			y: 0.0
		}.rotated_by((self.theta) + gear_angle);
		return &point_relative_to_parent + &gear_centre;
	}

	// this is a shitty name, it's not with respect to PointOnGear::theta but instead
	// with respect to...? (parent_attachment.theta?)
	fn get_centre_d_t(&self, d_theta_d_t: f64) -> Point {
		let gear_centre_d_theta = self.parent_attachment.get_child_centre_d_theta(self);
		let gear_angle = self.parent_attachment.get_child_angle(self);
		let gear_angle_d_theta = self.parent_attachment.get_child_angle_d_theta(self);
		let centre_d_theta = Point{
			x: self.radius,
			y: 0.0
		}.rotated_by(gear_angle + self.theta + PI*0.5) * gear_angle_d_theta;

		return (&gear_centre_d_theta + &centre_d_theta) * d_theta_d_t;
	}

	fn get_angle(&self) -> f64 {
		return self.theta + self.parent_attachment.get_child_angle(self);
	}

}

struct SharedAxleAttachment<'a> {
	parent_gear: &'a CircularGear<'a> ,
	theta: Cell<f64>
}

impl<'p, 'c> Attachment<'c> for SharedAxleAttachment<'p> {
	type Child = PointOnGear<'c>;
	fn get_child_centre(&self, _: &PointOnGear) -> Point {
		return self.parent_gear.get_centre();
	}

	fn get_child_centre_d_thetaz(&self, _: &PointOnGear) -> Point {
		return self.parent_gear.get_centre_d_theta();
	}

	fn get_child_angle(&self, _: &PointOnGear) -> f64 {
		return self.parent_gear.get_angle() + self.theta.get();
	}

	fn get_child_angle_d_theta(&self, _: &PointOnGear) -> f64 {
		return 1.0;
	}
}


struct OriginGearAttachment {
}

impl<'c> Attachment<'c> for OriginGearAttachment {
	type Child = CircularGear<'c>;

	fn get_child_centre(&self, _: & Self::Child) -> Point {
		static ORIGIN: Point = Point{ x: 0.0, y: 0.0 };
		return ORIGIN.clone();
	}

	fn get_child_centre_d_theta(&self, _: &Self::Child) -> Point {
		return Point{x: 0.0, y: 0.0};
	}

	fn get_child_angle(&self, _: & Self::Child) -> f64 {
		static START_ANGLE: f64 = 0.0;
		return START_ANGLE;
	}

	fn get_child_angle_d_theta(&self, _: &Self::Child) -> f64 {
		return 0.0;
	}

}

struct CubicBezierPiece {
	p_0: Point,
	p_1: Point,
	cp_0: Point,
	cp_1: Point
}

impl CubicBezierPiece {

	fn from_points(p_0: &Point, p_1: &Point, dp_0: &Point, dp_1: &Point, n_points: u32, theta_range: f64) -> CubicBezierPiece {
		const ONE_THIRD: f64 = 1.0/3.0;
		let MAGIC_FACTOR: f64 = theta_range / (n_points as f64);
		let cp_0 = dp_0 * ONE_THIRD*MAGIC_FACTOR + p_0;
		let cp_1 = p_1 - dp_1 * ONE_THIRD*MAGIC_FACTOR;
		return CubicBezierPiece {
			p_0: p_0.clone(),
			p_1: p_1.clone(),
			cp_0: cp_0,
			cp_1: cp_1
		}
	}
}

fn get_bezier_path_string<F>(f: F, n_steps: u32) -> String
	where F: Fn(f64) -> (Point, Point) {

	const THETA_RANGE: f64 = 4.0*PI;
    let centre_and_derivs = (0..n_steps+1)
    	.map(|n| (n as f64)/(n_steps as f64)*THETA_RANGE )
    	.map(f)
    	.collect::<Vec<_>>();
    let beziers = centre_and_derivs.windows(2)
    	.map( |pair| {
	    	let p_0 = &pair[0].0;
	    	let dp_0 = &pair[0].1;
	    	let p_1 = &pair[1].0;
	    	let dp_1 = &pair[1].1;

	    	CubicBezierPiece::from_points(p_0, p_1, dp_0, dp_1, n_steps, THETA_RANGE)
	    });


    let mut centre_path_d = Vec::new();
	for bezier in beziers {
		if centre_path_d.len() == 0 {
    		centre_path_d.push(format!("M {p_0}", p_0 = bezier.p_0*100.0));
		}
		centre_path_d.push(format!("C {cp_0} {cp_1} {p_1}",
										//p_0 = bezier.p_0,
										p_1 = bezier.p_1*100.0,
										cp_0 = bezier.cp_0*100.0,
										cp_1 = bezier.cp_1*100.0));
	}

	centre_path_d.push("Z".to_owned());

	let centre_path_d_str = centre_path_d.join("\n");
	centre_path_d_str
}