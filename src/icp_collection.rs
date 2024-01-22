use std::ops::{Deref, DerefMut};

use crate::ICPPoint;
use kdtree::{distance::squared_euclidean, KdTree};
use nalgebra as na;

pub(crate) trait ICPCol<T: ICPPoint> {
    type Collection;
    fn new(col: Self::Collection)->Self;
    fn get_points(&self) -> &[T];
    fn inner(self) -> Self::Collection;
}

pub(crate) trait ICPColCenterOfMass<T: ICPPoint>: ICPCol<T> {
    fn calculate_center_of_mass(&self) -> na::Point2<f32> {
        let mut center_of_mass = na::Point2::new(0.0, 0.0);
        let pts = self.get_points();
        pts.iter().for_each(|p| center_of_mass += p.point().coords);
        center_of_mass /= pts.len() as f32;
        return center_of_mass;
    }

    fn assert_valid(&self) {
        self.get_points().iter().for_each(|p| assert!(p.is_data_valid()));
        assert!(points_equal_tolerance(self.get_center_of_mass(), self.calculate_center_of_mass()));
    }

    fn get_center_of_mass(&self) -> na::Point2<f32>;
}

pub(crate) struct KDTreedIcpCollection<'a,T: ICPPoint> {
    collection: &'a [T],
    kd_tree: KdTree<f32, usize, [f32; 2]>,
    center_of_mass: na::Point2<f32>,
}

impl<'a,T: ICPPoint> KDTreedIcpCollection<'a,T> {
    pub fn closest_point_kd(&mut self, point: na::Point2<f32>) -> na::Point2<f32> {
        let pt = self
            .kd_tree
            .iter_nearest(&[point.x, point.y], &squared_euclidean)
            .unwrap()
            .next()
            .unwrap();

        self.collection[*pt.1].point()
    }


}

impl<'a,T: ICPPoint> ICPColCenterOfMass<T> for KDTreedIcpCollection<'a,T> {
    fn get_center_of_mass(&self) -> na::Point2<f32> {
        self.center_of_mass
    }
}

impl<'a,T: ICPPoint> ICPCol<T> for KDTreedIcpCollection<'a,T> {
    type Collection = &'a[T];
    fn new(points: &'a [T]) -> Self {
        let mut kd_tree = KdTree::new(2);
        points.iter().enumerate().for_each(|(i, p)| {
            let p = p.point();
            kd_tree.add([p.x, p.y], i).unwrap()
        });

        let mut this = Self {
            kd_tree,
            collection: points,
            center_of_mass: Default::default(),
        };
        this.center_of_mass = this.calculate_center_of_mass();
        this
    }
    fn get_points(&self) -> &[T] {
        self.collection
    }
    fn inner(self) -> Self::Collection {
        self.collection
    }
}

#[derive(Clone)]
pub(crate) struct ICPCollection<T: ICPPoint> {
    points: Vec<T>,
    center_of_mass: na::Point2<f32>,
}

impl<T: ICPPoint> ICPCol<T> for ICPCollection<T> {
    type Collection = Vec<T>;
    fn new(points: Vec<T>) -> Self {
        let mut this = Self {
            points,
            center_of_mass: Default::default(),
        };
        this.center_of_mass = this.calculate_center_of_mass();
        this
    }
    fn get_points(&self) -> &[T] {
        &self.points
    }
    fn inner(self) -> Self::Collection {
        self.points
    }
}

impl<T: ICPPoint> ICPColCenterOfMass<T> for ICPCollection<T> {
    fn get_center_of_mass(&self) -> na::Point2<f32> {
        self.center_of_mass
    }
}

impl<T: ICPPoint> ICPCollection<T> {



    pub fn translate(&mut self, x: f32, y: f32) -> &mut Self {
        self.points.iter_mut().for_each(|p| {
            //v.x += val;
            *p = p.translate(x, y)
        });
        self.center_of_mass += na::Point2::new(x, y).coords;
        debug_assert!(points_equal_tolerance(
            self.center_of_mass,
            self.calculate_center_of_mass()
        ));
        self
    }

    pub fn rotate(&mut self, rot_rad: f32) -> &mut Self {
        self.points.iter_mut().for_each(|p| {
            *p = p.rotate(rot_rad);
        });
        self.center_of_mass = self.calculate_center_of_mass();

        // *center_of_mass = ScanMeasurement::from_point(*center_of_mass).rotate(rot_rad).point;
        // todo!("Rotate center of mass");
        debug_assert!(points_equal_tolerance(
            self.get_center_of_mass(),
            self.calculate_center_of_mass()
        ));
        self
    }

}

fn points_equal_tolerance(p1: na::Point2<f32>, p2: na::Point2<f32>) -> bool {
    fn floats_are_equal(a: f32, b: f32) -> bool {
        (a - b).abs() <= 0.001
    }
    return floats_are_equal(p1.x, p2.x) && floats_are_equal(p1.y, p2.y);
}
