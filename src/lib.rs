mod icp_collection;
use icp_collection::{ICPCollection, ICPCol, KDTreedIcpCollection};

mod icp_point;
use nalgebra as na;


pub use icp_point::ICPPoint;


pub struct ICPResult{
    pub x_offset: f32,
    pub y_offset: f32,
    pub rotation_offset_rad: f32,
    ///Value between 0.0 and 1.0
    pub convergence: f32
}


pub struct Icp<'a,TRef: ICPPoint, TOther: ICPPoint> {
    points_reference: KDTreedIcpCollection<'a,TRef>,
    points_other: ICPCollection<TOther>,
    max_iterations: usize,
    convergence_distance: f32,
    convergence_rotation: f32,
    convergence_points_maxdist: f32,
}

impl<'a,TRef: ICPPoint, TOther: ICPPoint> Icp<'a,TRef,TOther> {
    pub fn new(scan1: &'a [TRef], scan2: Vec<TOther>,    max_iterations: usize,
        convergence_distance: f32,
        convergence_rotation: f32, convergence_points_maxdist: f32) -> Self {
        Self { points_reference: KDTreedIcpCollection::new(scan1), points_other: ICPCollection::new(scan2),max_iterations,convergence_distance,convergence_rotation, convergence_points_maxdist }
    }

    /// Converges at max 0.5cm and 0.1 degrees
    pub fn new_default(scan1: &'a [TRef], scan2: Vec<TOther>) -> Self {
        Self::new(scan1,scan2,50,0.005,0.1f32.to_radians(),0.01)
    }

    ///x,y in Meters, angle_rad in radians
    pub fn do_icp(mut self, x: f32, y: f32, angle_rad: f32) -> (ICPResult, Vec<TOther>) {
        let res = self.do_icp_generic(x, y, angle_rad, Self::center_of_mass_corresp_kd_with_svd);

        (res,self.points_other.inner())
    }


    fn do_icp_generic(&mut self, x: f32, y: f32, angle_rad: f32, transformation_fn: fn(&mut KDTreedIcpCollection<TRef>, &mut ICPCollection<TOther>) -> (na::Vector2<f32>, f32)) -> ICPResult {
        let mut total_translation = na::Vector2::new(x, y);
        let mut total_rotation = angle_rad;

        // Apply initial translation and rotation
        self.points_other.translate(x, y);
        self.points_other.rotate(angle_rad);

        let mut i = 0;
        while i < self.max_iterations {
            // Calculate translation and rotation
            let (translation_vector, rotation) = transformation_fn(&mut self.points_reference, &mut self.points_other);

            // Apply the calculated translation and rotation
            self.points_other.translate(translation_vector.x, translation_vector.y);
            self.points_other.rotate(rotation);

            // Update total translation and rotation
            total_translation += translation_vector;
            total_rotation += rotation;

            i += 1;

            if translation_vector.norm() < self.convergence_distance && rotation.abs() < self.convergence_rotation {
                break;
            }
        }
        let mut converged_count = 0;
        for pt in self.points_other.get_points(){
            let pt = pt.point();
            let closest = self.points_reference.closest_point_kd(pt);

            if (pt.x - closest.x).abs() < self.convergence_points_maxdist && (pt.y - closest.y).abs() < self.convergence_points_maxdist{
                converged_count+=1;
            }
        }

        ICPResult{x_offset: total_translation.x, y_offset: total_translation.y, rotation_offset_rad: total_rotation, convergence: converged_count as f32 / self.points_other.get_points().len() as f32 }
    }


    fn center_of_mass_corresp_kd_with_svd(scan1: &mut KDTreedIcpCollection<TRef>, scan2: &mut ICPCollection<TOther>) -> (na::Vector2<f32>, f32) {
        let n = scan2.get_points().len() as f32;

        // Compute centroids of corresponding points, iterating over scan2
        let mut centroid1 = na::Point2::new(0.0, 0.0);
        let mut centroid2 = na::Point2::new(0.0, 0.0);

        let corresp = scan2.get_points().iter().map(|p|p.point()).map(|p|(p,scan1.closest_point_kd(p))).collect::<Vec<_>>();

        for (point1,closest_point2) in &corresp{
            centroid1 += closest_point2.coords;
            centroid2 += point1.coords;
        }

        centroid1 /= n;
        centroid2 /= n;

        // Construct the cross-covariance matrix, iterating over scan2
        let mut h = na::Matrix2::zeros();
            for (point1,closest_point2) in &corresp{
            let d1 = closest_point2 - centroid1;
            let d2 = point1 - centroid2;
            h += d1 * d2.transpose();
        }

        // Perform SVD
        let svd = na::SVD::new(h, true, true);
        let u = svd.u.unwrap();
        let vt = svd.v_t.unwrap();

        // Compute rotation matrix
        let rotation_matrix = u * vt;

        // Extract the rotation angle from the rotation matrix
        let rotation_angle = rotation_matrix[(1, 0)].atan2(rotation_matrix[(0, 0)]);

        // Translation vector, adjusted to match the fast variant
        let translation_vector = centroid1 - rotation_matrix * centroid2;

        (translation_vector.into(), rotation_angle)
    }




    #[cfg(test)]
    fn do_icp_once_test(mut self, x: f32, y: f32, angle: f32) -> (ICPResult,ICPCollection<TOther>) {
        (self.do_icp_generic(x, y, angle, Self::center_of_mass_corresp_kd_with_svd),self.points_other)
    }
}


#[cfg(test)]
mod test{
    use std::{fs, ops::Sub};

    use super::*;
    use na::{Rotation2, Point2};
    use plotters::prelude::*;
    type Scan = Vec<na::Point2<f32>>;
    #[test]
    fn test_realworld(){
        parse_and_plot("LidarTest", |a,b|{
            let icp = Icp::new_default(a, b.clone());
            let (ICPResult { x_offset, y_offset, rotation_offset_rad, convergence },b_aligned) = icp.do_icp_once_test(0.0, 0.0, 0.0);
            println!("{}",convergence);
            assert!(-0.15f32.sub(x_offset).abs() < 0.01);
            assert!(0.0f32.sub(y_offset).abs() < 0.01);
            assert!(-1.77f32.to_radians().sub(rotation_offset_rad).abs() < 1.0f32.to_radians());
            // assert!(did_converge > 0.8);
            *b = b_aligned.inner();
            [x_offset,y_offset,rotation_offset_rad,convergence]
        });
    }


    #[test]
    fn test_effectiveness(){
        let scan1 = parse_scan("./assets/scan1.txt");

        let tests = [
            (-0.5,-0.5,60.0,"Meter5")
        ];

        for (x,y,rot,name) in tests{
            do_test(&scan1, &scan1, x, y, rot,name);

        }
    }

    /****
     * =======
     * Utils
     * =======
     ****/

    fn do_test(reference: &[na::Point2<f32>], scan2: &[na::Point2<f32>], x: f32,y:f32,rots_degree: f32, name: &str){
        let mut scan2 = scan2.iter().cloned().collect::<Vec<_>>();
        translate(&mut scan2, x, y);
        rotate(&mut scan2, rots_degree.to_radians());

        let (res,other) = Icp::new_default(reference, scan2.clone()).do_icp_once_test(0.0, 0.0, 0.0);
        plot(reference,&other.inner(),&scan2,[x,y,rots_degree.to_radians(),res.convergence],format!("./assets/{name}.svg")).unwrap();
        println!("{}",res.convergence);
    }

    fn rotate(v: &mut [na::Point2<f32>],rads: f32){
        let rot = Rotation2::new(rads);
        v.iter_mut().for_each(|p|*p = rot * *p);
    }

    fn translate(v: &mut [na::Point2<f32>],x: f32, y: f32){
        v.iter_mut().for_each(|p|*p = Point2::new(p.x+x,p.y+y));
    }

    fn parse_and_plot(name: &str, func: impl Fn(&mut Scan, &mut Scan)->[f32;4]) {
        let mut scan1 = parse_scan("./assets/scan1.txt");
        let mut scan2 = parse_scan("./assets/scan2.txt");
        let scan2_orig = scan2.clone();

        let res = func(&mut scan1, &mut scan2);
        plot(&scan1, &scan2, &scan2_orig, res, format!("./assets/{name}.svg")).unwrap();
    }

    pub fn parse_scan(name: &str) -> Scan {
        let str = fs::read_to_string(name).unwrap();
        str.split("\n").map(|line| {
            let mut split = line.split_whitespace();
            let p: na::Point2<f32> = na::Point2::new(
                split.next().unwrap().parse().unwrap(),
                split.next().unwrap().parse().unwrap(),
            );

            p
        }).collect()
    }

    fn plot(
        points1: &[na::Point2<f32>],
        points2: &[na::Point2<f32>],
        points2_orig: &[na::Point2<f32>],
        translation: [f32;4],
        name: String,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let [x, y, rot,convergence] = translation;

        let root = SVGBackend::new(&name, (720, 720)).into_drawing_area();
        root.fill(&WHITE)?;

        // Split the drawing area into two
        let (upper, lower) = root.split_vertically(360);

        // Create the first chart
        let mut chart1 = ChartBuilder::on(&upper)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(-4f32..4f32, -4f32..4f32)?;

        chart1.configure_mesh().draw()?;
        chart1.draw_series(points1.iter().map(|p| Circle::new((p.x, p.y), 1, BLUE.filled())))?;
        chart1.draw_series(points2_orig.iter().map(|p| Circle::new((p.x, p.y), 1, RED.filled())))?;

        // Create the second chart
        let mut chart2 = ChartBuilder::on(&lower)
            .margin(5)
            .x_label_area_size(30)
            .y_label_area_size(30)
            .build_cartesian_2d(-4f32..4f32, -4f32..4f32)?;

        chart2.configure_mesh().draw()?;
        chart2.draw_series(points1.iter().map(|p| Circle::new((p.x, p.y), 1, BLUE.filled())))?;
        chart2.draw_series(points2.iter().map(|p| Circle::new((p.x, p.y), 1, RED.filled())))?;

        // Add translation information as a label

        lower.draw_text(
            &format!("Translation: x={:.2}cm, y={:.2}cm, rot={:.2}deg, convergence: {:.2}%", x * 100.0, y * 100.0, rot.to_degrees(), convergence * 100.0),
            &("sans-serif", 20, &BLACK).into_text_style(&lower),
            (50, 5),
        )?;

        // Finish the drawing
        root.present()?;
        println!("Plot saved to {}", name);

        Ok(())
    }
}
