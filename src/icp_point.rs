use nalgebra as na;
pub trait ICPPoint: Clone{
    fn translate(&self,x: f32,y: f32) -> Self;
    fn rotate(&self,angle_rad: f32) -> Self;
    fn point(&self) -> na::Point2<f32>;
    fn is_data_valid(&self) -> bool;
}


impl ICPPoint for na::Point2<f32>{
    fn translate(&self,x: f32,y: f32) -> Self {
        na::Point2::new(self.x+x,self.y+y)
    }

    fn rotate(&self,angle_rad: f32) -> Self {
        na::Rotation2::new(angle_rad) * (*self)   
    }

    fn point(&self) -> na::Point2<f32> {
        *self
    }

    fn is_data_valid(&self) -> bool {
        true
    }
}