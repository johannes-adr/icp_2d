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


impl ICPPoint for (f32,f32){
    fn translate(&self,x: f32,y: f32) -> Self {
        let new = na::Point2::new(self.0+x,self.1+y);
        (new.x,new.y)
    }

    fn rotate(&self,angle_rad: f32) -> Self {
        let new = na::Rotation2::new(angle_rad) * self.point() ;
        (new.x,new.y)
    }

    fn point(&self) -> na::Point2<f32> {
        na::Point2::new(self.0,self.1)
    }

    fn is_data_valid(&self) -> bool {
        true
    }
}