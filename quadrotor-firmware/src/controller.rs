use quadrotor_x::datatypes::{ControllerParams, ControllerSetpoints, MotorSetpoints, Quatf, Vec3f};
use quadrotor_x::sensor_fusion::madgwick_fusion_9;

pub struct Controller {
    pub params: ControllerParams,
    pub setpoints: ControllerSetpoints,
    orientation: Quatf,
    last_update: f32,
}

impl Controller {
    pub fn new(params: ControllerParams, setpoints: ControllerSetpoints, timestamp: f32) -> Self {
        Self {
            params,
            setpoints,
            orientation: Quatf::default(),
            last_update: timestamp,
        }
    }

    pub async fn update(
        self: &mut Self,
        timestamp: f32,
        accel: &Vec3f,
        gyro: &Vec3f,
        mag: &Vec3f,
    ) -> MotorSetpoints {
        let delta_t = timestamp - self.last_update;
        self.last_update = timestamp;

        self.orientation = madgwick_fusion_9(self.orientation, *accel, *gyro, *mag, delta_t);

        // TODO: Implement controller logic
        MotorSetpoints::zeroed()
    }

    pub fn get_orientation(self: &Self) -> Quatf {
        self.orientation
    }
}
