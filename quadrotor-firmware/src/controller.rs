use core::f32::consts::FRAC_PI_2;

use quadrotor_x::datatypes::{ControllerParams, ControllerSetpoints, MotorSetpoints, PidParams, Quatf};

const YAW_ANGULAR_VELOCITY: f32 = FRAC_PI_2;

fn truncate_pos_or_neg(value: f32, abs_max: f32) -> f32 {
    if value > abs_max {
        abs_max
    } else if value < -abs_max {
        -abs_max
    } else {
        value
    }
}

struct Pid {
    integral: f32,
}

impl Pid {
    pub fn new() -> Self {
        Self {
            integral: 0f32,
        }
    }

    pub fn clear_integral(&mut self) {
        self.integral = 0.0;
    }

    pub fn get_output(&mut self, params: PidParams, setpoint: f32, delta_t: f32, current_val: f32, last_val: f32) -> f32 {
        let error = setpoint - current_val;

        self.integral = self.integral + (params.i * error * delta_t);
        self.integral = truncate_pos_or_neg(self.integral, params.integral_max);

        let output = params.p * error +
            self.integral -
            params.d * (current_val - last_val) / delta_t;

        output
    }
}

pub struct Controller {
    pub params: ControllerParams,
    last_orientation: Option<Quatf>,
    yaw_setpoint: Option<f32>,
    thrust_pid: Pid,
    roll_pid: Pid,
    pitch_pid: Pid,
    yaw_pid: Pid,
}

impl Controller {
    pub fn new(params: ControllerParams) -> Self {
        Self {
            params,
            last_orientation: None,
            yaw_setpoint: None,
            thrust_pid: Pid::new(),
            roll_pid: Pid::new(),
            pitch_pid: Pid::new(),
            yaw_pid: Pid::new(),
        }
    }

    pub fn reset(self: &mut Self) {
        self.last_orientation = None;
        self.yaw_setpoint = None;
        self.thrust_pid.clear_integral();
        self.roll_pid.clear_integral();
        self.pitch_pid.clear_integral();
        self.yaw_pid.clear_integral();
    }

    pub fn update(
        self: &mut Self,
        orientation: &Quatf,
        setpoints: &ControllerSetpoints,
        delta_t: f32,
    ) -> MotorSetpoints {
        // Retrieve & update "last" values (used for calculating PID derivative term).
        let eulers = orientation.as_euler_angles();
        let last_eulers = match self.last_orientation {
            Some(value) => value.as_euler_angles(),
            None => eulers,
        };

        // NOTE: The yaw input is treated as an angular velocity setpoint, so integrate it to get
        //       the yaw angle setpoint.
        let last_yaw_setpoint = self.yaw_setpoint.unwrap_or(eulers.z);
        let yaw_setpoint = last_yaw_setpoint + (YAW_ANGULAR_VELOCITY * setpoints.yaw * delta_t);
        self.yaw_setpoint = Some(yaw_setpoint);

        // Update PIDs
        // TODO: Implement altitude estimate to get thrust feedback. For now, this is basically just
        //       a gain setting for the controller input.
        let thrust_output = self.thrust_pid.get_output(
            self.params.thrust,
            setpoints.thrust,
            delta_t,
            0.,
            0.,
        );
        let roll_output = self.roll_pid.get_output(
            self.params.roll,
            setpoints.roll,
            delta_t,
            eulers.x,
            last_eulers.x,
        );
        let pitch_output = self.pitch_pid.get_output(
            self.params.pitch,
            setpoints.pitch,
            delta_t,
            eulers.y,
            last_eulers.y,
        );
        let yaw_output = self.yaw_pid.get_output(
            self.params.yaw,
            yaw_setpoint,
            delta_t,
            eulers.z,
            last_eulers.z,
        );

        // Clamp max output of any one axis to 1. The setpoints are normalized, so this basically
        // just means it can contribute the maximum +/- value for the motor, but no more, so that
        // it can't _completely_ drown out the other axes if the controller gets in a bad state.
        let thrust_output = truncate_pos_or_neg(thrust_output, 1.);
        let roll_output = truncate_pos_or_neg(roll_output, 1.);
        let pitch_output = truncate_pos_or_neg(pitch_output, 1.);
        let yaw_output = truncate_pos_or_neg(yaw_output, 1.);

        let front_left = (thrust_output + roll_output + pitch_output - yaw_output).clamp(0., 1.);
        let front_right = (thrust_output - roll_output + pitch_output + yaw_output).clamp(0., 1.);
        let back_left = (thrust_output + roll_output - pitch_output + yaw_output).clamp(0., 1.);
        let back_right = (thrust_output - roll_output - pitch_output - yaw_output).clamp(0., 1.);

        MotorSetpoints {
            front_left,
            front_right,
            back_left,
            back_right,
        }
    }
}
