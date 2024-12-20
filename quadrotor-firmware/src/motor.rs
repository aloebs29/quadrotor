use embassy_nrf::gpio;
use embassy_nrf::pwm;

const MAX_DUTY: f32 = 1000.0;

pub enum Motor {
    FrontLeft = 0,
    FrontRight = 1,
    BackLeft = 2,
    BackRight = 3,
}

impl From<u8> for Motor {
    fn from(value: u8) -> Self {
        match value {
            0 => Motor::FrontLeft,
            1 => Motor::FrontRight,
            2 => Motor::BackLeft,
            3 => Motor::BackRight,
            _ => panic!("Invalid motor ID")
        }
    }
}

pub struct MotorOutputs<'a, T: pwm::Instance> {
    pwm: pwm::SimplePwm<'a, T>,
}

impl<T: pwm::Instance> MotorOutputs<'_, T> {
    pub fn new(
        pwm_peripheral: T,
        motor_out_front_left: gpio::AnyPin,
        motor_out_front_right: gpio::AnyPin,
        motor_out_back_left: gpio::AnyPin,
        motor_out_back_right: gpio::AnyPin,
    ) -> Self {
        let pwm = pwm::SimplePwm::new_4ch(
            pwm_peripheral,
            motor_out_front_left,
            motor_out_front_right,
            motor_out_back_left,
            motor_out_back_right,
        );
        pwm.set_prescaler(pwm::Prescaler::Div1);
        pwm.set_max_duty(MAX_DUTY as u16);

        let mut instance = Self { pwm };
        instance.set(Motor::FrontLeft, 0.0);
        instance.set(Motor::FrontRight, 0.0);
        instance.set(Motor::BackLeft, 0.0);
        instance.set(Motor::BackRight, 0.0);

        instance
    }

    pub fn set(self: &mut Self, motor: Motor, output_ratio: f32) {
        // TODO: Linearize output_ratio to thrust; correct for battery voltage drop?
        // PWM duty is inverted (for an active-high signal)
        let output_ratio = output_ratio.clamp(0.0, 1.0);
        let duty = (MAX_DUTY - (output_ratio * MAX_DUTY)) as u16;

        self.pwm.set_duty(motor as usize, duty);
    }
}
