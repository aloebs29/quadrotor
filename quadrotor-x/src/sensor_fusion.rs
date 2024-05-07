use micromath::vector::{F32x3, Vector};
use micromath::Quaternion;

// NOTE: Only use micromath f32::sqrt function on target. Unit tests need to use more accurate sqrt function found in
//       std.
#[cfg(not(feature = "std"))]
use micromath::F32Ext;

const BETA_IMU: f32 = 0.031;
const BETA_MARG: f32 = 0.041;
const G_TO_MPS2: f32 = 9.80665;

fn vec3_norm(v: F32x3) -> f32 {
    v.iter().map(|n| n * n).sum()
}

fn try_normalize_vec3(v: F32x3) -> Option<F32x3> {
    let magnitude = v.magnitude();
    if magnitude.is_normal() {
        Some(v * (1. / magnitude))
    } else {
        None
    }
}

fn try_normalize_quat(q: Quaternion) -> Option<Quaternion> {
    let norm = q.norm();
    if norm == 0.0 {
        None
    } else {
        Some(q.scale(1. / norm.sqrt()))
    }
}

pub fn madgwick_fusion_6(q: Quaternion, accel: F32x3, gyro: F32x3, delta_t: f32) -> Quaternion {
    // Compute derivative using last estimate and gyro
    let mut q_dot = 0.5 * (q * Quaternion::new(0., gyro.x, gyro.y, gyro.z));

    // Normalize accel data
    if let Some(a_hat) = try_normalize_vec3(accel) {
        // Objective function
        let f = F32x3 {
            x: 2. * (q.x() * q.z() - q.w() * q.y()),
            y: 2. * (q.w() * q.x() + q.y() * q.z()),
            z: 2. * (0.5 - q.x() * q.x() - q.y() * q.y()),
        } - a_hat;

        if vec3_norm(f) > 0. {
            // Jacobian
            #[rustfmt::skip]
            let j = (
                (-2. * q.y(), 2. * q.z(), -2. * q.w(), 2. * q.x()),
                (2. * q.x(), 2. * q.w(), 2. * q.z(), 2. * q.y()),
                (0., -4. * q.x(), -4. * q.y(), 0.)
            );

            // Objective function gradient (J.T * f)
            let gradient = Quaternion::new(
                j.0 .0 * f.x + j.1 .0 * f.y + j.2 .0 * f.z,
                j.0 .1 * f.x + j.1 .1 * f.y + j.2 .1 * f.z,
                j.0 .2 * f.x + j.1 .2 * f.y + j.2 .2 * f.z,
                j.0 .3 * f.x + j.1 .3 * f.y + j.2 .3 * f.z,
            );
            if let Some(gradient_hat) = try_normalize_quat(gradient) {
                q_dot -= gradient_hat.scale(BETA_IMU);
            }
        }
    }

    let qnew = q + q_dot.scale(delta_t);
    if let Some(qnew_hat) = try_normalize_quat(qnew) {
        qnew_hat
    } else {
        q
    }
}

pub fn madgwick_fusion_9(
    q: Quaternion,
    accel: F32x3,
    gyro: F32x3,
    mag: F32x3,
    delta_t: f32,
) -> Quaternion {
    // Normalize magnetometer data; fall back to IMU update if mag data is nil
    let m_hat = match try_normalize_vec3(mag) {
        Some(v) => v,
        None => return madgwick_fusion_6(q, accel, gyro, delta_t),
    };

    // Compute derivative using last estimate and gyro
    let mut q_dot = 0.5 * (q * Quaternion::new(0., gyro.x, gyro.y, gyro.z));

    // Normalize accel data
    if let Some(a_hat) = try_normalize_vec3(accel) {
        // Rotate mag vector and eliminate y-component
        let h = q.rotate(m_hat);
        let bx = (h.x * h.x + h.y * h.y).sqrt();
        let bz = h.z;

        // Objective functions
        let fg = F32x3 {
            x: 2. * (q.x() * q.z() - q.w() * q.y()),
            y: 2. * (q.w() * q.x() + q.y() * q.z()),
            z: 2. * (0.5 - q.x() * q.x() - q.y() * q.y()),
        } - a_hat;
        #[rustfmt::skip]
        let fb = F32x3 {
            x: 2. * bx * (0.5 - q.y() * q.y() - q.z() * q.z()) + 2. * bz * (q.x() * q.z() - q.w() * q.y()),
            y: 2. * bx * (q.x() * q.y() - q.w() * q.z()) + 2. * bz * (q.w() * q.x() + q.y() * q.z()),
            z: 2. * bx * (q.w() * q.y() + q.x() * q.z()) + 2. * bz * (0.5 - q.x() * q.x() - q.y() * q.y()),
        } - m_hat;

        // Jacobian
        #[rustfmt::skip]
        let jg = (
            (-2. * q.y(), 2. * q.z(), -2. * q.w(), 2. * q.x()),
            (2. * q.x(), 2. * q.w(), 2. * q.z(), 2. * q.y()),
            (0., -4. * q.x(), -4. * q.y(), 0.)
        );
        #[rustfmt::skip]
        let jb = (
            (-2. * bx * q.y(), 2. * bz * q.z(), -4. * bx * q.y() - 2. * bz * q.w(), -4. * bx * q.z() + 2. * bz * q.x()),
            (-2. * bx * q.z() + 2. * bz * q.x(), 2. * bx * q.y() + 2. * bz * q.w(), 2. * bx * q.x() + 2. * bz * q.z(), -2. * bx * q.w() + 2. * bz * q.y()),
            (2. * bx * q.y(), 2. * bx * q.z() - 4. * bz * q.x(), 2. * bx * q.w() - 4. * bz * q.y(), 2. * bx * q.x())
        );

        // Objective function gradient (J.T * f)
        let gradient = Quaternion::new(
            jg.0 .0 * fg.x
                + jg.1 .0 * fg.y
                + jg.2 .0 * fg.z
                + jb.0 .0 * fb.x
                + jb.1 .0 * fb.y
                + jb.2 .0 * fb.z,
            jg.0 .1 * fg.x
                + jg.1 .1 * fg.y
                + jg.2 .1 * fg.z
                + jb.0 .1 * fb.x
                + jb.1 .1 * fb.y
                + jb.2 .1 * fb.z,
            jg.0 .2 * fg.x
                + jg.1 .2 * fg.y
                + jg.2 .2 * fg.z
                + jb.0 .2 * fb.x
                + jb.1 .2 * fb.y
                + jb.2 .2 * fb.z,
            jg.0 .3 * fg.x
                + jg.1 .3 * fg.y
                + jg.2 .3 * fg.z
                + jb.0 .3 * fb.x
                + jb.1 .3 * fb.y
                + jb.2 .3 * fb.z,
        );
        if let Some(gradient_hat) = try_normalize_quat(gradient) {
            q_dot -= gradient_hat.scale(BETA_MARG);
        }
    }

    let qnew = q + q_dot.scale(delta_t);
    if let Some(qnew_hat) = try_normalize_quat(qnew) {
        qnew_hat
    } else {
        q
    }
}

pub fn dead_reckon_estimate(
    last_velocity: F32x3,
    last_displacement: F32x3,
    orientation: Quaternion,
    accel: F32x3,
    delta_t: f32,
) -> (F32x3, F32x3) {
    // Rotate acceleration vector by our orientation
    let accel = orientation.rotate(accel);

    // Subtract gravity
    let accel = accel - orientation.rotate(F32x3::from((0., 0., G_TO_MPS2)));

    // Integrate accelerometer measurement
    let new_velocity = last_velocity + accel * delta_t;
    let avg_velocity = (last_velocity + new_velocity) * 0.5;
    let new_displacement = last_displacement + avg_velocity * delta_t;

    (new_velocity, new_displacement)
}

#[cfg(test)]
mod tests {
    use core::f32::consts::PI;

    use super::*;
    use assert_float_eq::*;

    fn madgwick_fusion_6_matches_expected(
        qlast: Quaternion,
        accel: F32x3,
        gyro: F32x3,
        delta_t: f32,
        expected: Quaternion,
    ) {
        let actual = madgwick_fusion_6(qlast, accel, gyro, delta_t);
        assert_float_absolute_eq!(actual.w(), expected.w(), 0.001);
        assert_float_absolute_eq!(actual.x(), expected.x(), 0.001);
        assert_float_absolute_eq!(actual.y(), expected.y(), 0.001);
        assert_float_absolute_eq!(actual.z(), expected.z(), 0.001);
    }

    #[test]
    fn madgwick_fusion_6_works() {
        // NOTE: These values are generated against the python `ahrs` library. To generate, run (from the repo root):
        //       python -m tools.gen_test_values
        madgwick_fusion_6_matches_expected(
            Quaternion::new(
                0.22851365164234216,
                -0.7784839644560404,
                -0.36871586051277133,
                -0.45364395984540035,
            ),
            F32x3::from((0.4729424283280248, 0.3533989748458226, 0.7843591354096908)),
            F32x3::from((-4.130611673705839, -0.7807818031472955, -4.702027805619297)),
            0.01,
            Quaternion::new(
                0.2002324762749464,
                -0.7758507395147014,
                -0.3783513039692044,
                -0.4634791000941646,
            ),
        );

        madgwick_fusion_6_matches_expected(
            Quaternion::new(
                -0.4482127280968318,
                0.008531031466940495,
                -0.7542332855174314,
                -0.47975485707982823,
            ),
            F32x3::from((0.2997688755590464, 0.08988296120643335, -0.5591187559186066)),
            F32x3::from((0.8926568387590876, 3.0943045667782663, -4.93501240321939)),
            0.01,
            Quaternion::new(
                -0.4480066645710422,
                0.032465535695588216,
                -0.762951802954038,
                -0.46490919958752264,
            ),
        );

        madgwick_fusion_6_matches_expected(
            Quaternion::new(
                0.5810664407598567,
                0.3764712402182691,
                -0.3035291703311852,
                -0.6546000607006179,
            ),
            F32x3::from((
                0.9144261444135624,
                -0.32681090977474647,
                -0.8145083132397042,
            )),
            F32x3::from((-4.03283623166536, 3.4749436634745976, 1.0372603136689111)),
            0.01,
            Quaternion::new(
                0.5971602563518745,
                0.3743683825388972,
                -0.28231785694693723,
                -0.6508030193828714,
            ),
        );

        madgwick_fusion_6_matches_expected(
            Quaternion::new(
                0.5033838216652086,
                0.3765308335536987,
                0.05937791051822774,
                0.7754376333475025,
            ),
            F32x3::from((-0.24293124558329304, 0.104081262546454, 0.6588093285059897)),
            F32x3::from((1.1851975236424606, 3.6170690031077726, 0.7735214525676204)),
            0.01,
            Quaternion::new(
                0.49706503734627755,
                0.3654170277320329,
                0.07167057837238784,
                0.7837474546607458,
            ),
        );

        madgwick_fusion_6_matches_expected(
            Quaternion::new(
                0.3379059785855432,
                -0.7501944496860815,
                -0.449449939649466,
                -0.3478830105731456,
            ),
            F32x3::from((-0.840416046152745, -0.5344182272779396, -0.7979971411805418)),
            F32x3::from((-2.220263968899079, 1.356844442644002, -1.3516782102991574)),
            0.01,
            Quaternion::new(
                0.3302050595325292,
                -0.7485793464062754,
                -0.44831175059101874,
                -0.3600141039946748,
            ),
        );
    }

    fn madgwick_fusion_9_matches_expected(
        qlast: Quaternion,
        accel: F32x3,
        gyro: F32x3,
        mag: F32x3,
        delta_t: f32,
        expected: Quaternion,
    ) {
        let actual = madgwick_fusion_9(qlast, accel, gyro, mag, delta_t);
        assert_float_absolute_eq!(actual.w(), expected.w(), 0.001);
        assert_float_absolute_eq!(actual.x(), expected.x(), 0.001);
        assert_float_absolute_eq!(actual.y(), expected.y(), 0.001);
        assert_float_absolute_eq!(actual.z(), expected.z(), 0.001);
    }

    #[test]
    fn madgwick_fusion_9_works() {
        madgwick_fusion_9_matches_expected(
            Quaternion::new(
                -0.2206333990196314,
                -0.493706121273431,
                -0.3960318762700287,
                0.742114493883679,
            ),
            F32x3::from((0.2960707704931871, 0.21826201133397638, -0.657722703603806)),
            F32x3::from((2.291267979503492, -3.3659750623807163, -1.205445582423522)),
            F32x3::from((29.371401038195714, 8.399985591245574, 3.416984626478772)),
            0.01,
            Quaternion::new(
                -0.2171556051352368,
                -0.48153435234228414,
                -0.3868085294816959,
                0.7558751697157765,
            ),
        );

        madgwick_fusion_9_matches_expected(
            Quaternion::new(
                0.33637427377380047,
                0.6246893999105488,
                0.5028824660621306,
                -0.4936848457364443,
            ),
            F32x3::from((
                -0.9357995121919245,
                -0.36909390388183616,
                -0.46451824804859454,
            )),
            F32x3::from((-2.890171564136735, 4.429097143350544, 3.763676264726689)),
            F32x3::from((-11.119327152091326, 9.3263199176928, -6.262085936360144)),
            0.01,
            Quaternion::new(
                0.3430602731602509,
                0.6398492822245773,
                0.5056799948378802,
                -0.4660367880722173,
            ),
        );

        madgwick_fusion_9_matches_expected(
            Quaternion::new(
                0.7658159478618358,
                -0.07601517483978708,
                -0.434349451331042,
                -0.46806856476693354,
            ),
            F32x3::from((
                0.12273626832630158,
                -0.47451678295412947,
                0.16917198044708104,
            )),
            F32x3::from((3.978228836024769, -1.0059949485960273, -2.8067924084271665)),
            F32x3::from((29.852256389706618, 0.5715776205878704, -24.545435269572366)),
            0.01,
            Quaternion::new(
                0.7584916125106851,
                -0.057135441936455526,
                -0.44837243893242573,
                -0.4694551853281148,
            ),
        );

        madgwick_fusion_9_matches_expected(
            Quaternion::new(
                -0.6684454556700794,
                -0.5761485970676807,
                0.18810732558384208,
                0.4311021931661422,
            ),
            F32x3::from((
                -0.15568006640063192,
                -0.8729445876960857,
                -0.23676142698692648,
            )),
            F32x3::from((4.961213802400968, 0.2911434509913704, 4.710783776136182)),
            F32x3::from((21.64678213406988, -29.311138683430823, 13.243309161611677)),
            0.01,
            Quaternion::new(
                -0.6640939577472688,
                -0.5887921826067681,
                0.2112016324278228,
                0.40950806029252973,
            ),
        );

        madgwick_fusion_9_matches_expected(
            Quaternion::new(
                0.5513310454537662,
                0.11217241494937058,
                -0.7074803297446763,
                0.4276949972441193,
            ),
            F32x3::from((
                -0.7768956528082471,
                -0.13046949866179003,
                -0.09255258734158711,
            )),
            F32x3::from((4.538159275210802, 3.7585294037819406, -2.3661094924890924)),
            F32x3::from((0.03516678301789966, -19.280887168192116, 24.757670360689232)),
            0.01,
            Quaternion::new(
                0.5668022341577814,
                0.12498322475494114,
                -0.6857061525323603,
                0.43922829287612003,
            ),
        );
    }

    fn check_f32x3_near(expected: F32x3, actual: F32x3) {
        assert_float_absolute_eq!(expected.x, actual.x, 0.002);
        assert_float_absolute_eq!(expected.y, actual.y, 0.002);
        assert_float_absolute_eq!(expected.z, actual.z, 0.002);
    }

    #[test]
    fn dead_reckon_estimate_works() {
        // Start from 0,0,..,0
        let mut orientation = Quaternion::default();
        let mut velocity = F32x3::default();
        let mut displacement = F32x3::default();
        let mut accel = F32x3::default();

        // Detects free fall
        (velocity, displacement) =
            dead_reckon_estimate(velocity, displacement, orientation, accel, 1.);
        check_f32x3_near(F32x3::from((0., 0., G_TO_MPS2 * -1.)), velocity);
        check_f32x3_near(F32x3::from((0., 0., G_TO_MPS2 * -0.5)), displacement);

        // Detects recovery
        accel = F32x3::from((0., 0., G_TO_MPS2 * 2.));
        (velocity, displacement) =
            dead_reckon_estimate(velocity, displacement, orientation, accel, 1.);
        check_f32x3_near(F32x3::default(), velocity);
        check_f32x3_near(F32x3::from((0., 0., G_TO_MPS2 * -1.)), displacement);

        // Rotate to face the left, maintain altitude, accelerate forward for half of second
        orientation = Quaternion::axis_angle((0., 0., 1.).into(), PI / 2.);
        accel = F32x3::from((2., 0., G_TO_MPS2));
        (velocity, displacement) =
            dead_reckon_estimate(velocity, displacement, orientation, accel, 0.5);
        check_f32x3_near(F32x3::from((0., 1., 0.)), velocity);
        check_f32x3_near(F32x3::from((0., 0.25, G_TO_MPS2 * -1.)), displacement);
    }
}
