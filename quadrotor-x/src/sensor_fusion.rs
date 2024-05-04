use micromath::vector::{F32x3, Vector};
use micromath::{Quaternion, F32};

// NOTE: Only use micromath f32::sqrt function on target. Unit tests need to use more accurate sqrt function found in
//       std.
#[cfg(not(feature = "std"))]
use micromath::F32Ext;

const BETA_IMU: f32 = 0.031;
const BETA_MARG: f32 = 0.041;

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

pub fn madgwick_fusion_6(
    qlast: Quaternion,
    accel: F32x3,
    gyro: F32x3,
    time_diff_ms: u32,
) -> Quaternion {
    // Compute derivative using last estimate and gyro
    let mut q_dot = 0.5 * (qlast * Quaternion::new(0., gyro.x, gyro.y, gyro.z));

    // Normalize accel data
    if let Some(a_hat) = try_normalize_vec3(accel) {
        // Objective function
        let f = F32x3 {
            x: 2. * (qlast.x() * qlast.z() - qlast.w() * qlast.y()),
            y: 2. * (qlast.w() * qlast.x() + qlast.y() * qlast.z()),
            z: 2. * (0.5 - qlast.x() * qlast.x() - qlast.y() * qlast.y()),
        } - a_hat;

        if vec3_norm(f) > 0. {
            // Jacobian
            #[rustfmt::skip]
            let j = (
                (-2. * qlast.y(), 2. * qlast.z(), -2. * qlast.w(), 2. * qlast.x()),
                (2. * qlast.x(), 2. * qlast.w(), 2. * qlast.z(), 2. * qlast.y()),
                (0., -4. * qlast.x(), -4. * qlast.y(), 0.)
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

    let qnew = qlast + q_dot.scale(time_diff_ms as f32 * 0.001);
    if let Some(qnew_hat) = try_normalize_quat(qnew) {
        qnew_hat
    } else {
        qlast
    }
}

pub fn madgwick_fusion_9(
    qlast: Quaternion,
    accel: F32x3,
    gyro: F32x3,
    mag: F32x3,
    time_diff_ms: u32,
) -> Option<Quaternion> {
    // rate of change of quaternion from gyroscope
    let mut q_dot = Quaternion::new(
        qlast.w() * gyro.x + qlast.y() * gyro.z - qlast.z() * gyro.y,
        qlast.w() * gyro.y - qlast.x() * gyro.z + qlast.z() * gyro.x,
        qlast.w() * gyro.z + qlast.x() * gyro.y - qlast.y() * gyro.x,
        -qlast.x() * gyro.x - qlast.y() * gyro.y - qlast.z() * gyro.z,
    );
    q_dot *= 0.5;

    // normalize accel and mag
    let accel = try_normalize_vec3(accel)?;
    let mag = try_normalize_vec3(mag)?;

    // pre-compute repeated operands
    let qw_mx_2 = 2.0 * qlast.w() * mag.x;
    let qw_my_2 = 2.0 * qlast.w() * mag.y;
    let qw_mz_2 = 2.0 * qlast.w() * mag.z;
    let qx_mx_2 = 2.0 * qlast.x() * mag.x;
    let qw_2 = 2.0 * qlast.w();
    let qx_2 = 2.0 * qlast.x();
    let qy_2 = 2.0 * qlast.y();
    let qz_2 = 2.0 * qlast.z();
    let qw_qy_2 = 2.0 * qlast.w() * qlast.y();
    let qy_qz_2 = 2.0 * qlast.y() * qlast.z();
    let qw_qw = qlast.w() * qlast.w();
    let qw_qx = qlast.w() * qlast.x();
    let qw_qy = qlast.w() * qlast.y();
    let qw_qz = qlast.w() * qlast.z();
    let qx_qx = qlast.x() * qlast.x();
    let qx_qy = qlast.x() * qlast.y();
    let qx_qz = qlast.x() * qlast.z();
    let qy_qy = qlast.y() * qlast.y();
    let qy_qz = qlast.y() * qlast.z();
    let qz_qz = qlast.z() * qlast.z();

    // reference direction of Earth's magetic field
    let hx = mag.x * qw_qw - qw_my_2 * qlast.z()
        + qw_mz_2 * qlast.y()
        + mag.x * qx_qx
        + qx_2 * mag.y * qlast.y()
        + qx_2 * mag.z * qlast.z()
        - mag.x * qy_qy
        - mag.x * qz_qz;
    let hy = qw_mx_2 * qlast.z() + mag.y * qw_qw - qw_mz_2 * qlast.x() + qx_mx_2 * qlast.y()
        - mag.y * qx_qx
        + mag.y * qy_qy
        + qy_2 * mag.z * qlast.z()
        - mag.y * qz_qz;
    let bx_2: f32 = F32::from(hx * hx + hy * hy).sqrt().into();
    let bz_2 = -qw_mx_2 * qlast.y() + qw_my_2 * qlast.x() + mag.z * qw_qw + qx_mx_2 * qlast.z()
        - mag.z * qx_qx
        + qy_2 * mag.y * qlast.z()
        - mag.z * qy_qy
        + mag.z * qz_qz;
    let bx_4 = 2.0 * bx_2;
    let bz_4 = 2.0 * bz_2;

    // gradient descent algorithm corrective step
    let sw = -qy_2 * (2.0 * qx_qz - qw_qy_2 - accel.x) + qx_2 * (2.0 * qw_qx + qy_qz_2 - accel.y)
        - bz_2 * qlast.y() * (bx_2 * (0.5 - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - mag.x)
        + (-bx_2 * qlast.z() + bz_2 * qlast.x())
            * (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - mag.y)
        + bx_2 * qlast.y() * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5 - qx_qx - qy_qy) - mag.z);
    let sx = qz_2 * (2.0 * qx_qz - qw_qy_2 - accel.x) + qw_2 * (2.0 * qw_qx + qy_qz_2 - accel.y)
        - 4.0 * qlast.x() * (1.0 - 2.0 * qx_qx - 2.0 * qy_qy - accel.z)
        + bz_2 * qlast.z() * (bx_2 * (0.5 - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - mag.x)
        + (bx_2 * qlast.y() + bz_2 * qlast.w())
            * (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - mag.y)
        + (bx_2 * qlast.z() - bz_4 * qlast.x())
            * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5 - qx_qx - qy_qy) - mag.z);
    let sy = -qw_2 * (2.0 * qx_qz - qw_qy_2 - accel.x) + qz_2 * (2.0 * qw_qx + qy_qz_2 - accel.y)
        - 4.0 * qlast.y() * (1.0 - 2.0 * qx_qx - 2.0 * qy_qy - accel.z)
        + (-bx_4 * qlast.y() - bz_2 * qlast.w())
            * (bx_2 * (0.5 - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - mag.x)
        + (bx_2 * qlast.x() + bz_2 * qlast.z())
            * (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - mag.y)
        + (bx_2 * qlast.w() - bz_4 * qlast.y())
            * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5 - qx_qx - qy_qy) - mag.z);
    let sz = qx_2 * (2.0 * qx_qz - qw_qy_2 - accel.x)
        + qy_2 * (2.0 * qw_qx + qy_qz_2 - accel.y)
        + (-bx_4 * qlast.z() + bz_2 * qlast.x())
            * (bx_2 * (0.5 - qy_qy - qz_qz) + bz_2 * (qx_qz - qw_qy) - mag.x)
        + (-bx_2 * qlast.w() + bz_2 * qlast.y())
            * (bx_2 * (qx_qy - qw_qz) + bz_2 * (qw_qx + qy_qz) - mag.y)
        + bx_2 * qlast.x() * (bx_2 * (qw_qy + qx_qz) + bz_2 * (0.5 - qx_qx - qy_qy) - mag.z);
    // println!("{:?} {:?} {:?} {:?}", sw, sx, sy, sz);

    let step = try_normalize_quat(Quaternion::new(sx, sy, sz, sw))?;

    // apply feedback step
    q_dot -= BETA_MARG * step;

    // integrate rate of change of quaternion to yield quaternion
    try_normalize_quat(qlast + q_dot.scale(time_diff_ms as f32 * 0.001))
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_float_eq::*;

    fn madgwick_fusion_6_matches_expected(
        qlast: Quaternion,
        accel: F32x3,
        gyro: F32x3,
        time_diff_ms: u32,
        expected: Quaternion,
    ) {
        let actual = madgwick_fusion_6(qlast, accel, gyro, time_diff_ms);
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
            10,
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
            10,
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
            10,
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
            10,
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
            10,
            Quaternion::new(
                0.3302050595325292,
                -0.7485793464062754,
                -0.44831175059101874,
                -0.3600141039946748,
            ),
        );
    }
}
