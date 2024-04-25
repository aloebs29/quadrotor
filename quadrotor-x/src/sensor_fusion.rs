use micromath::vector::{F32x3, Vector};
use micromath::{Quaternion, F32};

const BETA: f32 = 0.041;

fn try_normalize_vec3(v: F32x3) -> Option<F32x3> {
    let magnitude = v.magnitude();
    if magnitude.is_normal() {
        let n: f32 = F32(magnitude).inv().into();
        Some(v * n)
    } else {
        None
    }
}

fn try_normalize_quat(q: Quaternion) -> Option<Quaternion> {
    let norm = q.norm();
    if norm == 0.0 {
        None
    } else {
        let n = F32(norm).invsqrt();
        Some(q.scale(n))
    }
}

pub fn madgwick_fusion_6(
    qlast: Quaternion,
    accel: F32x3,
    gyro: F32x3,
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

    // normalize accel
    let accel = try_normalize_vec3(accel)?;

    let qw_2 = 2.0 * qlast.w();
    let qx_2 = 2.0 * qlast.x();
    let qy_2 = 2.0 * qlast.y();
    let qz_2 = 2.0 * qlast.z();
    let qw_4 = 4.0 * qlast.w();
    let qx_4 = 4.0 * qlast.x();
    let qy_4 = 4.0 * qlast.y();
    let qx_8 = 8.0 * qlast.x();
    let qy_8 = 8.0 * qlast.y();
    let qw_qw = qlast.w() * qlast.w();
    let qx_qx = qlast.x() * qlast.x();
    let qy_qy = qlast.y() * qlast.y();
    let qz_qz = qlast.z() * qlast.z();

    // gradient decent algorithm corrective step
    let step = Quaternion::new(
        qx_4 * qz_qz - qz_2 * accel.x + 4.0 * qw_qw * qlast.x() - qw_2 * accel.y - qx_4
            + qx_8 * qx_qx
            + qx_8 * qy_qy
            + qx_4 * accel.z,
        4.0 * qw_qw * qlast.y() + qw_2 * accel.x + qy_4 * qz_qz - qz_2 * accel.y - qy_4
            + qy_8 * qx_qx
            + qy_8 * qy_qy
            + qy_4 * accel.z,
        4.0 * qx_qx * qlast.z() - qx_2 * accel.x + 4.0 * qy_qy * qlast.z() - qy_2 * accel.y,
        qw_4 * qy_qy + qy_2 * accel.x + qw_4 * qx_qx - qx_2 * accel.y,
    );
    let step = try_normalize_quat(step)?;

    // apply feedback step
    q_dot -= BETA * step;

    // integrate rate of change of quaternion to yield quaternion
    try_normalize_quat(qlast + q_dot.scale(time_diff_ms as f32 * 0.001))
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
    q_dot -= BETA * step;

    // integrate rate of change of quaternion to yield quaternion
    try_normalize_quat(qlast + q_dot.scale(time_diff_ms as f32 * 0.001))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn madgwick_fusion_9_handles_normalization_failures() {
        let q = Quaternion::IDENTITY;

        // Accel is zero
        assert!(madgwick_fusion_9(
            q,
            F32x3 {
                x: 0.,
                y: 0.,
                z: 0.
            },
            F32x3::default(),
            F32x3 {
                x: 1.,
                y: 0.,
                z: 0.
            },
            10
        )
        .is_none());

        // Mag is zero
        assert!(madgwick_fusion_9(
            q,
            F32x3 {
                x: 0.,
                y: 0.,
                z: -1.
            },
            F32x3::default(),
            F32x3 {
                x: 0.,
                y: 0.,
                z: 0.
            },
            10
        )
        .is_none());

        // Gradient descent step is zero
        assert!(madgwick_fusion_9(
            q,
            F32x3 {
                x: 0.,
                y: 0.,
                z: -1.
            },
            F32x3::default(),
            F32x3 {
                x: 1.,
                y: 0.,
                z: 0.
            },
            10
        )
        .is_none());

        // Normal case (no normalization failure)
        assert!(madgwick_fusion_9(
            q,
            F32x3 {
                x: 0.,
                y: 0.1,
                z: -9.
            },
            F32x3 {
                x: 0.01,
                y: 0.01,
                z: 0.01
            },
            F32x3 {
                x: 1.,
                y: 0.,
                z: 0.
            },
            10
        )
        .is_some());
    }
}
