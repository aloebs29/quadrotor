use crate::datatypes::Vec3f;
use crate::sensor_fusion::G_TO_MPS2;

const GRAVITY: Vec3f = Vec3f::new(0.0, 0.0, G_TO_MPS2);

#[derive(Copy, Clone)]
pub struct AccelOffsets {
    inner: Vec3f,
}

/// Calculates accelerometer offsets based on the average measurement over a fixed number of samples.
///
/// Expects the accelerometer to be upright and stationary for the duration of sampling.
#[derive(Copy, Clone)]
pub struct AccelOffsetsBuilder {
    current_sample_count: usize,
    target_sample_count: usize,
    accumulator: Vec3f,
}

#[derive(Copy, Clone)]
pub enum AccelOffsetState {
    Ready(AccelOffsets),
    InProgress(AccelOffsetsBuilder),
}

impl AccelOffsets {
    pub fn apply(&self, raw_accel: Vec3f) -> Vec3f {
        raw_accel - self.inner
    }
}

impl AccelOffsetsBuilder {
    pub fn new(target_sample_count: usize) -> Self {
        AccelOffsetsBuilder {
            current_sample_count: 0,
            target_sample_count,
            accumulator: Vec3f::zeroed(),
        }
    }

    pub fn update(mut self, sample: Vec3f) -> AccelOffsetState {
        self.accumulator = self.accumulator + sample / self.target_sample_count as f32;
        self.current_sample_count += 1;

        if self.current_sample_count == self.target_sample_count {
            AccelOffsetState::Ready(AccelOffsets {
                inner: self.accumulator - GRAVITY,
            })
        } else {
            AccelOffsetState::InProgress(self)
        }
    }
}
