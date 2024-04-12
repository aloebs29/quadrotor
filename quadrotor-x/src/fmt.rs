use micromath::vector::F32x3;

pub struct F32x3Fmt(F32x3);

impl defmt::Format for F32x3Fmt {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "({}, {}, {})", self.0.x, self.0.y, self.0.z);
    }
}

impl From<F32x3> for F32x3Fmt {
    fn from(f: F32x3) -> Self {
        Self(f)
    }
}
