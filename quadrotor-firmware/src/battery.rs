use embassy_nrf::saadc::Saadc;

const VBAT_DIVIDER: f32 = 568.75;

pub struct BatteryReader {
    adc: Saadc<'static, 1>,
}

impl BatteryReader {
    pub fn new(adc: Saadc<'static, 1>) -> Self {
        Self { adc }
    }

    pub async fn read(self: &mut Self) -> f32 {
        let mut adc_msmt_buf = [0i16; 1];
        self.adc.sample(&mut adc_msmt_buf).await;

        adc_msmt_buf[0] as f32 / VBAT_DIVIDER
    }
}