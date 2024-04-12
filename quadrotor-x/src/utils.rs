use core::num::Wrapping;

use crate::lib::panic;

pub fn twos_complement(raw: u32, bits: usize) -> i32 {
    if bits > 32 {
        panic!("Bit size {} too large to calculate two's complement.", bits);
    }

    let top_bit = 1u32 << (bits - 1);
    if (raw & top_bit) == top_bit {
        (Wrapping(raw) - Wrapping(1u32 << bits)).0 as i32
    } else {
        raw as i32
    }
}

pub fn lipo_1s_charge_percent_from_voltage(voltage: f32) -> f32 {
    // Table mapping voltage to charge, starting at 100% and decreasing in 5% increments.
    const CHARGE_PERCENT_START: f32 = 100f32;
    const CHARGE_PERCENT_STEP: f32 = 5f32;
    const VOLTAGE_TO_CHARGE_TABLE: [f32; 21] = [
        4.2, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95, 3.91, 3.87, 3.85, 3.84, 3.82, 3.8, 3.79, 3.77,
        3.75, 3.73, 3.71, 3.69, 3.61, 3.27,
    ];

    // NOTE: We could do a binary search here, but the table is only 21 entries long so why bother..
    let mut right_percent = CHARGE_PERCENT_START;
    let mut left_voltage = VOLTAGE_TO_CHARGE_TABLE[0];
    let mut right_voltage = VOLTAGE_TO_CHARGE_TABLE[0];

    for entry in VOLTAGE_TO_CHARGE_TABLE.iter().skip(1) {
        left_voltage = right_voltage;

        right_percent -= CHARGE_PERCENT_STEP;
        right_voltage = *entry;

        if voltage >= right_voltage {
            break;
        }
    }

    // Lerp final value between the adjacent table entries
    let distance = ((voltage - right_voltage) / (left_voltage - right_voltage)).clamp(0.0, 1.0);
    return (CHARGE_PERCENT_STEP * distance) + right_percent;
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_float_eq::*;

    #[test]
    fn twos_complement_valid_inputs() {
        assert_eq!(twos_complement(5, 24), 5);

        assert_eq!(twos_complement(0xFFF, 12), -1);
        assert_eq!(twos_complement(0xFFF, 13), 0xFFF);

        assert_eq!(twos_complement(0xFFFFF - 41, 20), -42);
    }

    #[test]
    #[should_panic]
    fn twos_complement_too_many_bits() {
        twos_complement(1234, 33);
    }

    #[test]
    fn lipo_1s_charge_percent_from_voltage_handles_out_of_range() {
        assert_eq!(lipo_1s_charge_percent_from_voltage(55.0), 100.0);
        assert_eq!(lipo_1s_charge_percent_from_voltage(4.2), 100.0);
        assert_eq!(lipo_1s_charge_percent_from_voltage(3.27), 0.0);
        assert_eq!(lipo_1s_charge_percent_from_voltage(-42.0), 0.0);
    }

    #[test]
    fn lipo_1s_charge_percent_from_voltage_lerps_correctly() {
        assert_f32_near!(lipo_1s_charge_percent_from_voltage(4.175), 97.5); // halfway between
        assert_f32_near!(lipo_1s_charge_percent_from_voltage(4.035), 81.25); // 3/4-way between
        assert_f32_near!(lipo_1s_charge_percent_from_voltage(3.28), 0.1470587); // 33/34-way between
    }
}
