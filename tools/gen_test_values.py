from dataclasses import dataclass
import random
import typing

from ahrs.filters import Madgwick
import numpy as np

from .utils import Vec3, Quaternion


@dataclass
class TestInput:
    qlast: Quaternion
    accel: Vec3
    gyro: Vec3
    mag: typing.Optional[Vec3] = None
    time_diff_ms: int = 10


def make_random_vec3(low, hi):
    return Vec3(*[random.uniform(low, hi) for _ in range(3)])


def make_random_input(include_mag=False):
    qlast = Quaternion(*[random.uniform(-1, 1) for _ in range(4)]).normalize()
    accel = make_random_vec3(-1 ,1)
    gyro = make_random_vec3(-5, 5)

    if include_mag:
        mag = make_random_vec3(-30, 30)
        return TestInput(qlast, accel, gyro, mag)
    else:
        return TestInput(qlast, accel, gyro)


if __name__ == "__main__":
    madgwick = Madgwick(gain=0.033)

    # Make outputs repeatable
    random.seed(42)

    # Madgwick fusion 6 test values
    print("***\nMadgwick fusion 6 test values\n***\n")
    for _ in range(5):
        input = make_random_input()

        madgwick.Dt = input.time_diff_ms * 0.001
        output = madgwick.updateIMU(
            input.qlast.as_array(),
            input.gyro.as_array(),
            input.accel.as_array())
        output = Quaternion(*output)
        
        print((f"madgwick_fusion_6_matches_expected(\n"
               f"   Quaternion::new({input.qlast.w}, {input.qlast.x}, {input.qlast.y}, {input.qlast.z}),\n"
               f"   F32x3::from(({input.accel.x}, {input.accel.y}, {input.accel.z})),\n"
               f"   F32x3::from(({input.gyro.x}, {input.gyro.y}, {input.gyro.z})),\n"
               f"   {input.time_diff_ms},\n"
               f"   Quaternion::new({output.w}, {output.x}, {output.y}, {output.z}),\n"
               f");\n"))
