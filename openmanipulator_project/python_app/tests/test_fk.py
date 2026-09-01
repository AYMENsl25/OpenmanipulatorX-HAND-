import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kinematics import MotorAngles, forward_kinematics


def test_home_fk():
    motors = MotorAngles(351.0, 1.0, 1.0, 90.0)
    xyz = forward_kinematics(motors)
    expected = (143.523, 0.0, 208.985)
    error = math.sqrt((xyz.x - expected[0]) ** 2 + (xyz.y - expected[1]) ** 2 + (xyz.z - expected[2]) ** 2)
    print(f"Home FK actual: X={xyz.x:.3f}, Y={xyz.y:.3f}, Z={xyz.z:.3f}, error={error:.3f} mm")
    assert error < 0.01
