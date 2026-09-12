"""Dependency-free test runner for the project FK/IK checks."""

from __future__ import annotations

import math

from kinematics import MotorAngles, degrees_to_raw, forward_kinematics, inverse_kinematics, raw_to_degrees


def check_home_fk() -> None:
    motors = MotorAngles(351.0, 1.0, 1.0, 90.0)
    xyz = forward_kinematics(motors)
    expected = (156.481, 0.0, 323.589)
    error = math.sqrt((xyz.x - expected[0]) ** 2 + (xyz.y - expected[1]) ** 2 + (xyz.z - expected[2]) ** 2)
    print("HOME FK")
    print(f"actual: X={xyz.x:.3f}, Y={xyz.y:.3f}, Z={xyz.z:.3f}")
    print(f"expected: X={expected[0]:.3f}, Y={expected[1]:.3f}, Z={expected[2]:.3f}")
    print(f"error: {error:.6f} mm")
    assert error < 0.01


def check_ik_round_trip(label: str, target: tuple[float, float, float]) -> None:
    result = inverse_kinematics(*target)
    print(label)
    print(f"Target XYZ: X={target[0]:.3f}, Y={target[1]:.3f}, Z={target[2]:.3f}")
    print(
        "IK angles: "
        f"theta1={result.joint_angles.theta1:.3f}, theta2={result.joint_angles.theta2:.3f}, "
        f"theta3={result.joint_angles.theta3:.3f}, theta4={result.joint_angles.theta4:.3f}"
    )
    print(
        "Motor angles: "
        f"ID11={result.motor_angles.id11:.3f}, ID12={result.motor_angles.id12:.3f}, "
        f"ID13={result.motor_angles.id13:.3f}, ID14={result.motor_angles.id14:.3f}"
    )
    print(
        "FK reconstructed XYZ: "
        f"X={result.xyz_check.x:.3f}, Y={result.xyz_check.y:.3f}, Z={result.xyz_check.z:.3f}"
    )
    print(f"dX={result.dx:.6f}, dY={result.dy:.6f}, dZ={result.dz:.6f}")
    print(f"total error={result.position_error:.6f} mm")
    assert result.position_error <= 0.05


def check_conversion_layer() -> None:
    print("RAW/DEGREE CONVERSION")
    print(f"raw 1024 -> {raw_to_degrees(1024):.3f} deg")
    print(f"90 deg -> {degrees_to_raw(90.0)} raw")
    assert abs(raw_to_degrees(1024) - 90.0) < 1e-9
    assert degrees_to_raw(90.0) == 1024


if __name__ == "__main__":
    check_home_fk()
    print()
    check_ik_round_trip("HOME IK ROUND TRIP", (156.481, 0.0, 323.589))
    print()
    check_ik_round_trip("HORIZONTAL FORWARD IK ROUND TRIP", (380.231, 0.0, 76.501))
    print()
    check_ik_round_trip("TABLE REACH IK ROUND TRIP", (167.284, 230.246, 310.779))
    print()
    check_conversion_layer()
    print()
    print("All checks passed.")
