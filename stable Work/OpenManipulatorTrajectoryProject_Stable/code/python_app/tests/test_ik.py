import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import config
from kinematics import JointAngles, forward_kinematics_from_joints, inverse_kinematics


def _print_result(label, target, result):
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
    print(f"dX={result.dx:.3f}, dY={result.dy:.3f}, dZ={result.dz:.3f}, total error={result.position_error:.3f}")


def test_work_tip_ik_roundtrip():
    target = config.WORK_XYZ_MM
    result = inverse_kinematics(*target)
    _print_result("Work-tip IK round trip", target, result)
    assert result.position_error <= 0.05


def test_fwd_ik_roundtrip():
    target = (296.033, -0.436, 193.958)
    result = inverse_kinematics(*target)
    _print_result("Measured forward IK round trip", target, result)
    assert result.position_error <= 0.05


def test_table_reach_ik_roundtrip():
    target = (168.299, 10.806, 45.000)
    result = inverse_kinematics(*target)
    _print_result("Measured left IK round trip", target, result)
    assert result.position_error <= 0.05


def test_rotated_base_fk_ik_roundtrip():
    reference = JointAngles(35.0, 10.0, -15.0, 70.0)
    target_xyz = forward_kinematics_from_joints(reference)
    result = inverse_kinematics(target_xyz.x, target_xyz.y, target_xyz.z, reference=reference)
    assert result.position_error <= 0.05
    assert abs(result.joint_angles.theta1 - reference.theta1) <= 0.05
