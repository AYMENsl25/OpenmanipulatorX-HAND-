"""Dependency-free calibration and FK/IK checks.  This never opens a serial port."""

from __future__ import annotations

import math
import random

from kinematics_calibration import (
    CALIBRATIONS,
    CalibrationError,
    Joints,
    calibration_ready,
    forward_kinematics,
    inverse_kinematics,
    joint_to_raw,
    raw_to_joint,
)


def main() -> None:
    print("CALIBRATION STATUS")
    for motor_id, cal in CALIBRATIONS.items():
        print(f"ID{motor_id}: complete={cal.complete}, provisional={cal.provisional}, raw=[{cal.raw_min}, {cal.raw_home}, {cal.raw_max}]")
    assert not calibration_ready(), "Motion must remain locked until J2-J4 and provisional J3 are finalized"

    print("\nJ1 PIECEWISE CALIBRATION")
    for raw, expected in ((915, -90.0), (1941, 0.0), (2961, 90.0)):
        q = raw_to_joint(11, raw)
        rebuilt = joint_to_raw(11, q)
        print(f"RAW {raw} -> q1 {q:.6f} deg -> RAW {rebuilt}")
        assert abs(q - expected) < 1e-9 and rebuilt == raw

    try:
        raw_to_joint(12, 2040)
        raise AssertionError("Pending ID12 calibration did not block conversion")
    except CalibrationError as exc:
        print(f"Pending calibration correctly blocked: {exc}")

    print("\nKNOWN FK POSES (official zero convention; physical tape/fixture validation still required)")
    known = [Joints(0, 0, 0, 0), Joints(-45, 20, -30, 10), Joints(45, -20, 30, -10)]
    for joints in known:
        pose = forward_kinematics(joints)
        print(f"q={joints.as_tuple()} -> X={pose.x:.3f} Y={pose.y:.3f} Z={pose.z:.3f} pitch={pose.tool_pitch_deg:.3f}")

    print("\nFK -> IK -> FK RANDOM ROUND TRIPS")
    rng = random.Random(20260907)
    passed = 0
    for _ in range(250):
        joints = Joints(rng.uniform(-80, 80), rng.uniform(-80, 70), rng.uniform(-75, 75), rng.uniform(-80, 80))
        pose = forward_kinematics(joints)
        result = inverse_kinematics(pose.x, pose.y, pose.z, pose.tool_pitch_deg, reference=joints)
        if not result.success:
            continue
        assert result.error_mm is not None and result.error_mm < 1e-6
        assert result.reconstructed is not None
        assert math.isclose(result.reconstructed.tool_pitch_deg, pose.tool_pitch_deg, abs_tol=1e-9)
        passed += 1
    assert passed >= 100
    print(f"{passed} reachable random poses reconstructed below 1e-6 mm")
    print("\nOFFLINE MATHEMATICS PASS; PHYSICAL CALIBRATION/VALIDATION NOT YET PASSED")


if __name__ == "__main__":
    main()

