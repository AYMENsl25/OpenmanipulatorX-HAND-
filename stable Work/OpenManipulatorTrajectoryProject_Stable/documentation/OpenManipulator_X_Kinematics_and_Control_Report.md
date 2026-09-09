# OpenMANIPULATOR X Kinematics and Control Report

## Purpose

This report records the working calibration, forward kinematics, inverse kinematics, limits, named poses, serial control behavior, and trajectory workflow used by the OpenMANIPULATOR X project snapshot. It describes the implementation identified by kinematics version `gripper-tip-v4-rotating-base-offset`.

The robot is a remounted OpenMANIPULATOR X. Therefore, the encoder references in this report are project calibration values. They are not universal ROBOTIS motor-zero values.

## Coordinate frame and TCP

All position values are in millimetres and all joint values shown in equations are in radians unless the unit is explicitly shown as degrees.

- The base frame origin is the robot model base reference.
- +X points forward from the base when q1 = 0.
- +Y is created by positive q1 rotation around the base vertical axis.
- +Z points upward from the base frame.
- The working TCP is the gripper tip/contact point used by the experiment.
- The official ROBOTIS gripper-frame origin is 126.0 mm from joint 4.
- The experiment TCP extends 39.7 mm beyond that official frame. The working TCP distance from joint 4 is therefore 165.7 mm.

The mathematical Z=0 plane is a robot-base model plane. It is not automatically the physical table surface. For that reason, the application does not impose a Cartesian Z box limit.

## Geometry measurements

| Symbol | Value | Meaning |
|---|---:|---|
| Z_BASE | 17.0 mm | Base vertical offset |
| L1_Z | 59.5 mm | Joint 2 vertical offset |
| Z0 = Z_BASE + L1_Z | 76.5 mm | Fixed vertical offset before the planar arm chain |
| BASE_X | 12.0 mm | Joint 1 radial offset. It rotates with q1. |
| L2_X | 24.0 mm | Joint 2 to joint 3 local X offset |
| L2_Z | 128.0 mm | Joint 2 to joint 3 local Z offset |
| L2 | 130.230565 mm | Combined length sqrt(L2_X^2 + L2_Z^2) |
| ALPHA2_0 | 79.380345 deg | atan2(L2_Z, L2_X) |
| L3_X | 124.0 mm | Joint 3 to joint 4 length |
| Official gripper frame | 126.0 mm | Joint 4 to ROBOTIS gripper frame |
| Tip extension | 39.7 mm | Gripper frame to experimental contact point |
| L4_X | 165.7 mm | Joint 4 to working TCP |

## Motor encoder conversion and calibrated joint angles

The DYNAMIXEL encoder scale is 4096 counts per revolution.

$$
m_i = \left(\frac{\text{raw}_i}{4096} \times 360\right) \bmod 360
$$

The wrapped angle operator returns an angle in [-180, 180):

$$
\operatorname{wrap}(a) = (a + 180) \bmod 360 - 180
$$

For each arm motor i, the calibrated joint angle in degrees is:

$$
q_i = d_i\,\operatorname{wrap}(m_i - h_i)
$$

where h_i is the calibrated REST motor angle and d_i is the calibrated direction. The inverse conversion used to command motors is:

$$
m_i = \left(h_i + \frac{q_i}{d_i}\right) \bmod 360
$$

The commanded raw count is then:

$$
\text{raw}_i = \operatorname{round}\left(\frac{m_i}{360} \times 4096\right)
$$

### Calibrated REST reference

The straight raised REST pose defines q1 = q2 = q3 = q4 = 0.

| Motor | REST raw count | REST motor angle | Direction d_i |
|---|---:|---:|---:|
| ID11 | 1917 | 168.486328 deg | +1 |
| ID12 | 2046 | 179.824219 deg | +1 |
| ID13 | 4049 | 355.869141 deg | +1 |
| ID14 | 0 | 0.000000 deg | +1 |

The wrapped subtraction is essential. For example, a motor value slightly below 0 deg is represented close to 360 deg, but it is still correctly treated as a small motion around the calibrated reference.

## Named poses

| Pose | Raw counts ID11 ID12 ID13 ID14 | Calibrated q1 q2 q3 q4 | Working TCP XYZ |
|---|---|---|---|
| REST straight raised | 1917, 2046, 4049, 0 | 0, 0, 0, 0 deg | 325.700, 0.000, 204.500 mm |
| WORK tool down | 1917, 2046, 4049, 943 | 0, 0, 0, 82.880859 deg | 180.536, 0.000, 40.077 mm |

At REST, the official ROBOTIS gripper-frame origin is (286.000, 0.000, 204.500) mm. The working TCP is farther forward because it includes the 39.7 mm measured tip extension.

## Forward kinematics

Convert the four calibrated joint angles from degrees to radians: t1, t2, t3, and t4. The planar link directions are:

$$
\phi_2 = \alpha_{2,0} - t_2
$$

$$
\phi_3 = -(t_2 + t_3)
$$

$$
\phi_4 = -(t_2 + t_3 + t_4)
$$

The radial distance in the joint 1 plane is:

$$
r = \mathrm{BASE\_X} + L_2\cos(\phi_2) + L_{3X}\cos(\phi_3) + L_4\cos(\phi_4)
$$

The height is:

$$
z = Z_0 + L_2\sin(\phi_2) + L_{3X}\sin(\phi_3) + L_4\sin(\phi_4)
$$

The final Cartesian TCP position is:

$$
x = r\cos(t_1), \qquad y = r\sin(t_1), \qquad z = z
$$

The 12 mm base offset is included inside r, so it rotates with joint 1. This is required for FK and IK to use the same base geometry.

For official ROBOTIS gripper-frame FK, substitute L4 = 126.0 mm. For the experimental working-tip FK, use L4 = 165.7 mm.

## Inverse kinematics

The solver receives a target TCP position (x, y, z) and a reference joint pose. It uses the robot's current pose as the reference during single-point movement and previous IK point as the reference during trajectory playback.

First, resolve the base angle and planar target:

$$
t_1 = \operatorname{atan2}(y, x)
$$

$$
r_{target} = \sqrt{x^2+y^2} - \mathrm{BASE\_X}, \qquad z_{plane} = z-Z_0
$$

The arm has one redundant position degree of freedom. The solver tests a set of theta4 candidates, including the current theta4, common values, and a 5 degree sweep across the theta4 joint limit. For each theta4 candidate:

$$
A=L_{3X}+L_4\cos(t_4), \qquad B=-L_4\sin(t_4)
$$

$$
R_{eff}=\sqrt{A^2+B^2}, \qquad \psi_{eff}=\operatorname{atan2}(B,A)
$$

$$
D=\sqrt{r_{target}^2+z_{plane}^2}
$$

The candidate is geometrically possible only when:

$$
|L_2-R_{eff}| \leq D \leq L_2+R_{eff}
$$

For each elbow branch s in {+1,-1}:

$$
\alpha=\arccos\left(\frac{L_2^2+D^2-R_{eff}^2}{2L_2D}\right), \qquad \gamma=\operatorname{atan2}(z_{plane},r_{target})
$$

$$
\phi_2=\gamma+s\alpha, \qquad t_2=\alpha_{2,0}-\phi_2
$$

The remaining planar angle is reconstructed from the elbow-to-target vector:

$$
r_e=L_2\cos(\phi_2), \qquad z_e=L_2\sin(\phi_2)
$$

$$
\phi_{eff}=\operatorname{atan2}(z_{plane}-z_e,r_{target}-r_e)
$$

$$
\phi_3=\phi_{eff}-\psi_{eff}, \qquad t_3=-\phi_3-t_2
$$

Each candidate is converted to motor angles, checked against all limits, and reconstructed with FK. It is valid only when its Cartesian position error is at most 2.0 mm.

Among valid candidates, the chosen solution minimizes weighted squared wrapped joint motion from the reference pose:

$$
C=1.0\Delta q_1^2+1.5\Delta q_2^2+1.2\Delta q_3^2+0.8\Delta q_4^2
$$

where each delta q is wrapped to [-180,180). This gives the nearest valid configuration rather than selecting a fixed elbow posture.

## Limits and acceptance checks

### Cartesian envelope

| Axis | Allowed range |
|---|---|
| X | 0 to 350 mm |
| Y | -170 to 170 mm |
| Z | No Cartesian box limit |

### Calibrated joint limits

| Joint | Minimum | Maximum |
|---|---:|---:|
| q1 ID11 | -90 deg | +100 deg |
| q2 ID12 | -15 deg | +85 deg |
| q3 ID13 | -60 deg | +90 deg |
| q4 ID14 | -45 deg | +100 deg |

### Motor angle limits

Each commanded ID11-ID14 motor angle is normalized and limited to 0 through 360 deg.

### Motion and IK checks

- IK position tolerance: 2.0 mm.
- Python movement timeout: 30 seconds.
- Post-move motor readback tolerance: 2.0 deg.
- Firmware final target tolerance: 8 encoder counts, approximately 0.7 deg.
- Firmware final wait timeout: 3 seconds.
- Firmware interpolation: cosine S curve, 20 ms step interval, 120 to 4500 ms motion duration.

## OpenCR and serial configuration

| Setting | Value |
|---|---|
| Controller board | ROBOTIS OpenCR |
| PC serial port | COM7 during the validated setup |
| PC serial baud rate | 115200 |
| DYNAMIXEL bus | Serial3 |
| Direction pin | 84 |
| DYNAMIXEL bus baud rate | 1000000 |
| Protocol | 2.0 |
| Arm motor IDs | 11, 12, 13, 14 |
| Gripper motor ID | 15, not used in arm TCP FK |
| Teaching sample interval | 50 ms, 20 Hz |
| Maximum teaching duration | 30 seconds, 600 points |

The firmware controls DYNAMIXEL communication, torque, one-shot reads, interpolation, and teaching samples. Python owns the calibrated FK, IK, Excel export, GUI behavior, and trajectory replay validation.

## GUI movement and trajectory workflow

### Single XYZ command

1. Enter X, Y, and Z in millimetres.
2. Press PREVIEW NEAREST IK.
3. The GUI reads the current motor pose, calculates valid IK candidates, and displays the selected joints and delta joints.
4. Press MOVE TO XYZ only after reviewing the preview and clearing the workspace.
5. The GUI requests confirmation, sends motor targets, waits for DONE, and reads the motors back to verify target error.

### Teach and replay

Teaching and replay intentionally use different data paths:

$$
\text{Teaching: motor angles} \rightarrow \text{Python FK} \rightarrow \text{XYZ workbook}
$$

$$
\text{Replay: XYZ workbook} \rightarrow \text{Python IK} \rightarrow \text{new motor targets}
$$

The full workbook stores time, motor angles, calibrated joints, and XYZ for analysis. Replay reads only `trajectory_xyz.xlsx`; it does not replay stored joint-angle columns directly. This allows IK to choose a safe nearest solution from the robot's current posture.

If Excel has a normal trajectory workbook open, Windows can lock it. The application then saves a matched timestamped workbook pair instead of overwriting the open files.

## Verification recorded for this snapshot

The project contains offline checks for REST FK, WORK FK, motor-joint conversion, raw-degree conversion, FK to IK to FK round trips, rotated base geometry, trajectory endpoint preservation, workspace rejection, and post-move target verification. Python compilation also passed for the project application.

Hardware validation remains a separate requirement: upload the OpenCR sketch, close Arduino Serial Monitor before Python uses COM7, clear the workspace, support the arm when requested, and use STOP immediately if motion is unexpected.

## Snapshot contents

The stable snapshot contains:

- `code/python_app`: Python GUI, calibrated FK/IK, serial client, trajectory logic, and tests.
- `code/opencr_firmware`: OpenCR Arduino sketch for IDs 11 through 14.
- `code/README.md`: operating instructions.
- `code/docs`: project planning notes.
- `documentation/OpenManipulator_X_Kinematics_and_Control_Report.md`: this report.
- `documentation/OpenManipulator_X_Kinematics_and_Control_Report.docx`: Word version of this report.

No source code or equation was modified while creating this stable report and snapshot.
