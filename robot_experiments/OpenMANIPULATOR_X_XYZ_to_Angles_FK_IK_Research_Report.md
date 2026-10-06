# OpenMANIPULATOR-X: XYZ → Joint Angles / Forward Kinematics Research Report

## 1. Project objective

The goal of this project is to control an OpenMANIPULATOR-X by specifying a desired Cartesian position:

- `X`
- `Y`
- `Z`

and calculating the required arm joint angles:

- `θ1` — base
- `θ2` — shoulder
- `θ3` — elbow
- `θ4` — wrist

The resulting angles are then converted into DYNAMIXEL motor positions for:

- ID11
- ID12
- ID13
- ID14

The long-term architecture is:

```text
YOLO / User
    ↓
Target XYZ
    ↓
Inverse Kinematics
    ↓
θ1 θ2 θ3 θ4
    ↓
Motor calibration
    ↓
DYNAMIXEL positions
    ↓
OpenCR
    ↓
OpenMANIPULATOR-X
```

---

# 2. Main conclusion from the research

ROBOTIS already provides a manipulator kinematics framework supporting:

- Forward Kinematics (FK)
- Inverse Kinematics (IK)
- Jacobian calculations
- joint values
- target pose → joint values

The official ROBOTIS implementation therefore gives us an important reference for validating our own Python implementation.

Recommended references:

- ROBOTIS OpenMANIPULATOR-X e-Manual:
  https://emanual.robotis.com/docs/en/platform/openmanipulator_main/
- ROBOTIS manipulator library documentation:
  https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/software/robotis_manipulator_libs/robotis_manipulator_libs.md
- ROBOTIS OpenMANIPULATOR source:
  https://github.com/robotis-git/open_manipulator
- OpenMANIPULATOR-X specification:
  https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/

These should be treated as primary references for the physical robot and software conventions.

---

# 3. OpenMANIPULATOR-X degrees of freedom

The OpenMANIPULATOR-X has:

- 4 arm joints
- 1 gripper joint

For Cartesian arm positioning we use:

```text
ID11 → Joint 1 → Base
ID12 → Joint 2 → Shoulder
ID13 → Joint 3 → Elbow
ID14 → Joint 4 → Wrist
ID15 → Gripper
```

For XYZ positioning, ID15 is not part of the arm-position IK.

Therefore:

```text
XYZ → ID11 ID12 ID13 ID14
```

The gripper can be controlled separately.

---

# 4. Important distinction: XYZ alone does not uniquely determine all four arm joints

There are four arm variables:

```text
θ1
θ2
θ3
θ4
```

but XYZ provides only three constraints:

```text
X
Y
Z
```

Therefore:

```text
4 unknowns
3 constraints
```

does not generally give a unique solution.

A fourth condition is normally required, such as:

```text
wrist pitch
```

Therefore the general problem is better represented as:

```text
X
Y
Z
Pitch
 ↓
IK
 ↓
θ1 θ2 θ3 θ4
```

For the first experiment, however, it is reasonable to keep the wrist angle fixed.

The current implementation uses:

```text
θ4 = 90°
```

based on the measured home position.

Later, the solver should accept:

```python
inverse_kinematics(x, y, z, pitch)
```

instead of always fixing θ4.

---

# 5. Published OpenMANIPULATOR-X dimensions

Research using the OpenMANIPULATOR-X gives approximately:

```text
d1 = 77 mm
a2 = 130 mm
a3 = 124 mm
a4 = 126 mm
```

A published DH representation also includes a joint-angle offset relationship between joints 2 and 3.

This is important because the mathematical DH angles are not necessarily identical to raw DYNAMIXEL motor positions.

Useful research reference:

https://pdfs.semanticscholar.org/db2c/24bfcb613b42b8f2b25d8668b6cfe965a2d1.pdf

Another useful OpenMANIPULATOR-X IK reference:

https://www.scribd.com/document/860810323/A-comparison-study-on-the-dynamic-control-of-OpenMANIPULATOR-X-by-PD-with-gravity-compensation-tuned-by-oscillation-damping-based-on-the-phase-traject

Additional research:

https://www.mdpi.com/2218-6581/14/2/21

Recent OpenMANIPULATOR-X research:

https://www.sciencedirect.com/science/article/pii/S0016003225004053

---

# 6. Current project FK model

The current Python FK model uses:

```python
Z_BASE = 17.0
L1_Z   = 59.5
L2_X   = 24.0
L2_Z   = 128.0
L3_X   = 124.0
L4_X   = 126.0
```

Therefore:

```text
Z_BASE + L1_Z = 76.5 mm
```

The model represents the geometry using:

```text
24 mm radial component
128 mm vertical link component
124 mm link
126 mm link
```

This may be an alternative geometric decomposition of the published DH dimensions.

It must be validated against the official URDF/transformation model before being considered the final robot model.

---

# 7. Current motor-zero references

The project currently uses:

```python
ID11_CENTER = 351.0
ID12_ZERO   = 0.0
ID13_ZERO   = 0.0
ID14_ZERO   = 0.0
```

The important measured reference is:

```text
ID11 = 351°
```

for the centered base.

The current code therefore calculates:

```python
theta1 = normalize_angle(id11 - 351.0)
theta2 = normalize_angle(id12)
theta3 = normalize_angle(id13)
theta4 = normalize_angle(id14)
```

This is a project-specific calibration convention.

It should eventually be reconciled with the official ROBOTIS joint convention.

---

# 8. Real measured home position

The current measured home motor configuration is:

```text
ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

Using the current FK model, this produces approximately:

```text
X = 143.523 mm
Y = 0.000 mm
Z = 208.985 mm
```

This is a very important experimental reference point.

It should be used as a calibration and round-trip validation point.

---

# 9. Current Forward Kinematics

The current FK first converts motor angles into project FK angles.

```python
def motor_to_fk_angles(id11, id12, id13, id14):

    theta1 = normalize_angle(id11 - ID11_CENTER)
    theta2 = normalize_angle(id12 - ID12_ZERO)
    theta3 = normalize_angle(id13 - ID13_ZERO)
    theta4 = normalize_angle(id14 - ID14_ZERO)

    return theta1, theta2, theta3, theta4
```

Then degrees are converted to radians.

The compound planar angles are:

```text
a2 = θ2
a3 = θ2 + θ3
a4 = θ2 + θ3 + θ4
```

The radial distance is:

```text
r =
    L2_X cos(a2)
  + L3_X cos(a3)
  + L4_X cos(a4)
```

The height is:

```text
Z =
    Z_BASE
  + L1_Z
  + L2_Z sin(a2)
  + L3_X sin(a3)
  + L4_X sin(a4)
```

The Cartesian coordinates are:

```text
X = r cos(θ1)

Y = r sin(θ1)
```

Therefore:

```text
θ1 θ2 θ3 θ4
      ↓
     FK
      ↓
     X Y Z
```

---

# 10. Current FK Python code

```python
import math

Z_BASE = 17.0
L1_Z   = 59.5
L2_X   = 24.0
L2_Z   = 128.0
L3_X   = 124.0
L4_X   = 126.0

ID11_CENTER = 351.0
ID12_ZERO = 0.0
ID13_ZERO = 0.0
ID14_ZERO = 0.0


def normalize_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


def motor_to_fk_angles(id11, id12, id13, id14):

    theta1 = normalize_angle(id11 - ID11_CENTER)
    theta2 = normalize_angle(id12 - ID12_ZERO)
    theta3 = normalize_angle(id13 - ID13_ZERO)
    theta4 = normalize_angle(id14 - ID14_ZERO)

    return theta1, theta2, theta3, theta4


def forward_kinematics(id11, id12, id13, id14):

    theta1, theta2, theta3, theta4 = motor_to_fk_angles(
        id11, id12, id13, id14
    )

    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)
    t4 = math.radians(theta4)

    a2 = t2
    a3 = t2 + t3
    a4 = t2 + t3 + t4

    radial = (
        L2_X * math.cos(a2)
        + L3_X * math.cos(a3)
        + L4_X * math.cos(a4)
    )

    z = (
        Z_BASE
        + L1_Z
        + L2_Z * math.sin(a2)
        + L3_X * math.sin(a3)
        + L4_X * math.sin(a4)
    )

    x = radial * math.cos(t1)
    y = radial * math.sin(t1)

    return x, y, z
```

---

# 11. Current IK model

The current IK does:

```text
X,Y,Z
 ↓
θ1 = atan2(Y,X)
 ↓
calculate radial distance
 ↓
fix θ4 = home θ4
 ↓
brute-force θ2 and θ3
 ↓
choose lowest position error
```

The base equation is:

```text
θ1 = atan2(Y, X)
```

The target radial distance is:

```text
r_target = sqrt(X² + Y²)
```

The current solver then searches:

```text
θ2 = -120° ... +120°
step = 0.5°

θ3 = -180° ... +180°
step = 0.5°
```

This creates approximately:

```text
481 × 721 ≈ 346,000
```

candidate combinations per target.

---

# 12. Current IK home bias

The current solver uses the measured home configuration:

```text
ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

which becomes approximately:

```text
θ1 = 0°
θ2 = 1°
θ3 = 1°
θ4 = 90°
```

The solver uses a small penalty to prefer solutions close to the home configuration.

The idea is:

```text
total_score =
    position_error
    +
    small_home_distance_penalty
```

This is useful when several solutions exist.

---

# 13. Why the current brute-force IK should be replaced

The grid-search implementation is useful for proving the concept, but it should not be the final IK.

Problems:

### 13.1 Computational cost

Approximately 346,000 combinations are evaluated for every target.

### 13.2 Resolution limitation

With a 0.5° grid, the solver cannot directly return arbitrary angles such as:

```text
32.27°
-41.63°
```

It can only select values on the grid.

### 13.3 Cartesian error

A mathematically exact solution may exist between grid points.

### 13.4 Multiple configurations

The current search handles multiple possibilities only indirectly through the home-distance penalty.

### 13.5 Future YOLO control

YOLO may generate many targets. An analytical solver is much more appropriate for repeated real-time calculations.

---

# 14. Recommended analytical IK structure

The current FK equations allow the IK to be transformed into a planar geometry problem.

First calculate:

```text
r = sqrt(X² + Y²)
```

and:

```text
θ1 = atan2(Y,X)
```

Then solve the remaining vertical-plane problem.

The current model uses:

```text
L2 = 128 mm
L3 = 124 mm
L4 = 126 mm
Z0 = 76.5 mm
```

If the wrist/end-effector orientation is represented by:

```text
φ = θ2 + θ3 + θ4
```

then the wrist point can be calculated by removing the final link:

```text
rw = r - L4 cos(φ)

zw = Z - Z0 - L4 sin(φ)
```

The remaining problem is a two-link geometry problem.

---

# 15. Law-of-cosines solution

Define:

```text
R² = rw² + zw²
```

Then:

```text
cos(θ3*) =
    (R² - L2² - L3²)
    / (2 L2 L3)
```

and:

```text
θ3* = acos(cos(θ3*))
```

There are generally two solutions:

```text
+acos(...)
-acos(...)
```

These correspond to different elbow configurations, commonly described as:

```text
elbow-up
elbow-down
```

The correct mapping to the project's `θ3` convention must be verified against the actual OpenMANIPULATOR-X joint convention.

---

# 16. Shoulder angle

For the two-link planar problem:

```text
θ2* =
    atan2(zw, rw)
    -
    atan2(
        L3 sin(θ3*),
        L2 + L3 cos(θ3*)
    )
```

Again, the final transformation into the project's actual `θ2` and `θ3` must account for any OpenMANIPULATOR-X joint offsets.

This is the point where the current project FK model must be reconciled with the official ROBOTIS model.

---

# 17. Critical angle-convention issue

The current project treats:

```text
θ2 = motor ID12 angle
θ3 = motor ID13 angle
θ4 = motor ID14 angle
```

directly after calibration.

Published OpenMANIPULATOR-X DH models use joint relationships that can include offsets.

Therefore we must distinguish:

```text
Mathematical DH angle
        ↓
Robot joint angle
        ↓
Calibrated motor angle
        ↓
DYNAMIXEL position value
```

These are not automatically the same quantity.

This is one of the most important points to verify before commanding the physical robot.

---

# 18. Why the home position is the best first validation

The measured home position gives:

```text
Motor angles:

ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

Current FK gives:

```text
X = 143.523 mm
Y = 0.000 mm
Z = 208.985 mm
```

The new analytical IK should be tested using:

```text
X = 143.523
Y = 0
Z = 208.985
```

and should ideally recover a configuration close to:

```text
ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

This is a round-trip test.

---

# 19. Recommended round-trip validation

The main validation loop should be:

```text
Motor angles
    ↓
FK
    ↓
X Y Z
    ↓
IK
    ↓
Recovered motor angles
```

Then calculate:

```text
ΔID11
ΔID12
ΔID13
ΔID14
```

and independently:

```text
Target XYZ
    ↓
IK
    ↓
Joint angles
    ↓
FK
    ↓
Calculated XYZ
```

Then calculate:

```text
ΔX
ΔY
ΔZ
```

The IK is considered successful when the Cartesian error is sufficiently small and the returned joints are within physical limits.

---

# 20. Recommended Excel trajectory validation

The existing trajectory Excel data is extremely useful.

For each trajectory row:

```text
Index
Time
ID11
ID12
ID13
ID14
ID15
```

we can perform:

```text
Excel motor angles
       ↓
FK
       ↓
X,Y,Z
       ↓
IK
       ↓
Recovered motor angles
```

Then create a validation table such as:

| Index | X | Y | Z | Original J1 | IK J1 | J1 Error |
|------:|---:|---:|---:|---:|---:|---:|
| 0 | ... | ... | ... | ... | ... | ... |
| 1 | ... | ... | ... | ... | ... | ... |
| 2 | ... | ... | ... | ... | ... | ... |

and also:

| Index | dX | dY | dZ | Position Error |
|------:|---:|---:|---:|---:|
| 0 | ... | ... | ... | ... |
| 1 | ... | ... | ... | ... |
| 2 | ... | ... | ... | ... |

This provides a much stronger validation than testing only one pose.

---

# 21. Recommended final software architecture

Create one common kinematics module:

```text
openmanipulator_kinematics.py
```

It should contain:

```python
# Geometry
Z_BASE
L1_Z
L2_X
L2_Z
L3_X
L4_X

# Motor calibration
ID11_CENTER
ID12_ZERO
ID13_ZERO
ID14_ZERO

# Home
HOME_ID11
HOME_ID12
HOME_ID13
HOME_ID14

# Conversion
motor_to_fk_angles()
fk_to_motor_angles()

# FK
forward_kinematics()

# IK
inverse_kinematics()

# Validation
validate_ik_fk()
```

Other programs can then import this module.

For example:

```text
test_fk.py
test_ik.py
trajectory_validation.py
robot_controller.py
yolo_controller.py
```

---

# 22. Recommended final control pipeline

The intended final pipeline is:

```text
                  USER / YOLO
                       │
                       ▼
                 Target XYZ
                       │
                       ▼
              Target validation
                       │
              ┌────────┴────────┐
              │                 │
              ▼                 ▼
          Workspace         Joint limits
            check              check
              │                 │
              └────────┬────────┘
                       ▼
                Analytical IK
                       │
                       ▼
                  θ1 θ2 θ3 θ4
                       │
                       ▼
               Motor calibration
                       │
                       ▼
                 ID11 ID12 ID13 ID14
                       │
                       ▼
                     OpenCR
                       │
                       ▼
                DYNAMIXEL motors
                       │
                       ▼
              OpenMANIPULATOR-X
```

---

# 23. Integration with YOLO

The long-term project can use:

```text
Camera
   ↓
YOLO
   ↓
Object detection
   ↓
Pixel coordinates
   ↓
Camera-to-robot coordinate transformation
   ↓
Robot XYZ target
   ↓
Inverse Kinematics
   ↓
Joint angles
   ↓
Motor positions
   ↓
OpenCR
   ↓
OpenMANIPULATOR-X
```

The important point is that YOLO does not need to know anything about the motor protocol.

YOLO only needs to provide a target position.

The kinematics layer handles:

```text
XYZ → angles
```

and the motor-control layer handles:

```text
angles → DYNAMIXEL commands
```

---

# 24. Recommended development roadmap

## Step 1 — Confirm coordinate system

Verify:

```text
X axis
Y axis
Z axis
origin
positive rotations
```

against the official ROBOTIS URDF/model.

## Step 2 — Confirm physical dimensions

Verify the project's:

```text
17
59.5
24
128
124
126
```

representation against the official robot model.

## Step 3 — Confirm joint offsets

Especially for:

```text
Joint 2
Joint 3
```

## Step 4 — Keep the current FK

Use it as the first experimental model.

## Step 5 — Validate FK against real trajectory data

Use the existing Excel measurements.

## Step 6 — Replace brute-force IK

Implement analytical IK based on the verified geometry.

## Step 7 — Add joint limits

Reject solutions outside the robot's safe operating range.

## Step 8 — Support multiple IK solutions

For example:

```text
elbow-up
elbow-down
```

Then select the best solution based on:

- joint limits
- distance from current configuration
- distance from home
- desired wrist orientation

## Step 9 — Verify IK with FK

Perform:

```text
XYZ → IK → FK → XYZ
```

and calculate Cartesian error.

## Step 10 — Convert angles to DYNAMIXEL commands

Use the experimentally verified motor calibration.

## Step 11 — Test real robot movement

Only after software validation.

## Step 12 — Connect YOLO

Finally:

```text
YOLO → XYZ → IK → motors
```

---

# 25. Useful external implementations and research

## Official ROBOTIS

ROBOTIS OpenMANIPULATOR-X:

https://emanual.robotis.com/docs/en/platform/openmanipulator_main/

ROBOTIS OpenMANIPULATOR-X specification:

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/

ROBOTIS manipulator kinematics:

https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/software/robotis_manipulator_libs/robotis_manipulator_libs.md

ROBOTIS source:

https://github.com/robotis-git/open_manipulator

## Research / mathematical references

OpenMANIPULATOR-X DH / FK research:

https://pdfs.semanticscholar.org/db2c/24bfcb613b42b8f2b25d8668b6cfe965a2d1.pdf

OpenMANIPULATOR-X dynamic-control / IK reference:

https://www.scribd.com/document/860810323/A-comparison-study-on-the-dynamic-control-of-OpenMANIPULATOR-X-by-PD-with-gravity-compensation-tuned-by-oscillation-damping-based-on-the-phase-traject

OpenMANIPULATOR-X robotics research:

https://www.mdpi.com/2218-6581/14/2/21

Recent OpenMANIPULATOR-X research:

https://www.sciencedirect.com/science/article/pii/S0016003225004053

## Practical implementations

Geometric OpenMANIPULATOR-X IK:

https://github.com/MOGI-ROS/Week-11-12-Robot-arms

Robot kinematics project:

https://github.com/b-Tomas/robot-kinematics

## MoveIt

MoveIt KDL inverse kinematics:

https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html

---

# 26. Important warnings before physical robot control

The current FK/IK code is a mathematical experiment and should not yet be treated as a guaranteed physical controller.

Before sending calculated positions to the real motors, verify:

1. Joint-angle sign conventions.
2. Joint zero positions.
3. DYNAMIXEL position representation.
4. Motor direction.
5. Physical joint limits.
6. The official OpenMANIPULATOR-X coordinate frame.
7. The actual URDF link/joint transformations.
8. Whether the 24 mm + 128 mm representation exactly matches the physical model.
9. Wrist-angle convention.
10. Multiple IK solutions.

A mathematically correct IK equation can still produce the wrong physical motion if the motor calibration or coordinate convention is wrong.

---

# 27. Final recommended architecture

The final system should be separated into four layers:

```text
LAYER 1
Cartesian target
----------------
X Y Z Pitch


LAYER 2
Mathematical IK
---------------
X Y Z Pitch
      ↓
θ1 θ2 θ3 θ4


LAYER 3
Robot calibration
-----------------
θ1 θ2 θ3 θ4
      ↓
motor angles / calibrated positions


LAYER 4
Hardware control
----------------
ID11 ID12 ID13 ID14
      ↓
OpenCR
      ↓
DYNAMIXEL
```

This separation makes debugging much easier.

---

# 28. Final objective

The complete project should eventually achieve:

```text
Desired XYZ
    ↓
Verified OpenMANIPULATOR-X IK
    ↓
Correct joint angles
    ↓
Correct motor positions
    ↓
Real robot movement
    ↓
FK verification
    ↓
Measured Cartesian position
```

and ultimately:

```text
YOLO
 ↓
Object XYZ
 ↓
OpenMANIPULATOR-X IK
 ↓
Motor angles
 ↓
OpenCR
 ↓
Robot
 ↓
Object interaction
```

The immediate next milestone is:

```text
HOME MOTORS
351°, 1°, 1°, 90°
        ↓
       FK
        ↓
143.523, 0, 208.985 mm
        ↓
Analytical IK
        ↓
approximately
351°, 1°, 1°, 90°
```

If this round-trip test succeeds, we have a strong foundation for moving from the current prototype to a proper XYZ-controlled OpenMANIPULATOR-X.
