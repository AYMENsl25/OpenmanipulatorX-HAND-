# OpenMANIPULATOR-X IK/FK Practical Research and Lessons Learned

## Purpose

This report explains in detail how inverse kinematics (IK) and forward kinematics (FK) can be used with the OpenMANIPULATOR-X, what previous implementations and research approaches have done, and which lessons are directly useful for this project.

The target project is:

```text
XYZ target
   ↓
Inverse Kinematics
   ↓
Joint angles
   ↓
DYNAMIXEL motor positions
   ↓
OpenCR
   ↓
OpenMANIPULATOR-X
```

The reverse verification path is:

```text
Motor positions
   ↓
Joint angles
   ↓
Forward Kinematics
   ↓
XYZ
```

The objective is to build a reliable, experimentally validated XYZ-controlled OpenMANIPULATOR-X system and eventually connect it to YOLO-based object detection.

---

# 1. Executive summary

The most important lesson from existing OpenMANIPULATOR-X work is:

> Do not treat XYZ → motor position as one equation.

It is a chain of models:

```text
Cartesian target
        ↓
Robot coordinate system
        ↓
Inverse kinematics
        ↓
Robot joint angles
        ↓
Joint offsets / signs
        ↓
DYNAMIXEL motor position
        ↓
OpenCR communication
        ↓
Physical robot
```

Likewise, the reverse direction is:

```text
DYNAMIXEL position
        ↓
Motor-angle calibration
        ↓
Robot joint angles
        ↓
Forward kinematics
        ↓
Cartesian position
```

This separation is essential because a mathematically correct IK solution can still cause incorrect physical motion if the joint zero, sign, or coordinate convention is wrong.

---

# 2. What previous OpenMANIPULATOR-X work tells us

Several existing sources use the OpenMANIPULATOR-X as a mathematical robotics platform.

Common approaches include:

1. Denavit-Hartenberg (DH) modeling.
2. Homogeneous transformation matrices.
3. Geometric inverse kinematics.
4. Numerical/Jacobian-based IK.
5. ROS and MoveIt-based IK.
6. Custom ROS nodes that receive XYZ targets and return joint angles.
7. FK → IK → FK validation.
8. Joint-limit checking.
9. Multiple IK solution handling.
10. Simulation before real hardware.

These approaches are complementary rather than competing.

For this project, the best path is:

```text
Official robot model
        +
Published kinematic model
        +
Our experimentally calibrated motor data
        ↓
Verified FK
        ↓
Analytical IK
        ↓
Real robot validation
```

---

# 3. Official ROBOTIS approach

ROBOTIS provides a manipulator software framework containing kinematic functionality.

Relevant official documentation:

https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/software/robotis_manipulator_libs/robotis_manipulator_libs.md

The framework includes functionality related to:

- forward kinematics
- inverse kinematics
- Jacobian calculations
- joint values
- target poses
- manipulator chains

The important lesson is that ROBOTIS treats kinematics as a separate layer from actuator communication.

That supports the architecture we are building.

---

# 4. Official robot model and URDF

The OpenMANIPULATOR-X is also represented by URDF/Xacro robot descriptions.

Official source/reference:

https://github.com/ROBOTIS-GIT/open_manipulator

Official specification:

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/

The URDF is important because it defines the robot's actual:

- links
- joints
- joint origins
- axes
- limits
- transformations

The URDF should eventually become one of our strongest references for resolving differences between a simplified geometric model and the actual OpenMANIPULATOR-X.

---

# 5. Why DH parameters are useful

Previous research commonly represents the OpenMANIPULATOR-X using Denavit-Hartenberg parameters.

A published model gives approximately:

```text
d1 = 77 mm
a2 = 130 mm
a3 = 124 mm
a4 = 126 mm
```

and includes an angular offset relationship for the shoulder/elbow joints.

Research reference:

https://pdfs.semanticscholar.org/db2c/24bfcb613b42b8f2b25d8668b6cfe965a2d1.pdf

DH modeling provides a systematic way to construct:

```text
T01
T12
T23
T34
```

and therefore:

```text
T04 = T01 T12 T23 T34
```

The translation part of the final transformation gives:

```text
X
Y
Z
```

while the rotation part gives end-effector orientation.

---

# 6. Why our current geometric FK is still useful

Our current project uses:

```python
Z_BASE = 17.0
L1_Z   = 59.5
L2_X   = 24.0
L2_Z   = 128.0
L3_X   = 124.0
L4_X   = 126.0
```

This gives:

```text
Z0 = 17 + 59.5 = 76.5 mm
```

and:

```text
L2 = 128 mm
L3 = 124 mm
L4 = 126 mm
```

The model calculates:

```text
a2 = θ2
a3 = θ2 + θ3
a4 = θ2 + θ3 + θ4
```

then:

```text
r =
24 cos(a2)
+ 124 cos(a3)
+ 126 cos(a4)
```

and:

```text
Z =
76.5
+ 128 sin(a2)
+ 124 sin(a3)
+ 126 sin(a4)
```

then:

```text
X = r cos(θ1)
Y = r sin(θ1)
```

This is a very useful project model because it is easy to understand and easy to invert.

However, it must still be checked against the official URDF and joint conventions.

---

# 7. The first major lesson: coordinate conventions matter

Many robotics problems fail because the mathematical equations are correct but the coordinate system is wrong.

We must define:

```text
Origin
X direction
Y direction
Z direction
positive joint rotations
zero angle
```

For example:

```text
θ1 = atan2(Y,X)
```

only works if the XY coordinate convention agrees with the robot model.

The same applies to shoulder and elbow angles.

Therefore we should never assume:

```text
motor angle = DH angle
```

without verification.

---

# 8. The second major lesson: motor angle is not necessarily joint angle

Our current calibration uses:

```text
ID11_CENTER = 351°
ID12_ZERO   = 0°
ID13_ZERO   = 0°
ID14_ZERO   = 0°
```

This creates the mapping:

```text
ID11 = 351°
      ↓
θ1 = 0°
```

The other joints currently use:

```text
θ2 = ID12
θ3 = ID13
θ4 = ID14
```

This is a project calibration convention.

A more general system should represent:

```text
joint_angle = sign × motor_angle + offset
```

For example:

```python
theta = sign * motor_angle + offset
```

This makes it possible to handle:

- reversed motors
- nonzero zero positions
- mechanical offsets

without changing the mathematical FK.

---

# 9. The real home position is an experimental calibration point

Our measured home position is:

```text
ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

Our current FK gives:

```text
X = 143.523 mm
Y = 0.000 mm
Z = 208.985 mm
```

This is valuable because it connects:

```text
Real robot
    ↓
Motor positions
    ↓
Our FK model
    ↓
Cartesian coordinates
```

This point should be preserved as a permanent regression test.

---

# 10. What a regression test should look like

Every time the kinematics code is changed, test:

```text
351°, 1°, 1°, 90°
```

and verify:

```text
X ≈ 143.523 mm
Y ≈ 0 mm
Z ≈ 208.985 mm
```

If a change suddenly produces:

```text
X = 200 mm
```

we know that the change broke the model.

This is standard engineering practice and will save a lot of debugging time.

---

# 11. The third major lesson: XYZ does not uniquely define a 4-DOF arm

The arm has four positioning joints:

```text
θ1
θ2
θ3
θ4
```

XYZ provides:

```text
X
Y
Z
```

Therefore one additional constraint is needed for a unique general solution.

Previous OpenMANIPULATOR-X work often uses an end-effector pitch angle.

Therefore the general interface should be:

```python
inverse_kinematics(
    x,
    y,
    z,
    pitch
)
```

For the first version we can use a fixed pitch:

```python
pitch = 90.0
```

This gives a controlled and repeatable experiment.

---

# 12. The fourth major lesson: exploit the robot's geometry

The base joint is easy:

```text
θ1 = atan2(Y,X)
```

Then:

```text
r = sqrt(X² + Y²)
```

reduces the 3D positioning problem to a 2D vertical plane.

This is one of the most useful observations for the OpenMANIPULATOR-X.

Instead of solving:

```text
X,Y,Z → θ1,θ2,θ3,θ4
```

we solve:

```text
X,Y → θ1
```

and:

```text
r,Z → θ2,θ3,θ4
```

This makes analytical IK practical.

---

# 13. Geometric IK used by previous implementations

A practical OpenMANIPULATOR-X implementation is available here:

https://github.com/MOGI-ROS/Week-11-12-Robot-arms

This project demonstrates an OpenMANIPULATOR-X inverse-kinematics approach using geometric equations and target coordinates.

The useful lesson is:

> A custom geometric IK implementation can be used without requiring the entire ROS/MoveIt stack.

That is directly relevant to our Python + serial + OpenCR architecture.

---

# 14. Another useful kinematics implementation

Another OpenMANIPULATOR-X kinematics project:

https://github.com/b-Tomas/robot-kinematics

This project is useful because it approaches the robot through mathematical robotics concepts such as:

- transformations
- DH parameters
- FK
- IK
- joint relationships

This provides an independent reference against which we can compare our implementation.

---

# 15. Why analytical IK is better than our current grid search

Our current IK searches:

```text
θ2 = -120° ... +120°
step = 0.5°

θ3 = -180° ... +180°
step = 0.5°
```

That is approximately:

```text
481 × 721
≈ 346,000
```

candidate combinations.

This is acceptable as a prototype.

It is not ideal as the final solver.

An analytical solver can directly calculate the candidate solutions.

The desired architecture is:

```text
XYZ + pitch
     ↓
Analytical equations
     ↓
Candidate solution 1
Candidate solution 2
     ↓
Joint-limit check
     ↓
Choose best solution
```

---

# 16. The two-link geometry behind analytical IK

After determining the base angle:

```text
θ1 = atan2(Y,X)
```

calculate:

```text
r = sqrt(X² + Y²)
```

If the wrist/end-effector orientation is:

```text
φ = θ2 + θ3 + θ4
```

then remove the last link:

```text
rw = r - L4 cos(φ)

zw = Z - Z0 - L4 sin(φ)
```

with:

```text
Z0 = 76.5 mm
L4 = 126 mm
```

The remaining problem is a two-link problem.

Use:

```text
L2 = 128 mm
L3 = 124 mm
```

and:

```text
R² = rw² + zw²
```

Then:

```text
cos(θ3*) =
(R² - L2² - L3²)
/
(2 L2 L3)
```

and:

```text
θ3* = ±acos(cos(θ3*))
```

The two signs provide two candidate elbow configurations.

---

# 17. Shoulder solution

The corresponding planar shoulder solution is:

```text
θ2* =
atan2(zw,rw)
-
atan2(
    L3 sin(θ3*),
    L2 + L3 cos(θ3*)
)
```

The exact conversion between this mathematical angle and the project's calibrated motor/joint angle must be established from the OpenMANIPULATOR-X convention.

This is a key point.

---

# 18. Multiple IK solutions are normal

A robot can often reach the same XYZ point in more than one configuration.

Conceptually:

```text
       Target
          ●
        /   \
       /     \
      /       \
 elbow-up   elbow-down
```

Both can potentially produce the same Cartesian position.

A good solver should:

1. Calculate all valid solutions.
2. Reject solutions outside joint limits.
3. Compare the remaining solutions with the current robot configuration.
4. Select the closest or otherwise preferred configuration.

This is better than blindly taking the first mathematical solution.

---

# 19. Home position can be used as a solution preference

Our current IK already contains a home-distance bias.

That idea can be generalized.

For each candidate solution:

```text
cost =
    Cartesian error
    +
    joint movement cost
    +
    optional home/configuration penalty
```

For example:

```text
cost =
    w1 * position_error
    +
    w2 * joint_distance
```

where:

```text
joint_distance =
sqrt(
    Δθ1² +
    Δθ2² +
    Δθ3² +
    Δθ4²
)
```

This is especially useful when controlling the robot continuously.

---

# 20. Previous ROS/MoveIt approach

MoveIt can perform numerical inverse kinematics using plugins such as KDL.

Reference:

https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html

The advantage is that MoveIt handles much of the robotics infrastructure.

The disadvantage for our current experiment is complexity.

Our current system already has:

```text
Python
 ↓
Serial
 ↓
OpenCR
```

Adding:

```text
ROS
MoveIt
controllers
planning scene
robot description
```

would introduce many additional components.

Therefore MoveIt should be considered a future validation/reference system rather than the first implementation.

---

# 21. Numerical IK and Jacobians

ROBOTIS's manipulator framework also supports Jacobian-related kinematics.

The basic numerical idea is:

```text
Current joint angles
        ↓
FK
        ↓
Current XYZ
        ↓
Position error
        ↓
Jacobian
        ↓
Joint update
        ↓
Repeat
```

Mathematically:

```text
Δx = J Δθ
```

so approximately:

```text
Δθ = J⁺ Δx
```

where `J⁺` is the pseudoinverse.

This approach is useful when:

- geometry becomes complicated
- analytical IK is difficult
- additional constraints are required
- continuous numerical refinement is desired

For our current OpenMANIPULATOR-X project, analytical IK is simpler and should be the first target.

---

# 22. Why FK is needed even when we already have IK

FK is not only for displaying XYZ.

It is the most important validation tool for IK.

Suppose IK calculates:

```text
θ1
θ2
θ3
θ4
```

We immediately run:

```text
FK(θ1,θ2,θ3,θ4)
```

and obtain:

```text
X'
Y'
Z'
```

Then:

```text
error =
sqrt(
    (X'-X)² +
    (Y'-Y)² +
    (Z'-Z)²
)
```

This tells us whether the mathematical IK actually solved the target.

---

# 23. The strongest validation loop

The strongest software test is:

```text
Target XYZ
     ↓
IK
     ↓
Joint angles
     ↓
FK
     ↓
XYZ'
     ↓
Compare XYZ and XYZ'
```

The strongest real-robot test is:

```text
Motor command
     ↓
Physical robot
     ↓
Measured/estimated joint angles
     ↓
FK
     ↓
XYZ
     ↓
Compare with commanded target
```

Both tests should be performed.

---

# 24. Use the existing Excel trajectory data

The existing Excel data has:

```text
Index
Time
ID11
ID12
ID13
ID14
ID15
```

This is an excellent experimental dataset.

For each row:

```text
ID11 ID12 ID13 ID14
        ↓
      FK
        ↓
      XYZ
```

Then:

```text
XYZ
 ↓
IK
 ↓
ID11' ID12' ID13' ID14'
```

Compare original and recovered motor angles.

This lets us measure how well the IK reproduces real recorded configurations.

---

# 25. Why this is better than testing only one pose

One pose can accidentally agree with a wrong model.

Many trajectory points reveal systematic errors.

For example, if we observe:

```text
ΔX ≈ +5 mm
```

for almost every point, we may have a link-length or base-offset problem.

If:

```text
ΔY
```

changes sign with base rotation, we may have a base-angle convention problem.

If errors increase dramatically near certain joint configurations, we may have a singularity or orientation problem.

This makes the Excel dataset a powerful diagnostic tool.

---

# 26. A useful validation experiment

For every recorded trajectory row:

### Test A — FK

```text
Recorded motors
→ FK
→ XYZ
```

### Test B — IK

```text
XYZ
→ IK
→ recovered motors
```

### Test C — FK after IK

```text
Recovered motors
→ FK
→ XYZ_reconstructed
```

Then store:

```text
Index
Original motor angles
XYZ
Recovered motor angles
Motor errors
XYZ reconstruction errors
```

This should eventually be automated in Python.

---

# 27. How to detect a bad IK solution

An IK result should be rejected if:

```text
cos_argument > 1
```

or:

```text
cos_argument < -1
```

before numerical tolerance handling.

This means the target is outside the mathematical reach of the selected geometry.

A robust implementation should use a small tolerance, for example:

```python
if c < -1.0 - tolerance or c > 1.0 + tolerance:
    raise ValueError("Target unreachable")
```

and clamp tiny floating-point deviations:

```python
c = max(-1.0, min(1.0, c))
```

---

# 28. Workspace checking

Before running IK, we should check whether the target is physically plausible.

For the simplified two-link section, the target distance must satisfy approximately:

```text
|L2-L3| ≤ R ≤ L2+L3
```

after accounting for the wrist position.

With:

```text
L2 = 128
L3 = 124
```

the basic two-link reach interval is:

```text
4 mm ≤ R ≤ 252 mm
```

before other robot geometry and joint limits are considered.

The full robot workspace is more restrictive because of:

- joint limits
- offsets
- wrist orientation
- physical collisions
- base geometry

Therefore geometric reach alone is not sufficient.

---

# 29. Joint limits are essential

A mathematically valid IK solution can still be physically invalid.

The solver should eventually return only solutions satisfying:

```text
θ1_min ≤ θ1 ≤ θ1_max
θ2_min ≤ θ2 ≤ θ2_max
θ3_min ≤ θ3 ≤ θ3_max
θ4_min ≤ θ4 ≤ θ4_max
```

These limits should be taken from the official OpenMANIPULATOR-X model/configuration rather than guessed.

---

# 30. Motor command limits are different from mathematical joint limits

There are at least three levels:

```text
Mathematical IK angle
        ↓
Allowed robot joint angle
        ↓
DYNAMIXEL command range
```

The command layer must not blindly accept every mathematical angle.

A safety layer should be placed between IK and the motor command.

---

# 31. Why our motor conversion should remain separate

Use:

```python
motor_to_fk_angles()
```

and:

```python
fk_to_motor_angles()
```

as separate functions.

That means the FK does not need to know about DYNAMIXEL calibration details.

Likewise, the IK does not need to know about serial packets.

This is good software architecture.

---

# 32. Recommended calibration representation

Instead of only:

```python
ID11_CENTER = 351
```

a more flexible future representation is:

```python
JOINT_CALIBRATION = {
    11: {"offset": ..., "sign": ...},
    12: {"offset": ..., "sign": ...},
    13: {"offset": ..., "sign": ...},
    14: {"offset": ..., "sign": ...},
}
```

Then:

```python
joint_angle = sign * motor_angle + offset
```

and:

```python
motor_angle = sign * (joint_angle - offset)
```

This will make future calibration much easier.

---

# 33. A major lesson from real robot experiments

Simulation and equations are not enough.

Real robots have:

- mechanical tolerances
- backlash
- flexible components
- encoder offsets
- mounting errors
- cable effects
- friction
- load-dependent errors

Therefore our system should distinguish:

```text
Kinematic model error
```

from:

```text
Motor calibration error
```

and from:

```text
Mechanical positioning error
```

---

# 34. How to use real measurements for calibration

A strong calibration experiment is:

1. Move to known joint configurations.
2. Record motor positions.
3. Calculate FK.
4. Measure actual end-effector position.
5. Compare calculated and measured XYZ.
6. Fit corrections.

Potential corrections include:

```text
joint zero offsets
link dimensions
coordinate offsets
```

This can gradually improve the model.

---

# 35. The importance of not changing everything at once

For this project, calibration should be incremental.

Recommended order:

```text
1. Coordinate system
2. Link dimensions
3. Joint zero offsets
4. Joint signs
5. FK
6. IK
7. Motor conversion
8. Real movement
9. YOLO integration
```

If we change all eight at once, it becomes difficult to identify the source of an error.

---

# 36. Recommended software test structure

Create:

```text
openmanipulator_kinematics.py
```

with:

```text
Geometry
Calibration
Motor ↔ Joint conversion
FK
IK
Validation
```

Then create:

```text
test_home.py
test_fk.py
test_ik.py
test_roundtrip.py
trajectory_validation.py
```

Finally:

```text
robot_controller.py
```

handles OpenCR/DYNAMIXEL communication.

This gives:

```text
Kinematics
    ≠
Hardware communication
```

which is much easier to debug.

---

# 37. Suggested `inverse_kinematics()` interface

The final function should eventually look conceptually like:

```python
inverse_kinematics(
    x,
    y,
    z,
    pitch,
    current_angles=None,
    preferred_configuration="nearest"
)
```

It should:

1. Calculate θ1.
2. Calculate radial distance.
3. Calculate wrist position.
4. Generate elbow-up and elbow-down candidates.
5. Convert mathematical angles to project joint angles.
6. Apply offsets/signs.
7. Check joint limits.
8. Check FK error.
9. Select the best valid solution.
10. Return joint and motor angles.

---

# 38. Recommended return value

Instead of returning only four numbers, a mature solver can return information such as:

```python
{
    "joint_angles": [...],
    "motor_angles": [...],
    "position_error_mm": ...,
    "configuration": "elbow_up",
    "reachable": True
}
```

This is useful for debugging and experiments.

---

# 39. Why this matters for YOLO

Eventually YOLO will provide an object location.

The pipeline will be:

```text
Camera
 ↓
YOLO
 ↓
Object detection
 ↓
Object coordinates
 ↓
Camera-to-robot transform
 ↓
X,Y,Z
 ↓
IK
 ↓
Joint angles
 ↓
Motor positions
 ↓
OpenCR
```

YOLO should not directly command motor positions.

The kinematics module should act as the mathematical interface between perception and motion.

---

# 40. Camera coordinates are a separate problem

A future YOLO system may give:

```text
pixel_x
pixel_y
```

but the robot needs:

```text
robot_X
robot_Y
robot_Z
```

Therefore we will eventually need:

```text
Camera calibration
+
Depth or geometric estimation
+
Camera-to-robot coordinate transformation
```

This is separate from IK.

The architecture should remain:

```text
Vision
 ↓
Cartesian target
 ↓
Kinematics
 ↓
Motion control
```

---

# 41. Research-based approach comparison

| Approach | Advantages | Disadvantages | Recommendation |
|---|---|---|---|
| Grid-search IK | Easy to understand | Slow, approximate | Prototype only |
| Analytical IK | Fast, accurate, transparent | Requires correct geometry | **Best first implementation** |
| Jacobian IK | Flexible, general | Requires iteration and good initial guess | Future option |
| MoveIt/KDL | Powerful robotics framework | More software complexity | Later validation |
| ROBOTIS solver | Official reference | Requires integrating their framework | Important reference |
| Numerical optimization | Flexible constraints | More complex | Future advanced option |

---

# 42. Recommended strategy for this project

Use a hybrid validation philosophy:

```text
Official ROBOTIS model
        ↓
Research DH model
        ↓
Our current FK model
        ↓
Real Excel measurements
        ↓
Analytical IK
        ↓
FK verification
        ↓
Real robot
```

If all of these agree, confidence in the system becomes much higher.

---

# 43. The home-point experiment

Our first analytical IK test should use:

```text
Target:

X = 143.523 mm
Y = 0.000 mm
Z = 208.985 mm
```

with the chosen wrist orientation corresponding to:

```text
θ4 = 90°
```

The expected calibrated motor configuration is approximately:

```text
ID11 = 351°
ID12 = 1°
ID13 = 1°
ID14 = 90°
```

The important test is:

```text
Home motors
   ↓
FK
   ↓
Home XYZ
   ↓
IK
   ↓
Recovered joints
```

If the result is not close, we investigate the angle convention before moving the robot.

---

# 44. The previous target example

Another useful target from the project is:

```text
X = 169.672900 mm
Y = -21.402300 mm
Z = 86.995000 mm

Roll  = 0°
Pitch = 84.820000°
Yaw   = -7.730000°
```

This is valuable because it tests a configuration away from the home point.

For this pose:

```text
θ1 = atan2(Y,X)
```

gives approximately:

```text
θ1 ≈ -7°
```

The remaining joints must be solved using the planar geometry and the selected orientation convention.

This should become the second important validation point.

---

# 45. What previous implementations can teach us

## Lesson A — Geometric IK is practical

The MOGI-ROS implementation demonstrates that a custom OpenMANIPULATOR-X geometric solver is feasible.

Reference:

https://github.com/MOGI-ROS/Week-11-12-Robot-arms

## Lesson B — DH is useful for verification

The research literature provides a formal kinematic representation.

Reference:

https://pdfs.semanticscholar.org/db2c/24bfcb613b42b8f2b25d8668b6cfe965a2d1.pdf

## Lesson C — ROS/MoveIt can provide an independent numerical reference

Reference:

https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html

## Lesson D — Official ROBOTIS software gives the manufacturer-side reference

Reference:

https://github.com/ROBOTIS-GIT/emanual/blob/master/docs/en/software/robotis_manipulator_libs/robotis_manipulator_libs.md

---

# 46. What we should NOT do

We should not:

```text
Guess joint offsets
```

or:

```text
Assume every motor angle equals a DH angle
```

or:

```text
Send analytical IK output directly to motors
```

without calibration.

We should also not:

```text
Use only one test pose
```

or:

```text
Assume FK is correct because the result looks reasonable
```

Instead:

```text
Model
 ↓
Test
 ↓
Compare
 ↓
Calibrate
 ↓
Test again
```

---

# 47. Recommended experimental phases

## Phase 1 — Mathematical

No robot movement.

```text
FK
IK
FK
```

Test with known numbers.

## Phase 2 — Dataset

Use Excel trajectory data.

```text
Motors
→ FK
→ XYZ
→ IK
→ motors
```

## Phase 3 — Simulation

If available, compare against URDF/ROS/MoveIt.

## Phase 4 — Low-risk physical validation

Use known, conservative poses and verify the resulting configuration.

## Phase 5 — Cartesian motion

Enter XYZ manually.

```text
X = ...
Y = ...
Z = ...
```

and command the robot.

## Phase 6 — YOLO

Connect vision only after the Cartesian controller is reliable.

---

# 48. Final architecture

The final project should look like:

```text
                         CAMERA
                            │
                            ▼
                           YOLO
                            │
                            ▼
                    Object coordinates
                            │
                            ▼
              Camera → Robot transformation
                            │
                            ▼
                         XYZ + Pitch
                            │
                            ▼
                  ┌──────────────────┐
                  │  IK SOLVER       │
                  │                  │
                  │  XYZ → θ1..θ4    │
                  └────────┬─────────┘
                           │
                           ▼
                  Joint validation
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
                      DYNAMIXEL
                           │
                           ▼
                  OpenMANIPULATOR-X
                           │
                           ▼
                          FK
                           │
                           ▼
                      XYZ feedback
```

---

# 49. Final checklist

Before considering XYZ → motor control reliable, verify:

- [ ] Official coordinate frame confirmed.
- [ ] Link dimensions confirmed.
- [ ] Joint axes confirmed.
- [ ] Joint zero positions confirmed.
- [ ] Motor signs confirmed.
- [ ] Home position recorded.
- [ ] FK reproduces known positions.
- [ ] IK reproduces known FK positions.
- [ ] Analytical IK replaces grid search.
- [ ] Both elbow configurations handled.
- [ ] Joint limits implemented.
- [ ] Unreachable targets rejected.
- [ ] IK → FK error calculated.
- [ ] Motor calibration separated from IK.
- [ ] Excel trajectory validation completed.
- [ ] Real robot tested only after mathematical validation.
- [ ] YOLO connected only after XYZ control is reliable.

---

# 50. Most important next experiment

The next experiment should be:

```text
STEP 1
Motor:
351°, 1°, 1°, 90°
        ↓
FK
        ↓
143.523, 0, 208.985 mm
```

Then:

```text
STEP 2
143.523, 0, 208.985
        ↓
Analytical IK
        ↓
Recovered θ1 θ2 θ3 θ4
```

Then:

```text
STEP 3
Recovered angles
        ↓
FK
        ↓
Recovered XYZ
```

Then calculate:

```text
ΔX
ΔY
ΔZ
```

Only after this succeeds should the IK output be connected to OpenCR.

---

# 51. Final conclusion

The previous OpenMANIPULATOR-X implementations and research strongly support the approach being developed here.

The best path for this project is not to copy one implementation blindly.

Instead, combine:

```text
ROBOTIS official documentation
        +
Official robot model / URDF
        +
DH research
        +
Geometric IK implementations
        +
Our existing FK
        +
Our measured motor calibration
        +
Our Excel trajectory data
```

The result should be a project-specific, experimentally validated kinematics layer.

The immediate target is:

```text
XYZ + wrist orientation
        ↓
Analytical IK
        ↓
θ1 θ2 θ3 θ4
        ↓
Motor calibration
        ↓
DYNAMIXEL positions
```

with:

```text
Motor positions
        ↓
FK
        ↓
XYZ
```

used continuously as the validation mechanism.

The long-term goal is:

```text
YOLO
 ↓
Object XYZ
 ↓
OpenMANIPULATOR-X IK
 ↓
Joint angles
 ↓
Motor positions
 ↓
OpenCR
 ↓
Robot
```

The most important engineering principle is:

> **Separate perception, Cartesian kinematics, joint calibration, and hardware control. Validate every layer independently before combining them.**
