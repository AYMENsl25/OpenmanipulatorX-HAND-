# OpenMANIPULATOR-X XYZ + Motor Angle Controller

## 1. Architecture explanation

The project is split into two layers:

```text
Python GUI -> XYZ / motor-angle commands -> OpenCR -> DYNAMIXEL motors
Motor positions -> Python -> FK -> XYZ
```

Python owns the GUI, calibration, FK, IK, input validation, and FK-after-IK validation. OpenCR owns DYNAMIXEL communication, torque control, one-shot motor reads, slow motor interpolation, and HOME movement. The OpenCR firmware does not contain FK/IK.

## 2. OpenCR code

Firmware path:

```text
openmanipulator_project/opencr_firmware/OpenManipulatorXYZController/OpenManipulatorXYZController.ino
```

The sketch uses:

```cpp
#include <Dynamixel2Arduino.h>
```

It does not use `OpenManipulator.h`.

## 3. Python code

Python paths:

```text
openmanipulator_project/python_app/main.py
openmanipulator_project/python_app/gui.py
openmanipulator_project/python_app/serial_controller.py
openmanipulator_project/python_app/kinematics.py
openmanipulator_project/python_app/config.py
openmanipulator_project/python_app/run_tests.py
```

Run the GUI:

```powershell
.\venv\Scripts\python.exe openmanipulator_project\python_app\main.py
```

## 4. Arduino/OpenCR installation

1. Install Arduino IDE.
2. Add the ROBOTIS OpenCR board package using the official OpenCR setup instructions.
3. Select the OpenCR board in Arduino IDE.
4. Install or confirm the ROBOTIS `Dynamixel2Arduino` library.
5. Open `OpenManipulatorXYZController.ino`.
6. Connect OpenCR over USB.
7. Select the correct COM port.
8. Upload the sketch.

The firmware assumes OpenMANIPULATOR-X arm motors on IDs `11`, `12`, `13`, and `14`, DYNAMIXEL bus on OpenCR `Serial3`, direction pin `84`, protocol `2.0`, and DYNAMIXEL baud `1000000`.

## 5. Python installation

Tkinter is included with normal Python installers on Windows. The serial layer needs `pyserial`.

```powershell
.\venv\Scripts\python.exe -m pip install pyserial
```

Your current local venv already imported `pyserial 3.5` successfully.

## 6. Serial protocol explanation

All messages are ASCII and newline terminated.

Python commands:

```text
PING
TORQUE_STATUS
TORQUE_ON
TORQUE_OFF
READ_ANGLES
HOME
MOVE_MOTORS,351.000,1.000,1.000,90.000
```

OpenCR responses:

```text
OK,PONG
OK,READY
TORQUE:ON
TORQUE:OFF
ANGLES,351.000,1.000,1.000,90.000
MOVING
DONE
ERROR,TORQUE_OFF
ERROR,BAD_COMMAND
ERROR,INVALID_ANGLE
ERROR,TIMEOUT
ERROR,BUS,...
```

For movement commands, OpenCR first replies `MOVING`, performs slow interpolation, then replies `DONE`.

## 7. FK explanation

The FK model is exactly the project model:

```python
a2 = theta2
a3 = theta2 + theta3
a4 = theta2 + theta3 + theta4
radial = L2_X*cos(a2) + L3_X*cos(a3) + L4_X*cos(a4)
z = Z_BASE + L1_Z + L2_Z*sin(a2) + L3_X*sin(a3) + L4_X*sin(a4)
x = radial*cos(theta1)
y = radial*sin(theta1)
```

Calibration is centralized in `config.py`. The measured home is:

```text
ID11=351, ID12=1, ID13=1, ID14=90
```

The verified FK output is:

```text
X=143.523 mm, Y=0.000 mm, Z=208.985 mm
```

## 8. IK explanation

The IK solver uses fixed:

```python
DEFAULT_THETA4 = 90.0
```

XYZ alone does not uniquely determine all four arm joints. The solver calculates `theta1 = atan2(y, x)`, reduces the fixed-wrist chain into a geometric planar problem, generates possible shoulder/elbow candidates, rejects invalid candidates, runs FK on each valid candidate, calculates Cartesian error, and selects the best candidate close to the current or home posture.

Because the project FK has separate `L2_X*cos(theta2)` and `L2_Z*sin(theta2)` terms, the solver uses a bounded one-dimensional geometric root solve for shoulder candidates instead of the old nested `theta2/theta3` brute-force grid.

## 9. How to use the GUI

Manual teaching:

```text
CONNECT
TORQUE ON
HOME if desired
TORQUE OFF
Manually move the arm
READ MOTOR ANGLES
GET XYZ
```

Move to XYZ:

```text
TORQUE ON
Enter Target X/Y/Z
MOVE TO XYZ
```

Return home:

```text
TORQUE ON
HOME
```

`READ MOTOR ANGLES` and `GET XYZ` are snapshots only. There is no continuous polling loop for manual teaching.

## 10. Safety checks

Python rejects movement if:

```text
serial is disconnected
torque is OFF
XYZ input is invalid
values are NaN or infinite
IK target is unreachable
IK has no valid solution
FK validation error exceeds tolerance
motor angles are invalid
OpenCR returns an error or timeout
```

OpenCR rejects movement if torque is off, if angles are invalid, or if the DYNAMIXEL bus reports an error. `TORQUE_ON` synchronizes motor goals to present positions before enabling torque, so the arm should not jump when torque is restored.

## 11. Test procedure

Run:

```powershell
.\venv\Scripts\python.exe openmanipulator_project\python_app\run_tests.py
```

Optional if `pytest` is installed:

```powershell
.\venv\Scripts\python.exe -m pytest openmanipulator_project\python_app\tests -q -s
```

## 12. Expected output

Local test output observed:

```text
HOME FK actual: X=143.523, Y=0.000, Z=208.985
HOME IK ROUND TRIP total error=0.000000 mm
SECOND TARGET IK ROUND TRIP total error=0.000000 mm
raw 1024 -> 90.000 deg
90 deg -> 1024 raw
All checks passed.
```

For the second target, this controller uses fixed `theta4=90 deg`. The supplied roll/pitch/yaw data is orientation information and is not uniquely represented by this XYZ-only controller.

## 13. Known limitations

This uses your current experimental FK/calibration, not an official universal ROBOTIS zero model. Before production use, compare against the official OpenMANIPULATOR-X URDF, ROBOTIS kinematics/model, MoveIt configuration, and measured real robot positions.

The firmware has not been compiled on this machine because the Arduino/OpenCR toolchain is not available here. The code uses the standard ROBOTIS `Dynamixel2Arduino` API pattern already present in your local sketches.

## 14. Recommended next step

Upload the firmware, open the GUI, test `PING`/connect, run `READ MOTOR ANGLES` with torque off, then test `TORQUE ON -> HOME` while physically supporting the arm. After that, validate one small XYZ move near home before testing larger targets.
