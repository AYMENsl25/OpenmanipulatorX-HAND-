# OpenMANIPULATOR-X calibrated IK/FK, teach and replay

This project uses the calibration measured on the remounted robot. The straight,
raised REST encoder readings define mathematical `q1=q2=q3=q4=0`:

| Joint | REST RAW | Direction |
|---|---:|:---:|
| ID11 | 1917 | + |
| ID12 | 2046 | + |
| ID13 | 4049 | + |
| ID14 | 0 | + |

The official ROBOTIS chain dimensions are retained. The two named poses are:

| Pose | Calibrated joints q1,q2,q3,q4 (deg) | TCP XYZ (mm) |
|---|---|---|
| REST (straight/raised) | `0, 0, 0, 0` | official frame `286,0,204.5`; working tip `325.7,0,204.5` |
| WORK (tool down) | `0, 0, 0, 82.881` | working tip `180.536,0,40.077` |

For this mounting, ID14 encoder zero is calibrated q4 zero. The previously
saved ID14 value `82.881 deg` is therefore q4=`82.881 deg`, not q4=`0`. The
experiment uses a gripper-tip TCP approximately `39.7 mm` beyond the official
ROBOTIS gripper-frame origin. This is why the WORK tip is near X=`180 mm`,
Z=`40 mm` while the official REST frame remains X=`286 mm`, Z=`204.5 mm`.

The Z value is always calculated from the complete arm pose. In the base frame,
q2, q3, and q4 contribute together to height; q1 rotates that pose around the
base and normally does not directly change Z. The ID13 direction is positive for
this remounted robot: decreasing ID13 encoder degrees produces negative q3 and,
for the recorded upward test, an increasing calculated Z.

## Files to use

- OpenCR firmware:
  `opencr_firmware/OpenManipulatorXYZController/OpenManipulatorXYZController.ino`
- Python GUI entry point: `python_app/main.py`
- Saved manual readings: `manual_poses.xlsx`
- Full taught data: `trajectory_full.xlsx`
- Cartesian replay source: `trajectory_xyz.xlsx`

Replay always reads XYZ only and calculates new IK angles. Recorded joint
angles are retained for analysis and are never replayed directly.

## Install and start

Upload the firmware from Arduino IDE using board **OpenCR** and port **COM7**.
The USB Serial Monitor and Python GUI cannot use COM7 at the same time, so close
Serial Monitor before connecting the GUI.

```powershell
.\venv\Scripts\python.exe -m pip install pyserial openpyxl
.\venv\Scripts\python.exe openmanipulator_trajectory_project\python_app\main.py
```

The GUI defaults to COM7 when it is available.

## First hardware test

1. Support the arm and clear the workspace.
2. Upload the new OpenCR sketch. Startup configures IDs 11-14 for Extended
   Position Mode and leaves torque OFF.
3. Close Arduino Serial Monitor.
4. Start the GUI, select COM7 and press **CONNECT**.
5. Press **CAPTURE + SAVE MANUAL POSE**. It keeps torque OFF, reads the current
   motor angles, calculates calibrated q and FK XYZ, and appends the capture to
   `manual_poses.xlsx`.
6. Move the arm manually and capture at least three more poses.
7. Inspect the saved angles and XYZ before enabling movement.

## REST and WORK test

1. Put the robot close to the straight raised REST pose manually.
2. Press **TORQUE ON**. The firmware first copies the current positions into
   the goal registers, then enables torque, so it holds without jumping toward
   an old goal.
3. Press **REST (STRAIGHT)** and accept the movement confirmation.
4. Press **CAPTURE + SAVE MANUAL POSE** and confirm q is near zero. The working
   tip XYZ is near `(325.7,0,204.5) mm`; the official gripper frame is
   `(286,0,204.5) mm`.
5. Clear the table, support the arm, press **WORK (DOWN)**, and confirm the
   captured pose is near q=`(0,0,0,82.881) deg` and working-tip
   XYZ=`(180.5,0,40.1) mm`.

REST, WORK and every commanded target are checked against these provisional limits:

```text
q1: -90 to +100 deg
q2: -15 to  +85 deg
q3: -60 to  +90 deg
q4: -45 to +100 deg
```

## Teach and trajectory test

1. Support the arm and press **START TEACHING**. Torque turns OFF.
2. Move the arm slowly by hand for 2-5 seconds.
3. Press **STOP TEACHING**. The program saves both workbooks and deliberately
   leaves torque OFF.
   If either normal workbook is open in Excel, Windows locks it and the program
   saves a matched timestamped pair instead. Close Excel before recording if
   you want the normal `trajectory_full.xlsx` and `trajectory_xyz.xlsx` names.
4. Press **LOAD XYZ TRAJECTORY** and select `trajectory_xyz.xlsx`.
5. Press **VALIDATE XYZ ONLY**. Continue only when every point is valid and the
   maximum IK-to-FK error is at most 2 mm.
6. Press **TORQUE ON**, then **GO TO TEACH START**.
7. Start with speed scale `0.25`, press **PLAY XYZ TRAJECTORY**, and accept the
   movement confirmation.
8. Keep a hand near power. Press **STOP** immediately if motion is unexpected.

The current Cartesian command envelope keeps the proven X/Y bounds. Z is not
box-limited because this remounted arm's physical work surface is below the
ROBOTIS-model Z=0 base-plane reference:

```text
X:   0 to 300 mm
Y: -170 to 170 mm
Z:   no Cartesian box limit
```

An XYZ point inside the X/Y box is not automatically reachable. IK, calibrated
joint limits, motor limits, and the FK reconstruction check must all pass.

## Offline checks

```powershell
.\venv\Scripts\python.exe openmanipulator_trajectory_project\python_app\run_tests.py
.\venv\Scripts\python.exe -m compileall -q openmanipulator_trajectory_project\python_app
```

These checks validate calculations and file generation only. Arduino Verify,
upload, collision clearance and physical robot behavior remain hardware gates.

The active GUI also has a **Move to one XYZ point** panel. `PREVIEW NEAREST IK`
reads the current motor pose and selects the valid IK solution with the smallest
weighted joint movement. `MOVE TO XYZ` repeats the calculation, shows every
joint delta, and requires confirmation before sending motor commands.

The 12 mm joint-1 base offset rotates with q1 in both FK and IK. Workbooks made
with kinematics v3 predate this correction and should not be replayed as v4 data.
