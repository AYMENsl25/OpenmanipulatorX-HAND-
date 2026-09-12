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
| REST (straight/raised) | `0, 0, 0, 0` | official frame `286,0,204.5`; finger-center TCP `318.7,0,204.5` |
| WORK (tool down) | `0, 0, 0, 82.881` | finger-center TCP `179.668,0,47.023` |

For this mounting, ID14 encoder zero is calibrated q4 zero. The previously
saved ID14 value `82.881 deg` is therefore q4=`82.881 deg`, not q4=`0`. The
experiment uses the center between the two finger tips as its TCP. A physical
ground check at X=`180`, Y=`0` found that this TCP was `7 mm` above ground when
the previous model reported Z=`0`. The old `39.7 mm` extension was therefore
shortened to `32.7 mm`, making total L4=`158.7 mm`. WORK is now near X=`179.7`
mm, Z=`47.0 mm`; the official REST frame remains X=`286 mm`, Z=`204.5 mm`.

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

On every GUI connection, Python requests `JOINT_LIMITS` from OpenCR and compares
all four firmware limits with its own configuration. Connection is rejected with
an explicit outdated/mismatch message if the wrong sketch is still installed;
this prevents a green offline IK result from being rejected later by firmware.
The response also reports the encoder-boundary tolerance. The physical q4 limit
remains 100 degrees, but the firmware permits approximately half of one RAW
count (0.045 degrees) because an exact 100-degree command quantizes to about
100.020 degrees on a 4096-count encoder.

```powershell
.\venv\Scripts\python.exe -m pip install pyserial openpyxl
.\venv\Scripts\python.exe openmanipulator_trajectory_project\python_app\main.py
```

The GUI always starts with COM7 as the lab's known OpenCR port. Windows owns
the actual COM assignment; if Device Manager does not currently show COM7,
the sketch or GUI cannot recreate that port until Windows detects the board.

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

## Real-time angle and point-experiment tools

The point experiment is separate from trajectory recording/playback. The GUI
now reads one coherent robot state containing the four encoder RAW counts,
motor degrees, calibrated joints, physical TCP XYZ and internal FK XYZ. Use
**READ ANGLES NOW** for one sample or **START LIVE READ** for continuous samples.
The interval is configurable from 100 to 5000 ms; the default is 250 ms.

Every GUI session creates a new directory under `logs/` containing:

- `history.jsonl`: connection, validation, planned motion and completed phase history;
- `errors.jsonl`: exceptions with diagnostic context and traceback;
- `telemetry.csv`: time-series RAW, motor degrees, q1-q4, physical/internal XYZ,
  commanded point and measured Cartesian error.

The OpenCR sketch must be uploaded again to enable the new `READ_STATE` command
and RAW-count telemetry. The Python app remains compatible with the older
sketch by falling back to degree-only `READ_ANGLES` reads.

### P01-P07 analysis and guarded automation

Press **ANALYZE ALL POINTS** before any motion. It performs the complete
physical-frame transform, soft-workspace, IK, calibrated joint limit, motor
limit and FK round-trip checks. The table displays physical XYZ, transformed
internal XYZ, selected touch joints, motor targets and FK error.

The user-authorized q1/ID11 experiment limit is now -110 to +110 degrees. P01
and P07 require approximately -103.57 and +103.57 degrees, so all seven points
pass the offline approach/touch/retract validation. The q2-q4 limits remain
unchanged. Upload the current OpenCR sketch before testing because the firmware
independently enforces the same q1 limit.

For a controlled first experiment:

1. Keep the robot supported, clear the complete volume and keep a hand near
   **STOP POINT TEST** or the power switch.
2. Connect on COM7, press **TORQUE ON**, then **START LIVE READ**.
3. Press **ANALYZE ALL POINTS**. Confirm all P01-P07 rows are green/READY.
4. Select one reachable point (start with P04) and press **RUN SELECTED POINT**.
5. Review the confirmation. The commanded cycle is approach 50 mm above the
   point, touch at calibrated ground Z=0, dwell 0.5 s, then retract 50 mm.
6. For the first physical ground commissioning, stop before contact if the tool
   appears lower than expected. Z=0 is calibration-based and there is no force
   sensor in this software.
7. Inspect the session's telemetry and errors. Continue with one point at a
   time before using **RUN ALL REACHABLE**.

**RUN ALL REACHABLE** preserves the requested P01-P07 order and commands only
points that pass every check. Each point is replanned
from a fresh measured encoder state. After every approach/touch/retract phase,
the app reads the robot again. Error above 5 mm is recorded as a warning; error
above 10 mm is a hard stop. After every completed selected or full-run point,
the robot returns to the calibrated WORK pose before proceeding.

## REST and WORK test

1. Put the robot close to the straight raised REST pose manually.
2. Press **TORQUE ON**. The firmware first copies the current positions into
   the goal registers, then enables torque, so it holds without jumping toward
   an old goal.
3. Press **REST (STRAIGHT)** and accept the movement confirmation.
4. Press **CAPTURE + SAVE MANUAL POSE** and confirm q is near zero. The working
   finger-center TCP is near `(318.7,0,204.5) mm`; the official gripper frame is
   `(286,0,204.5) mm`.
5. Clear the table, support the arm, press **WORK (DOWN)**, and confirm the
   captured pose is near q=`(0,0,0,82.881) deg` and working-tip
   XYZ=`(179.7,0,47.0) mm`.

REST, WORK and every commanded target are checked against these provisional limits:

```text
q1: -110 to +110 deg (authorized P01/P07 experiment range)
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

## ID11-centered physical experiment frame

All GUI entries, saved workbook XYZ values, and replay targets now use the
physical experiment frame. Its X/Y origin is the center axis of ID11:

```text
+X = forward       -X = behind ID11
+Y = physical right  -Y = physical left
Z=0 = calibrated physical ground; +Z = upward
```

The tested internal FK/IK equations are unchanged. The configurable transform
maps physical `(X,Y,Z)` to internal `(X,-Y,Z)`. PREVIEW displays both values
before motion. The frame, transform, soft limits, ground plane, and P01-P07 are
defined once in `config/experiment_config.json`.

The configurable soft Cartesian envelope is:

```text
X: -100 to 350 mm
Y: -200 to 200 mm
Z: no arbitrary maximum; targets below physical ground Z=0 are blocked
```

An XYZ point inside this box is not automatically reachable. The enforced
sequence is physical-to-internal transform, soft envelope, IK geometry,
configured calibrated joint limits, motor limits, FK reconstruction, preview,
and only then confirmed hardware motion. P01 and P07 now pass using the
authorized q1 range of -110 to +110 degrees; a target outside that range remains
blocked rather than forced.

Use the measured-point selector in the workspace preview to load P01-P07. The
plot shows negative X, places ID11 at `(0,0)` inside the plot, and highlights
the loaded point. Always press **PREVIEW NEAREST IK** and inspect both physical
and internal XYZ before considering **MOVE TO XYZ**.

## Offline checks

```powershell
.\venv\Scripts\python.exe openmanipulator_trajectory_project\python_app\run_tests.py
.\venv\Scripts\python.exe -m unittest discover -s openmanipulator_trajectory_project\python_app\tests -p "test_*.py" -v
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
