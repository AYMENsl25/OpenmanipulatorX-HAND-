# ISU XR Lab portable Windows app

Double-click `ISU-XR-OpenManipulator-Smooth.exe`. Python is not required on the target PC.
Keep the entire folder together, including `_internal`, `data`, the JSON file and
`openmanipulator_repeatability_project`. Do not move only the executable.
Use a writable folder such as Downloads or Documents, not Program Files.

The app starts disconnected. Connect the correct robot port and start the camera
manually. Existing robot limits and confirmations are unchanged. Camera calibration
is specific to the original robot/camera setup; revalidate after changing the setup.

Upload the included `firmware/OpenManipulatorXYZController/OpenManipulatorXYZController.ino`
to use transfer speeds above 1.0. Pick/place speed is adjustable from 0.25 to 2.0;
1.0 is normal transfer speed. Contact phases remain slower. The queue supports
more than two cubes; each entry retains its own destination X/Y, while Z and speed
settings apply to the entire queue. Travel Z 60 or 70 is allowed when the complete
route is reachable. CHECK DESTINATION helps diagnose pose limits. The new speed
range requires a supervised physical trial; offline tests do not measure smoothness.

Keyboard tab: click ENABLE KEYBOARD once, then hold W/S (forward/backward),
A/D (left/right), R/F (up/down). Arrow keys and Page Up/Page Down are aliases.
Release the keys to stop. Esc or Space disables movement. Leaving the keyboard
control area or changing tabs stops keyboard motion; click ENABLE KEYBOARD to resume.
Set speed before enabling (0.25–2.0). No gamepad is needed. The same live serial
firmware described below is required; if already uploaded, no new flash is needed.
At SCAN, first use MOVE TO WORK once to enter the Cartesian jog range.

Controller tab: choose MOTOR angles or XYZ, then click ENABLE CONTROLLER. In
MOTOR mode, X/Square selects the previous motor and B/Circle the next, cycling
through ID11–ID14. Right-stick up/down changes only the selected calibrated
joint angle; center to hold. This mode can start at SCAN. In XYZ mode,
right-stick vertical commands X forward/back, right-stick horizontal commands
Y left/right, and left-stick vertical commands Z up/down. The commanded axes
not being moved retain their XYZ targets rather than rebasing on encoder sag.
XYZ must start from a reachable IK pose: press Y/Triangle to request the
existing confirmed WORK move when at SCAN. Y operates only when both sticks
are centered and jogging is stopped. A/Cross toggles ID15 based on measured RAW
position, to 2650 RAW if open or 1800 RAW if closed; it also requires centered
sticks and a stopped arm jog, and disables control until re-enabled. Start
is the default STOP button. Default Logitech F710 mapping: right vertical axis
3, right horizontal axis 2, left vertical axis 1; A=0, B=1, X=2, Y=3,
Start=7. Use Detect controls for another driver mapping and check live signs.
Start physical trials at speed 0.25; range is 0.25–2.0 and default is 1.25.
MOTOR speed 1.0 is approximately 10 degrees/s; XYZ speed 1.0 is 20 mm/s.
Controller joint-stream speed remains capped at 20 degrees/s by firmware.
Both modes ramp into motion and drop old momentum on reversal. App focus loss,
disconnect, Stop, stale input, held-axis drift, joint/ground/workspace limit,
or OpenCR watchdog stops the jog.
Upload the included OpenCR firmware advertising `GAMEPAD_STREAM_V1`, then reconnect.

The app aims for one latest-target update every 50 ms and never queues a sequence
of moves or waits for arrival at every update. OpenCR interpolates the targets in
its main loop and holds if no valid update arrives for 250 ms. Actual response
depends on serial/bus latency and the motors. Target lead, joint speed, workspace,
ground and calibrated joint limits remain checked. The requested motor angle
accumulates continuously but stays within 3 degrees of measured position,
below the OpenCR 5-degree tracking limit. Reversing the stick drops any old
target backlog immediately. Each jog saves timestamped selected ID, stick input,
commanded and measured ID11–ID14 angles, joint angles, and TCP XYZ in
`openmanipulator_repeatability_project/logs/joint_jog_*.csv`. If a motor moves
opposite the request or fails to follow it, the jog stops with a diagnostic.
The three non-selected arm joints retain their command angles from jog start;
they do not track sagging encoder readings. Drift above 1.5 degrees for three
feedback samples stops the jog. Keep their torque ON: disabling loaded joints
can let the arm fall. The GUI displays the current calibrated limits (ID11 is
-110 to +110 degrees, already wider than a requested -90 to +90 range).
The OPEN ID15 and CLOSE ID15 buttons use the measured 1800/2650 RAW targets
and run only while arm jogging and other robot operations are stopped.
A blocked direction can be reversed without entering a new target. This software validation does not replace
a physical controller/robot trial.

Calibration is saved under `data/vision_calibration`; experiment results go under
`openmanipulator_repeatability_project`. Back these up before replacing a release.
This is an unsigned local build, not an installer. Only run copies from a trusted source.

Build from the workspace virtual environment with PyInstaller 6.22.3 installed:
`powershell -File openmanipulator_repeatability_project/build_windows.ps1`

Packaging smoke check: `ISU-XR-OpenManipulator-Smooth.exe --smoke-test`.
It creates the GUI hidden with serial/camera access blocked and writes
`smoke-test-ok.txt` on success or `startup-error.txt` on failure.
