# Automatic repeatability collection

## Runs with the wrist camera

Use the GUI's **Camera payload** profile and record it in the workbook `Run Config` sheet. The current profile is speed `<= 0.50`, at least `10` touch samples, at least `100 ms` between samples, `0.75 s` settling, and `1.00 s` touch dwell. The orange `2.0 deg` line is a payload-aware warning for investigation. The red `10.0 deg` line remains the hard stop and is not loosened.

Interpret low within-touch FK noise together with a repeatable signed XYZ offset as systematic payload deflection or calibration/TCP bias, not random encoder noise. Do not subtract that bias from FK/IK until it has been checked against an independent physical measurement method.

Upload `opencr_firmware/OpenManipulatorXYZController/OpenManipulatorXYZController.ino` from this project to OpenCR. This version supports SET_SPEED. Close Arduino Serial Monitor before connecting the GUI.

Launch from the lab workspace:

```powershell
.\venv\Scripts\python.exe openmanipulator_repeatability_project\python_app\main.py
```

Connect to the actual OpenCR port (COM7 if Windows still assigns COM7), enable torque, move to WORK, and open REPEATABILITY SETUP. Select the points, repetition count, touch sample count, sample interval, and speed. Speed 1.0 retains the existing motion timing; 0.5 doubles the interpolated move duration. The supported range is 0.25 to 1.0. The selected speed applies to approach, touch, retract, and return to WORK during the run. It resets to 1.0 when the run ends. Existing joint, motor, and error-stop checks still apply.

Start with one point, one repetition, and speed 0.5. Then test several points and repetitions. The robot samples automatically at each touch, retracts, and returns to WORK after every point. There is no manual measurement pause.

Each session saves files under `experiment_results/<session-id>/`:

- `repeatability_point_results.xlsx`: target XYZ, averaged encoder FK XYZ (`fk_actual_x_mm`, `fk_actual_y_mm`, `fk_actual_z_mm`), signed errors (actual minus target), distance error, and within-touch sample noise. The `fk_actual_*` values remain the average of the configured FK samples for one touch; only their names changed. Includes planned approach/touch/retract/WORK path, run settings, and per-point error summary.
- `repeatability_point_results_motor_angles.xlsx`: planned IK motor angles versus read motor angles; planned calibrated q versus read q; angle errors and mean RAW counts.
- `repeatability_telemetry_details.xlsx`: individual samples including position, velocity, current, PWM, voltage, temperature and hardware status, plus run events.

Open the **Error Plots** tab in the point results file. It contains signed XYZ error by touch, distance error and sample noise by touch, and mean/maximum error per point. The motor file contains a **Motor Error Plot** tab with calibrated joint tracking errors. Excel charts update from the saved rows every time the files are regenerated. For repeatability across cycles, use the per-point XYZ standard deviations in **Point Error Summary**; with one touch the standard deviation is zero and does not establish repeatability.

Motor degrees describe encoder position around the motor's revolution. Calibrated q degrees describe angle relative to the calibrated zero with its configured direction. FK uses calibrated q. Motor display errors wrap to the shortest equivalent angular difference; q errors use the continuous calibrated angle difference.

These results measure encoder-based tracking and repeatability. They do not independently measure physical TCP accuracy, ground contact force, flex, or calibration bias. A calculated Z=0 is not a contact sensor. Close output workbooks while collecting; if a file is locked, a timestamped recovery workbook is saved.

The stable snapshot and FK/IK equations are unchanged. The speed command needs the updated firmware; an older sketch rejects the setting before the experiment moves.

## Motor arrival tolerance for collection

The current OpenCR sketch attempts the original 8-count (0.703-degree) arrival threshold for up to three seconds after interpolation. If it cannot settle that closely, it accepts a maximum motor error of 10 degrees. Python also rejects motor errors above 10 degrees. Upload this sketch again to enable the wider collection tolerance. It applies to all motor-arrival checks in this experimental project, including REST and WORK. DONE means arrival accepted within the collection tolerance, not exact target achievement.

Repeatability approach, touch and retract arrivals above 0.703 degrees produce motor-tracking warnings in the GUI, history log, and workbook Run Events. The touch dataset keeps the actual encoder readings and FK errors. Existing joint/motor limits, STOP, communication fault checks and the separate hard TCP-error stop remain active, so these can still stop a run. A ten-degree motor error can create a large TCP displacement; this setting enables data collection and does not establish acceptable accuracy.
