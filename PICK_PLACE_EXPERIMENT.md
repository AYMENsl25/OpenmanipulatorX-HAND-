# Camera-guided pick/place experiment (supervised)

The new **5 Camera Pick and Place** tab queues one or two detected cubes, lets
you put them in order, and stores a separate destination X/Y for each. Camera
calibration provides **X/Y only**. Enter the measured pick TCP Z, place TCP Z,
and travel TCP Z yourself. X is measured from the rear of ID11; the existing
25 mm rear-to-axis conversion is provisional. The software does not infer cube
height, jaw width, grasp force, or collision clearance from an image.

Before use, upload the updated
`openmanipulator_repeatability_project/opencr_firmware/OpenManipulatorXYZController/OpenManipulatorXYZController.ino`.
The updated OpenCR advertises `GRIPPER_RAW_V2` only when ID15 initializes. The
existing four-motor controller continues to work if ID15 is absent, but the
pick experiment refuses to start. No firmware was uploaded by this change.

1. The UI now starts with a shorter opening target of **1800 RAW** and a cube
   grasp target of **2650 RAW**. You may enter **2000 RAW** for a still smaller
   opening. Firmware accepts a configured open target from **1250–2200 RAW**;
   that range is not a measured jaw-width calibration. Before each pick, the
   app moves the jaw near the selected open target and verifies the position.
   Confirm the chosen opening clears the real cube before running.
2. The separate `PRESENT_CURRENT` cutoff is **not** gripper position. The UI
   starts at 200 current RAW as a software ceiling; entering 1200 current RAW
   remains blocked. Confirm a safe current threshold on the real gripper. If
   ID15 times out while closing, OpenCR holds its last RAW position and the
   app pauses. Read the displayed RAW and **only** press CONFIRM GRIP if the
   cube is visibly held; otherwise STOP. A timeout alone is not a grasp
   detection. Measure a suitable close target/current cutoff before retrying.
3. **From Camera Tab (Quick Pick)**: Click **PICK IT UP** in a valid cube row, or
   select a row and click **PICK IT UP (SELECTED CUBE)…**. The dialog asks for:
   - Destination X rear and Y (mm)
   - The 3 Z heights: Pick TCP Z, Place TCP Z, and Travel TCP Z (mm)
   - Open and close ID15 RAW positions (defaults 1800 and 2650)
   The three Z values and destination have no assumed defaults. **PICK IT UP —
   REVIEW** takes you to tab 5 for path verification and final motion
   confirmation; it does not start the robot by itself. The earlier queue/order
   controls remain available for one or two cubes. The dialog now checks that
   both above-place and release poses satisfy IK before queuing.
4. Measure and enter pick/place/travel TCP Z. Travel Z must be at least the
   start TCP height and 20 mm above pick and place TCP heights; this is a software check, not
   a collision guarantee. Start the arm at CENTER SCAN. Preview the
   full path. The plan commands direct goal poses smoothly interpolated via OpenCR's
   built-in minimum-jerk cosine S-curve profile at 50 Hz.
5. Verify the *entire physical path*, check the verification box, then run
   under supervision with emergency stop available. Before each pick the
   program verifies ID15 open band. It lowers to pick Z, closes toward the grasp RAW
   target with current cutoff, settles, and lifts to travel Z, moves to destination,
   lowers to place Z, opens gripper, retracts, and returns through WORK to
   CENTER SCAN after the queued placement(s). The camera-loaded speed remains
   capped at the configured 0.5 scale; fewer intermediate moves make it
   smoother without raising that hardware limit.
   No slow CSV trajectory data is recorded during pick/place.

The repeatability experiment points use **ID11-axis X**, while the camera
pick/place destination field uses **ID11-rear X**. With the provisional 25 mm
offset, rear X=20 mm means axis X=-5 mm. The reported rear X=20, Y=100 mm
drop target cannot be reached at Z=80–120 mm under the current IK limits;
changing joint limits would not make it safe. For a side drop at about Z=100 mm,
rear X=50, Y=170, travel Z=120 mm passes an offline IK/path check for one
representative pick, but is **not** a confirmed physical drop location. Measure
the real surface and clearance, then preview your exact source/destination
before commanding motion. No Y calibration was changed.

Rear X=5 mm already lies inside the configured software X envelope (its
ID11-axis X is -20 mm; the soft minimum is -100 mm). It also passes an offline
representative route at Y=170, place TCP Z=100, travel TCP Z=120 mm. It does
**not** pass the same route at Y=100 mm. This is a Y/Z-dependent reachability
constraint, not an X slider limit; the conservative Cartesian IK joint limits
remain unchanged. Do not use either candidate until the box position, floor,
rim height, gripper clearance, and entire path are physically checked.

For the camera-dialog example rear X=50, Y=100, pick TCP Z=0, place TCP Z=5,
travel TCP Z=90 mm, the release point at Z=5 is reachable, but the current
Cartesian IK reaches only about Z=52 mm at the same X/Y. The top-down plan
needs at least about Z=67 mm to clear WORK. The dialog reports these values;
reducing travel Z below the required clearance is not a safe fix. An offline
plan using the screenshot's source XYZ (rear X=240.5, Y=36.5, pick Z=0) and
rear X=50, Y=170, place Z=5, travel Z=90 passes mathematically. A fixed box
at Y=100 instead needs measured box/rim geometry before a different entry
path can be designed or approved.

With a later camera observation at rear pick X=236.5, Y=-62.3, the entered
destination rear X=50, place TCP Z=5 and travel TCP Z=80 is rejected at
destination Y=100 but passes the offline full-route planner at Y=160. Y=100
is still inside the configured +/-200 mm Y envelope: at X-axis=25, Y=100,
the high approach is not available within validated IK joint limits. Moving
the destination to Y=160 changes the radial reach, not the Z setting. The
camera shortcut now checks the full offline route before saying it is ready
for physical review; passing this check is not a collision-clearance proof.

A later screenshot with a **different detected cube** at rear pick X=149.1,
Y=-24.1 and travel Z=80 fails at `ABOVE_PICK`, before destination planning.
At that pick X/Y, the validated IK samples TCP Z=0–71 mm only. Changing
destination Y from 100 to 150 cannot repair this same-source, same-travel-Z
failure. A representative offline route with destination rear X=50, Y=150
passes at travel Z=70 but not Z=80; the narrow 67–71 mm pick approach still
requires physical object/box clearance measurements before any robot run.

## Lower travel heights (updated)

The former requirement of WORK Z plus 20 mm has been removed. All GUI and
planner checks now use `max(WORK Z, pick Z + 20, place Z + 20)`. With pick
Z=0 and place Z=5, the minimum is approximately 47.024 mm, so travel Z=50
is accepted. A full offline route from rear pick (236.5, -62.3) to rear
destination (50, 100), with pick/place/travel Z=(0, 5, 50), passes the planner.
Earlier notes above referring to a 67 mm minimum describe the previous rule.
Actual held-cube, box-rim and gripper clearance must still be checked before
running. IK, motor limits and tracking-error stops remain enforced.

## ID11 target-not-reached versus destination reachability

An OpenCR response such as `ERROR,TARGET_NOT_REACHED,ID11,ERROR_DEG,17.490`
after `ABOVE_PICK` means a motor command was accepted but the ID11 encoder
remained 17.49 degrees from its target after the firmware wait. It is **not**
the same as an offline `UNREACHABLE / JOINT LIMIT` result, and changing the
destination Y does not repair a failed pick-side ID11 motion. Another reported
ID11 miss of 10.635 degrees makes repeated physical tracking trouble a
possibility; the logs alone cannot identify whether the cause is load,
obstruction/cable interference, power, an actuator fault, or settling. The
pick/place worker now aborts without retry and captures a read-only motor and
ID11 telemetry snapshot when this specific failure occurs. Firmware may still
be holding its last goal; no automatic recovery or limit change is attempted.

With destination rear X=50, Y=100, place Z=5, travel Z=80, the sampled
Cartesian reachability at that destination is Z=0–52 mm, so the required
80 mm top-down waypoint cannot be reached. Y=100 is already within the
configured software Y range of -200 to +200 mm. The proposed alternative
rear X=60, Y=10 samples Z=0–25 mm at that X/Y and likewise cannot support
the Z=80 approach. Neither coordinate should be installed as an automatic
default. If a physically different box location is chosen, measure its
surface/rim and run full-route preflight with the current cube first.
