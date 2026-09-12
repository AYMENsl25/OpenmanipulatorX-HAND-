# Cartesian Teach and Replay Implementation Plan

**Goal:** Add OpenCR teaching, dual Excel recording, Cartesian-only replay, IK/FK validation, and an expanded Python GUI while preserving the existing FK/IK equations and separating ID14 physical zero from custom home.

**Architecture:** OpenCR records motor angles and FK-generated XYZ while torque is off, then sends the complete recording to Python. Python writes both workbooks, loads only the XYZ workbook for replay, validates each point through the existing IK/FK functions, and sends `XYZ,time,x,y,z` commands to OpenCR. OpenCR performs motor IO, IK, and slow trajectory execution.

**Global constraints:** ID14 physical zero remains 0 degrees; custom home remains ID14=90 degrees; playback never reads J1-J4 from the full workbook; FK/IK equations remain unchanged; initial playback speed is 0.25; IK validation threshold is 2 mm; teaching samples at 20 Hz with a 30 second maximum.

### Task 1: Add trajectory data and workbook handling

- [ ] Add typed trajectory points, dual workbook writers, XYZ workbook loader, and validation statistics.
- [ ] Add tests for exact workbook headers and Cartesian-only loading.

### Task 2: Extend serial control

- [ ] Add teaching commands, trajectory reception, XYZ playback transmission, and stop handling.
- [ ] Add tests for protocol parsing and playback command generation without requiring hardware.

### Task 3: Extend the GUI

- [ ] Add teaching, save, load, validation, playback, stop, status, and diagnostics controls.
- [ ] Keep all actions one-shot or event-driven without interface-spamming polling.

### Task 4: Extend OpenCR firmware

- [ ] Add teaching buffer, 20 Hz FK sampling, trajectory transfer, XYZ playback, IK validation, and emergency stop sections.
- [ ] Keep motor IDs 11-14 and existing motor control conventions unchanged.

### Task 5: Verify and document

- [ ] Run dependency-free kinematics tests, trajectory tests, syntax checks, and workbook smoke tests.
- [ ] Update README with installation, calibration distinction, recording, validation, and replay procedure.
