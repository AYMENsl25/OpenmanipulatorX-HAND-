# P01-P07 Ground-Point Experiment Engineering Analysis

## Scope and preserved calibration

This experiment uses the existing tested FK, IK, encoder calibration, TCP length,
REST/WORK poses without changing their kinematic mathematics. The q1 safety
limit was expanded with user authorization from `-90..+100` to `-110..+110`
degrees for the P01/P07 experiment; q2-q4 remain unchanged. It does not
calculate or replay an Excel trajectory. Each named point is solved and executed
as an independent guarded Cartesian point test.

Physical coordinates use the center axis of ID11 as `(X,Y)=(0,0)`: +X forward,
+Y right and +Z upward. The current transform is physical `(X,Y,Z)` to internal
`(X,-Y,Z)`. Z=0 is the calibrated ground plane.

## Offline reachability result

The table reports the deterministic offline touch solution. During hardware use,
the GUI recalculates the nearest valid solution from the currently measured joint
pose, so an equivalent valid branch can have different q2-q4 values.

| Point | Physical XYZ mm | Internal XYZ mm | Result | Touch q1,q2,q3,q4 deg | Touch ID11,12,13,14 deg | FK error mm |
|---|---|---|---|---|---|---:|
| P01 | (-35, +145, 0) | (-35, -145, 0) | READY | (-103.570, 1.703, 16.266, 75.000) | (64.916, 181.527, 12.135, 75.000) | <0.000001 |
| P02 | (+146, +145, 0) | (+146, -145, 0) | READY | (-44.803, 11.093, 4.770, 65.000) | (123.683, 190.917, 0.639, 65.000) | <0.000001 |
| P03 | (+257, +145, 0) | (+257, -145, 0) | READY | (-29.432, 26.523, -8.112, 40.000) | (139.055, 206.347, 347.757, 40.000) | <0.000001 |
| P04 | (+257, 0, 0) | (+257, 0, 0) | READY | (0.000, 20.952, -6.065, 55.000) | (168.486, 200.776, 349.805, 55.000) | 0.000000 |
| P05 | (+257, -145, 0) | (+257, +145, 0) | READY | (+29.432, 26.523, -8.112, 40.000) | (197.918, 206.347, 347.757, 40.000) | <0.000001 |
| P06 | (+146, -145, 0) | (+146, +145, 0) | READY | (+44.803, 11.093, 4.770, 65.000) | (213.289, 190.917, 0.639, 65.000) | <0.000001 |
| P07 | (-35, -145, 0) | (-35, +145, 0) | READY | (+103.570, 11.809, 4.221, 85.000) | (272.057, 191.633, 0.090, 85.000) | <0.000001 |

P01 geometrically requires q1 approximately -103.570 degrees and P07 requires
approximately +103.570 degrees. The authorized -110..+110 degree range leaves
about 6.43 degrees of software margin at both points. Raising Z would not have
changed q1 because q1 is determined by the target's horizontal direction.

## Automated point cycle

For every READY point the GUI performs:

1. Read one coherent encoder state and use its calibrated joints as the IK reference.
2. Re-plan the point and repeat Cartesian, geometry, joint, motor and FK checks.
3. Move to `(X,Y,Z+50 mm)` as APPROACH.
4. Read the actual encoders and reconstruct physical/internal FK.
5. Move to `(X,Y,Z=0)` as TOUCH and dwell 0.5 seconds.
6. Read and verify the actual state.
7. Move to `(X,Y,Z+50 mm)` as RETRACT and verify again.
8. Log a warning if measured physical TCP error exceeds 5 mm; stop if it
   exceeds 10 mm.
9. Return to the calibrated WORK pose before ending a selected-point run or
   continuing to the next point in a full run.

The run preserves P01-P07 order and all seven points now pass the configured
limit checks. `RUN SELECTED POINT` is the required commissioning
path before attempting the multi-point run. Both modes return to WORK after
every successfully completed point.

## Real-time measurements

The new OpenCR `READ_STATE` response samples RAW positions for IDs 11-14 once,
then reports RAW, motor degrees and calibrated joint degrees from that same state.
Python derives physical and internal FK from it. Reads retry transient controller
or bus failures three times with a 50 ms delay. Serial request/response operations
are locked so live monitoring cannot consume a motion command's response.

The `JOINT_LIMITS` handshake is checked during connection. Python refuses the
connection if OpenCR still reports old or different limits, preventing offline
`READY` results from disagreeing with firmware motion protection.

Joint-limit enforcement is encoder-quantization aware. A requested boundary is
rounded to an integer RAW count, so the firmware permits half a RAW count plus a
small floating-point guard (about 0.045 degrees) during its post-conversion
limit check. This permits q4=100 degrees mapping to approximately 100.020 degrees
without changing the configured or commandable q4 maximum.

The GUI supports a single read or continuous monitoring from 100 to 5000 ms
(250 ms default). During motion, serial locking may delay a sample; this is safer
than interleaving protocol replies. Monitoring resumes automatically after the
move transaction finishes.

## Debug records

Each GUI launch creates `logs/<UTC-session-id>/` with:

- `history.jsonl`: analysis, confirmation, command, completion, cancellation,
  stop and session lifecycle records;
- `errors.jsonl`: exception type, message, traceback and diagnostic context;
- `telemetry.csv`: elapsed time, RAW counts, motor degrees, q1-q4, physical and
  internal XYZ, target XYZ, measured position error, point and phase metadata.

All records are append-only, thread-safe and flushed immediately. Together these
files distinguish an IK/limit rejection from a serial bus error, following error,
coordinate-frame problem or physical calibration mismatch.

## Hardware gates

- Arduino Verify/upload remains required; the Python tests cannot compile OpenCR firmware.
- Close Arduino Serial Monitor before connecting the GUI to COM7.
- There is no force sensor in this program. Z=0 is a calibration coordinate, not
  force-confirmed surface contact. Commission P04 first with the tool supported
  and stop before contact if physical ground does not agree with the calibration.
- P01/P07 software validation now passes, but first hardware commissioning must
  check ID11 cable slack, base clearance and mechanical interference at both
  sides before running the complete seven-point sequence.
