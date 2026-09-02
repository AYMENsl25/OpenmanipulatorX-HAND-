#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdint.h>

// ============================================================
// OPENMANIPULATOR-X
// CALIBRATION + TEACH / REPLAY CONTROLLER - V9
// ============================================================
//
// IDs:
//   11 = Joint 1
//   12 = Joint 2
//   13 = Joint 3
//   14 = Joint 4
//   15 = Gripper
//
// Motor:
//   XM430-W350-T
//
// Communication:
//   OpenCR Serial3
//   Direction pin 84
//   Baudrate 1,000,000
//   Protocol 2.0
//
// ============================================================
//
// V9 FEATURES
//
// TEACH / REPLAY:
//
//   T = Start manual teaching
//   Y = Replay recorded trajectory forward
//   U = Replay recorded trajectory backward
//   C = Clear recorded trajectory
//
// During TEACH:
//
//   1. Press T
//   2. 5 second preparation countdown
//   3. Torque OFF on ALL FIVE motors
//   4. Manually move the robot AND gripper
//   5. ID11-ID15 recorded every 50 ms
//   6. Teaching finishes
//   7. Actual positions of ALL FIVE motors are synchronized
//      to Goal Position
//   8. Torque ON
//
// During REPLAY:
//
//   Y = replay forward
//   U = replay backward
//
// The gripper ID15 is fully part of the trajectory.
//
// ============================================================
//
// GRIPPER ID15
//
// Operating Mode:
//   OP_CURRENT_BASED_POSITION
//
// Goal Current:
//   80 raw
//
// Calibrated range:
//
//   OPEN   = 90 degrees
//   CLOSED = 234 degrees
//
// ============================================================
//
// IMPORTANT SAFETY
//
// During TEACH mode ALL FIVE motors have torque OFF.
//
// The robot can move/fall because of gravity.
//
// KEEP YOUR HAND ON THE ROBOT.
//
// Do not leave the robot unattended.
//
// ============================================================


// ============================================================
// DYNAMIXEL
// ============================================================

#define DXL_SERIAL Serial3

static const uint8_t DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(
    DXL_SERIAL,
    DXL_DIR_PIN
);


// ============================================================
// GENERAL SETTINGS
// ============================================================

static const uint32_t BAUDRATE = 1000000;

static const float PROTOCOL_VERSION = 2.0f;

static const int32_t COUNTS_PER_REV = 4096;

static const float COUNTS_PER_DEGREE =
    COUNTS_PER_REV / 360.0f;


// ============================================================
// EXTENDED POSITION MODE LIMITS
// ============================================================

static const int32_t EXTENDED_MIN =
    -1048575L;

static const int32_t EXTENDED_MAX =
     1048575L;


// ============================================================
// MOVEMENT PROFILE - ARM
// ============================================================

static const int32_t MOVE_PROFILE_VELOCITY =
    3000;

static const int32_t MOVE_PROFILE_ACCELERATION =
    1400;


// ============================================================
// HOME PROFILE - ARM
// ============================================================

static const int32_t HOME_PROFILE_VELOCITY =
    3000;

static const int32_t HOME_PROFILE_ACCELERATION =
    2000;


// ============================================================
// GRIPPER PROFILE
// ============================================================
//
// These are intentionally kept separate from the arm profile.
//
// ============================================================

static const int32_t GRIPPER_PROFILE_VELOCITY =
    20;

static const int32_t GRIPPER_PROFILE_ACCELERATION =
    10;

static const int32_t GRIPPER_GOAL_CURRENT =
    80;


// ============================================================
// STEP SIZE
// ============================================================

static const float STEP_DEGREES =
    1.0f;


// ============================================================
// POSITION TOLERANCE
// ============================================================

static const int32_t POSITION_TOLERANCE_COUNTS =
    25;

static const int32_t HOME_PATH_ENTRY_MARGIN_COUNTS =
    30;


// ============================================================
// MOVEMENT TIMEOUT
// ============================================================

static const uint32_t MOVE_TIMEOUT_MS =
    240000UL;


// ============================================================
// POSITION POLLING
// ============================================================

static const uint32_t STATUS_POLL_MS =
    50UL;


// ============================================================
// TEACH / REPLAY SETTINGS
// ============================================================

// Preparation countdown.

static const uint32_t TEACH_START_DELAY_SECONDS =
    5;


// Manual teaching duration.

static const uint32_t TEACH_DURATION_SECONDS =
    10;


// Record every 50 ms.

static const uint32_t TEACH_SAMPLE_INTERVAL_MS =
    50;


// Replay interval.

static const uint32_t REPLAY_INTERVAL_MS =
    100;


// ============================================================
// MAXIMUM TRAJECTORY MEMORY
// ============================================================
//
// 600 samples.
//
// At 50 ms:
//
// 600 * 0.05 = 30 seconds.
//
// Each sample stores:
//
//   ID11
//   ID12
//   ID13
//   ID14
//   ID15 GRIPPER
//
// ============================================================

static const uint16_t MAX_TRAJECTORY_SAMPLES =
    600;


// ============================================================
// HOME POSITIONS
// ============================================================

static const float HOME_ID11 =
    351.0f;

static const float HOME_ID12 =
    1.0f;

static const float HOME_ID13 =
    1.0f;

static const float HOME_ID14 =
    90.0f;


// ============================================================
// GRIPPER
// ============================================================

static const uint8_t GRIPPER_ID =
    15;


// Calibrated gripper limits.

static const float GRIPPER_OPEN_ANGLE =
    90.0f;

static const float GRIPPER_CLOSED_ANGLE =
    234.0f;


// ============================================================
// CALIBRATED ARM MECHANICAL LIMITS
// ============================================================

// ID11

static const float ID11_LIMIT_A =
    261.0f;

static const float ID11_LIMIT_B =
    81.0f;


// ID12

static const float ID12_LIMIT_MIN =
    83.0f;

static const float ID12_LIMIT_MAX =
    240.0f;


// ID13

static const float ID13_LIMIT_MIN =
    90.0f;

static const float ID13_LIMIT_MAX =
    240.0f;


// ID14

static const float ID14_LIMIT_MIN =
    125.0f;

static const float ID14_LIMIT_MAX =
    258.0f;


// ============================================================
// MOTOR CONFIGURATION
// ============================================================

struct MotorConfig
{
  uint8_t id;

  float homeAngle;

  float limitA;
  float limitB;

  float limitAOffset;
  float limitBOffset;
};


// ============================================================
// ARM MOTOR TABLE
// ============================================================

MotorConfig motors[] =
{
  {11, 351.0f, 261.0f,  81.0f,  -90.0f,  +90.0f},
  {12,   1.0f,  83.0f, 240.0f,  +82.0f, -121.0f},
  {13,   1.0f,  90.0f, 240.0f,  +89.0f, -121.0f},
  {14,  90.0f, 125.0f, 258.0f,  +35.0f, -192.0f}
};


static const size_t MOTOR_COUNT =
    sizeof(motors) / sizeof(motors[0]);


// ============================================================
// CONTROLLER STATE
// ============================================================

bool controllerReady =
    false;

bool emergencyStop =
    false;

bool motionHoldLatched =
    false;


// ============================================================
// TEACH / REPLAY STATE
// ============================================================

bool trajectoryRecorded =
    false;

bool trajectoryValid =
    false;

bool teachModeActive =
    false;

bool replayModeActive =
    false;


// Number of recorded samples.

uint16_t trajectorySampleCount =
    0;


// ============================================================
// TRAJECTORY SAMPLE
// ============================================================
//
// ALL FIVE motors are stored.
//
// ============================================================

struct TrajectorySample
{
  int32_t id11;

  int32_t id12;

  int32_t id13;

  int32_t id14;

  int32_t id15;
};


// ============================================================
// TRAJECTORY MEMORY
// ============================================================

TrajectorySample trajectory[
    MAX_TRAJECTORY_SAMPLES
];


// ============================================================
// NORMALIZE RAW ENCODER
// ============================================================

int32_t normalizeRaw(
    int32_t raw)
{
  raw %= COUNTS_PER_REV;

  if (raw < 0)
    raw += COUNTS_PER_REV;

  return raw;
}


// ============================================================
// RAW -> ENCODER DEGREES
// ============================================================

float rawToEncoderDegrees(
    int32_t raw)
{
  return normalizeRaw(raw) /
         COUNTS_PER_DEGREE;
}


// ============================================================
// ENCODER DEGREES -> RAW
// ============================================================

int32_t degreesToRaw(
    float degrees)
{
  return (int32_t)lroundf(
      degrees *
      COUNTS_PER_DEGREE
  );
}


// ============================================================
// FIND MOTOR
// ============================================================

MotorConfig* findMotor(
    uint8_t id)
{
  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    if (motors[i].id == id)
      return &motors[i];
  }

  return NULL;
}


// ============================================================
// ERROR REPORT
// ============================================================

void printBusError(
    const char *operation,
    uint8_t id)
{
  Serial.print(
      "BUS ERROR: "
  );

  Serial.print(
      operation
  );

  Serial.print(
      " | ID="
  );

  Serial.print(
      id
  );

  Serial.print(
      " | lib="
  );

  Serial.print(
      dxl.getLastLibErrCode()
  );

  Serial.print(
      " | status="
  );

  Serial.println(
      dxl.getLastStatusPacketError()
  );
}


// ============================================================
// READ POSITION
// ============================================================

bool readPosition(
    uint8_t id,
    int32_t &position)
{
  position =
      dxl.readControlTableItem(
          ControlTableItem::PRESENT_POSITION,
          id
      );

  if (
      dxl.getLastLibErrCode()
      != DXL_LIB_OK)
  {
    printBusError(
        "read PRESENT_POSITION",
        id
    );

    return false;
  }

  return true;
}


// ============================================================
// READ BYTE
// ============================================================

bool readByte(
    uint8_t item,
    uint8_t id,
    uint8_t &value)
{
  int32_t result =
      dxl.readControlTableItem(
          item,
          id
      );

  if (
      dxl.getLastLibErrCode()
      != DXL_LIB_OK)
  {
    printBusError(
        "read control table",
        id
    );

    return false;
  }

  value =
      (uint8_t)result;

  return true;
}


// ============================================================
// TORQUE OFF ALL FIVE
// ============================================================

void torqueOffAll()
{
  // ----------------------------------------------------------
  // Arm
  // ----------------------------------------------------------

  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    dxl.torqueOff(
        motors[i].id
    );
  }


  // ----------------------------------------------------------
  // Gripper
  // ----------------------------------------------------------

  dxl.torqueOff(
      GRIPPER_ID
  );


  Serial.println(
      "ALL FIVE MOTORS TORQUE OFF"
  );
}


// ============================================================
// TORQUE ON ALL FIVE
// ============================================================

bool torqueOnAll()
{
  // ----------------------------------------------------------
  // Arm
  // ----------------------------------------------------------

  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    if (!dxl.torqueOn(
            motors[i].id))
    {
      printBusError(
          "torque ON",
          motors[i].id
      );

      torqueOffAll();

      return false;
    }
  }


  // ----------------------------------------------------------
  // Gripper
  // ----------------------------------------------------------

  if (!dxl.torqueOn(
          GRIPPER_ID))
  {
    printBusError(
        "torque ON",
        GRIPPER_ID
    );

    torqueOffAll();

    return false;
  }


  Serial.println(
      "ALL FIVE MOTORS TORQUE ON"
  );

  return true;
}


// ============================================================
// BACKWARD-COMPATIBLE NAME
// ============================================================

bool torqueOnArm()
{
  return torqueOnAll();
}


// ============================================================
// HOLD ONE MOTOR
// ============================================================

void holdMotorAtPresentPosition(
    uint8_t id,
    const char *reason)
{
  int32_t currentRaw =
      0;

  motionHoldLatched =
      true;

  emergencyStop =
      true;


  if (readPosition(
          id,
          currentRaw))
  {
    if (!dxl.setGoalPosition(
            id,
            currentRaw,
            UNIT_RAW))
    {
      printBusError(
          "hold current position",
          id
      );
    }
  }


  Serial.print(
      "MOTION HOLD ID "
  );

  Serial.print(
      id
  );

  Serial.print(
      " | "
  );

  Serial.println(
      reason
  );


  Serial.println(
      "Torque was left ON. Inspect the robot, then press B."
  );
}


// ============================================================
// MOTOR FAULT CHECK
// ============================================================

bool checkMotorFault(
    uint8_t id)
{
  uint8_t hardwareError =
      0;

  uint8_t movingStatus =
      0;


  if (!readByte(
          ControlTableItem::HARDWARE_ERROR_STATUS,
          id,
          hardwareError))
  {
    emergencyStop =
        true;

    Serial.println(
        "BUS READ FAILED: inspect bus/power, then press B."
    );

    return false;
  }


  if (hardwareError != 0)
  {
    Serial.print(
        "HARDWARE FAULT ID "
    );

    Serial.print(
        id
    );

    Serial.print(
        " | error=0x"
    );

    Serial.println(
        hardwareError,
        HEX
    );

    torqueOffAll();

    emergencyStop =
        true;

    return false;
  }


  if (!readByte(
          ControlTableItem::MOVING_STATUS,
          id,
          movingStatus))
  {
    emergencyStop =
        true;

    Serial.println(
        "BUS READ FAILED: inspect bus/power, then press B."
    );

    return false;
  }


  // Bit 3 = following error

  if (
      (movingStatus & 0x08U)
      != 0)
  {
    Serial.print(
        "FOLLOWING ERROR ID "
    );

    Serial.println(
        id
    );

    holdMotorAtPresentPosition(
        id,
        "following error"
    );

    return false;
  }


  return true;
}


// ============================================================
// HOME OFFSET -> CANONICAL RAW
// ============================================================

int32_t homeOffsetToCanonicalRaw(
    MotorConfig &motor,
    float offsetDegrees)
{
  float extendedDegrees =
      motor.homeAngle +
      offsetDegrees;

  return (int32_t)lroundf(
      extendedDegrees *
      COUNTS_PER_DEGREE
  );
}


// ============================================================
// GET CURRENT HOME PATH OFFSET
// ============================================================

bool getCurrentHomePathOffset(
    MotorConfig &motor,
    int32_t currentRaw,
    float &currentOffset)
{
  float lowerOffset =
      fminf(
          motor.limitAOffset,
          motor.limitBOffset
      );

  float upperOffset =
      fmaxf(
          motor.limitAOffset,
          motor.limitBOffset
      );


  int32_t homeRaw =
      homeOffsetToCanonicalRaw(
          motor,
          0.0f
      );


  int32_t lowerRaw =
      homeOffsetToCanonicalRaw(
          motor,
          lowerOffset
      );


  int32_t upperRaw =
      homeOffsetToCanonicalRaw(
          motor,
          upperOffset
      );


  // ----------------------------------------------------------
  // Search nearby encoder turns.
  // ----------------------------------------------------------

  for (
      int k = -3;
      k <= 3;
      k++)
  {
    int32_t candidateRaw =
        currentRaw +
        k * COUNTS_PER_REV;


    if (
        candidateRaw >=
            lowerRaw -
            HOME_PATH_ENTRY_MARGIN_COUNTS
        &&
        candidateRaw <=
            upperRaw +
            HOME_PATH_ENTRY_MARGIN_COUNTS)
    {
      if (candidateRaw < lowerRaw)
        candidateRaw =
            lowerRaw;

      if (candidateRaw > upperRaw)
        candidateRaw =
            upperRaw;


      currentOffset =
          (candidateRaw - homeRaw) /
          COUNTS_PER_DEGREE;


      return true;
    }
  }


  // ----------------------------------------------------------
  // Outside calibrated range.
  // ----------------------------------------------------------

  int32_t bestRaw =
      currentRaw;

  int32_t bestDistance =
      INT32_MAX;


  for (
      int k = -3;
      k <= 3;
      k++)
  {
    int32_t candidateRaw =
        currentRaw +
        k * COUNTS_PER_REV;


    int32_t distance;


    if (candidateRaw < lowerRaw)
    {
      distance =
          lowerRaw -
          candidateRaw;
    }

    else if (candidateRaw > upperRaw)
    {
      distance =
          candidateRaw -
          upperRaw;
    }

    else
    {
      distance = 0;
    }


    if (
        distance <
        bestDistance)
    {
      bestDistance =
          distance;

      bestRaw =
          candidateRaw;
    }
  }


  currentOffset =
      (bestRaw - homeRaw) /
      COUNTS_PER_DEGREE;


  Serial.print(
      "WARNING: ID "
  );

  Serial.print(
      motor.id
  );

  Serial.println(
      " current RAW is outside calibrated HOME path."
  );


  Serial.print(
      "Nearest HOME-path coordinate = "
  );

  Serial.print(
      currentOffset,
      2
  );

  Serial.println(
      " deg"
  );


  return false;
}


// ============================================================
// GET HOME-PATH TARGET
// ============================================================

bool getHomePathTarget(
    MotorConfig &motor,
    float targetOffset,
    int32_t currentRaw,
    int32_t &targetRaw)
{
  float currentOffset =
      0.0f;


  if (!getCurrentHomePathOffset(
          motor,
          currentRaw,
          currentOffset))
  {
    return false;
  }


  int32_t canonicalTargetRaw =
      homeOffsetToCanonicalRaw(
          motor,
          targetOffset
      );


  int32_t homeRaw =
      homeOffsetToCanonicalRaw(
          motor,
          0.0f
      );


  int32_t currentCanonicalRaw =
      homeRaw +
      (int32_t)lroundf(
          currentOffset *
          COUNTS_PER_DEGREE
      );


  int32_t homePathTurnShift =
      currentRaw -
      currentCanonicalRaw;


  targetRaw =
      canonicalTargetRaw +
      homePathTurnShift;


  return true;
}


// ============================================================
// GET DISPLAY OFFSET
// ============================================================

float getDisplayOffset(
    MotorConfig &motor,
    int32_t raw)
{
  float currentAngle =
      rawToEncoderDegrees(
          raw
      );


  float difference =
      currentAngle -
      motor.homeAngle;


  while (
      difference >
      180.0f)
  {
    difference -=
        360.0f;
  }


  while (
      difference <
      -180.0f)
  {
    difference +=
        360.0f;
  }


  return difference;
}


// ============================================================
// SHOW ARM MOTOR
// ============================================================

void showMotor(
    MotorConfig &motor)
{
  int32_t raw =
      0;


  if (!readPosition(
          motor.id,
          raw))
    return;


  float angle =
      rawToEncoderDegrees(
          raw
      );


  float offset =
      0.0f;


  bool onHomePath =
      getCurrentHomePathOffset(
          motor,
          raw,
          offset
      );


  Serial.print(
      "ID "
  );

  Serial.print(
      motor.id
  );

  Serial.print(
      " | RAW="
  );

  Serial.print(
      raw
  );

  Serial.print(
      " | ENCODER ANGLE="
  );

  Serial.print(
      angle,
      2
  );

  Serial.print(
      " deg | HOME="
  );

  Serial.print(
      motor.homeAngle,
      2
  );

  Serial.print(
      " deg | HOME-PATH OFFSET="
  );


  if (onHomePath)
  {
    Serial.print(
        offset,
        2
    );

    Serial.println(
        " deg"
    );
  }

  else
  {
    Serial.println(
        "OUTSIDE CALIBRATED RANGE"
    );
  }
}


// ============================================================
// SHOW GRIPPER
// ============================================================

void showGripper()
{
  int32_t raw =
      0;


  if (!readPosition(
          GRIPPER_ID,
          raw))
  {
    return;
  }


  float angle =
      rawToEncoderDegrees(
          raw
      );


  Serial.print(
      "ID 15 | GRIPPER | RAW="
  );

  Serial.print(
      raw
  );

  Serial.print(
      " | ENCODER ANGLE="
  );

  Serial.print(
      angle,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "       OPEN = "
  );

  Serial.print(
      GRIPPER_OPEN_ANGLE,
      2
  );

  Serial.print(
      " deg | CLOSED = "
  );

  Serial.print(
      GRIPPER_CLOSED_ANGLE,
      2
  );

  Serial.println(
      " deg"
  );
}


// ============================================================
// SHOW ALL FIVE POSITIONS
// ============================================================

void showPositions()
{
  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "CURRENT MOTOR POSITIONS"
  );

  Serial.println(
      "======================================"
  );


  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    showMotor(
        motors[i]
    );
  }


  showGripper();


  Serial.println(
      "======================================"
  );
}


// ============================================================
// WAIT FOR TARGET
// ============================================================

bool waitForTarget(
    uint8_t id,
    int32_t targetRaw)
{
  uint32_t startTime =
      millis();


  while (
      millis() -
      startTime <
      MOVE_TIMEOUT_MS)
  {
    uint8_t hardwareError =
        0;


    if (!readByte(
            ControlTableItem::HARDWARE_ERROR_STATUS,
            id,
            hardwareError))
    {
      emergencyStop =
          true;

      Serial.println(
          "BUS READ FAILED: inspect bus/power, then press B."
      );

      return false;
    }


    if (hardwareError != 0)
    {
      Serial.print(
          "HARDWARE FAULT ID "
      );

      Serial.print(
          id
      );

      Serial.print(
          " | error=0x"
      );

      Serial.println(
          hardwareError,
          HEX
      );

      torqueOffAll();

      emergencyStop =
          true;

      return false;
    }


    int32_t currentRaw =
        0;


    if (!readPosition(
            id,
            currentRaw))
    {
      emergencyStop =
          true;

      Serial.println(
          "POSITION READ FAILED."
      );

      return false;
    }


    int32_t difference =
        currentRaw -
        targetRaw;


    if (difference < 0)
      difference =
          -difference;


    if (
        difference <=
        POSITION_TOLERANCE_COUNTS)
    {
      return true;
    }


    delay(
        STATUS_POLL_MS
    );
  }


  Serial.print(
      "TIMEOUT ID "
  );

  Serial.println(
      id
  );


  holdMotorAtPresentPosition(
      id,
      "movement timeout"
  );


  return false;
}


// ============================================================
// MOVE MOTOR USING HOME OFFSET
// ============================================================

bool moveMotorToOffset(
    uint8_t id,
    float targetOffset,
    int32_t velocity,
    int32_t acceleration)
{
  if (!controllerReady)
  {
    Serial.println(
        "BLOCKED: controller not ready."
    );

    return false;
  }


  if (emergencyStop)
  {
    Serial.println(
        "BLOCKED: emergency stop active."
    );

    Serial.println(
        "Press B to re-enable."
    );

    return false;
  }


  MotorConfig *motor =
      findMotor(id);


  if (motor == NULL)
  {
    Serial.println(
        "ERROR: unknown motor ID."
    );

    return false;
  }


  int32_t currentRaw =
      0;


  if (!readPosition(
          id,
          currentRaw))
    return false;


  uint8_t hardwareError =
      0;


  if (!readByte(
          ControlTableItem::HARDWARE_ERROR_STATUS,
          id,
          hardwareError))
  {
    emergencyStop =
        true;

    return false;
  }


  if (hardwareError != 0)
  {
    Serial.print(
        "HARDWARE FAULT ID "
    );

    Serial.print(
        id
    );

    Serial.print(
        " | error=0x"
    );

    Serial.println(
        hardwareError,
        HEX
    );

    torqueOffAll();

    emergencyStop =
        true;

    return false;
  }


  int32_t targetRaw =
      0;


  if (!getHomePathTarget(
          *motor,
          targetOffset,
          currentRaw,
          targetRaw))
  {
    Serial.println(
        "BLOCKED: current position is outside calibrated HOME path."
    );

    return false;
  }


  if (
      targetRaw <
          EXTENDED_MIN
      ||
      targetRaw >
          EXTENDED_MAX)
  {
    Serial.println(
        "BLOCKED: target outside extended position range."
    );

    return false;
  }


  Serial.println();

  Serial.println(
      "--------------------------------------"
  );


  Serial.print(
      "MOVE ID "
  );

  Serial.print(
      id
  );

  Serial.print(
      " | CURRENT="
  );

  Serial.print(
      rawToEncoderDegrees(
          currentRaw
      ),
      2
  );

  Serial.print(
      " deg | TARGET="
  );

  Serial.print(
      rawToEncoderDegrees(
          targetRaw
      ),
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "HOME="
  );

  Serial.print(
      motor->homeAngle,
      2
  );

  Serial.print(
      " | TARGET OFFSET="
  );

  Serial.print(
      targetOffset,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "CURRENT RAW="
  );

  Serial.println(
      currentRaw
  );


  Serial.print(
      "TARGET RAW="
  );

  Serial.println(
      targetRaw
  );


  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          id,
          velocity))
  {
    printBusError(
        "write PROFILE_VELOCITY",
        id
    );

    return false;
  }


  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          id,
          acceleration))
  {
    printBusError(
        "write PROFILE_ACCELERATION",
        id
    );

    return false;
  }


  if (!dxl.setGoalPosition(
          id,
          targetRaw,
          UNIT_RAW))
  {
    printBusError(
        "setGoalPosition",
        id
    );

    return false;
  }


  if (!waitForTarget(
          id,
          targetRaw))
  {
    return false;
  }


  Serial.print(
      "ID "
  );

  Serial.print(
      id
  );

  Serial.println(
      " TARGET REACHED"
  );


  showMotor(
      *motor
  );


  return true;
}


// ============================================================
// STEP TARGET CALCULATION
// ============================================================

bool getStepTargetRaw(
    MotorConfig &motor,
    int32_t currentRaw,
    int direction,
    int32_t &targetRaw,
    float &currentOffset,
    float &targetOffset)
{
  if (!getCurrentHomePathOffset(
          motor,
          currentRaw,
          currentOffset))
  {
    return false;
  }


  targetOffset =
      currentOffset +
      direction *
      STEP_DEGREES;


  float lowerOffset =
      fminf(
          motor.limitAOffset,
          motor.limitBOffset
      );


  float upperOffset =
      fmaxf(
          motor.limitAOffset,
          motor.limitBOffset
      );


  if (
      targetOffset <
      lowerOffset)
  {
    Serial.println();

    Serial.print(
        "BLOCKED: ID "
    );

    Serial.print(
        motor.id
    );

    Serial.println(
        " step would exceed calibrated HOME-path limit."
    );

    return false;
  }


  if (
      targetOffset >
      upperOffset)
  {
    Serial.println();

    Serial.print(
        "BLOCKED: ID "
    );

    Serial.print(
        motor.id
    );

    Serial.println(
        " step would exceed calibrated HOME-path limit."
    );

    return false;
  }


  int32_t homeRaw =
      homeOffsetToCanonicalRaw(
          motor,
          0.0f
      );


  int32_t canonicalTargetRaw =
      homeOffsetToCanonicalRaw(
          motor,
          targetOffset
      );


  int32_t currentCanonicalRaw =
      homeRaw +
      (int32_t)lroundf(
          currentOffset *
          COUNTS_PER_DEGREE
      );


  int32_t turnShift =
      currentRaw -
      currentCanonicalRaw;


  int32_t revolutionShift =
      (int32_t)lroundf(
          (float)turnShift /
          (float)COUNTS_PER_REV
      );


  targetRaw =
      canonicalTargetRaw +
      revolutionShift *
      COUNTS_PER_REV;


  if (
      targetRaw <
          EXTENDED_MIN
      ||
      targetRaw >
          EXTENDED_MAX)
  {
    Serial.println(
        "BLOCKED: step target outside extended position range."
    );

    return false;
  }


  return true;
}


// ============================================================
// MOVE ONE DEGREE STEP
// ============================================================

void moveStep(
    uint8_t id,
    int direction)
{
  if (!controllerReady)
  {
    Serial.println(
        "BLOCKED: controller not ready."
    );

    return;
  }


  if (emergencyStop)
  {
    Serial.println(
        "BLOCKED: emergency stop active. Press B."
    );

    return;
  }


  MotorConfig *motor =
      findMotor(id);


  if (motor == NULL)
    return;


  int32_t currentRaw =
      0;


  if (!readPosition(
          id,
          currentRaw))
    return;


  uint8_t hardwareError =
      0;


  if (!readByte(
          ControlTableItem::HARDWARE_ERROR_STATUS,
          id,
          hardwareError))
  {
    emergencyStop =
        true;

    return;
  }


  if (hardwareError != 0)
  {
    Serial.print(
        "HARDWARE FAULT ID "
    );

    Serial.print(
        id
    );

    Serial.print(
        " | error=0x"
    );

    Serial.println(
        hardwareError,
        HEX
    );

    torqueOffAll();

    emergencyStop =
        true;

    return;
  }


  float currentOffset =
      0.0f;

  float targetOffset =
      0.0f;

  int32_t targetRaw =
      0;


  if (!getStepTargetRaw(
          *motor,
          currentRaw,
          direction,
          targetRaw,
          currentOffset,
          targetOffset))
  {
    return;
  }


  Serial.println();

  Serial.println(
      "--------------------------------------"
  );


  Serial.print(
      "STEP ID "
  );

  Serial.print(
      id
  );

  Serial.print(
      " | CURRENT OFFSET="
  );

  Serial.print(
      currentOffset,
      2
  );

  Serial.print(
      " -> TARGET OFFSET="
  );

  Serial.print(
      targetOffset,
      2
  );

  Serial.println(
      " deg"
  );


  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          id,
          MOVE_PROFILE_VELOCITY))
  {
    printBusError(
        "write PROFILE_VELOCITY",
        id
    );

    return;
  }


  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          id,
          MOVE_PROFILE_ACCELERATION))
  {
    printBusError(
        "write PROFILE_ACCELERATION",
        id
    );

    return;
  }


  if (!dxl.setGoalPosition(
          id,
          targetRaw,
          UNIT_RAW))
  {
    printBusError(
        "setGoalPosition",
        id
    );

    return;
  }


  if (!waitForTarget(
          id,
          targetRaw))
  {
    return;
  }


  Serial.print(
      "ID "
  );

  Serial.print(
      id
  );

  Serial.println(
      " STEP REACHED"
  );


  showMotor(
      *motor
  );
}


// ============================================================
// MOVE MOTOR TO ABSOLUTE ENCODER ANGLE
// ============================================================

bool moveMotorToAngle(
    uint8_t id,
    float targetDegrees,
    int32_t velocity,
    int32_t acceleration)
{
  MotorConfig *motor =
      findMotor(id);


  if (motor == NULL)
    return false;


  float targetOffset =
      0.0f;


  if (id == 11)
  {
    if (
        fabs(
            targetDegrees -
            ID11_LIMIT_A) < 0.5f)
    {
      targetOffset =
          -90.0f;
    }

    else if (
        fabs(
            targetDegrees -
            ID11_LIMIT_B) < 0.5f)
    {
      targetOffset =
          +90.0f;
    }

    else
    {
      targetOffset =
          0.0f;
    }
  }


  else if (id == 12)
  {
    if (
        fabs(
            targetDegrees -
            ID12_LIMIT_MIN) < 0.5f)
    {
      targetOffset =
          +82.0f;
    }

    else if (
        fabs(
            targetDegrees -
            ID12_LIMIT_MAX) < 0.5f)
    {
      targetOffset =
          -121.0f;
    }

    else
    {
      targetOffset =
          0.0f;
    }
  }


  else if (id == 13)
  {
    if (
        fabs(
            targetDegrees -
            ID13_LIMIT_MIN) < 0.5f)
    {
      targetOffset =
          +89.0f;
    }

    else if (
        fabs(
            targetDegrees -
            ID13_LIMIT_MAX) < 0.5f)
    {
      targetOffset =
          -121.0f;
    }

    else
    {
      targetOffset =
          0.0f;
    }
  }


  else if (id == 14)
  {
    if (
        fabs(
            targetDegrees -
            ID14_LIMIT_MIN) < 0.5f)
    {
      targetOffset =
          +35.0f;
    }

    else if (
        fabs(
            targetDegrees -
            ID14_LIMIT_MAX) < 0.5f)
    {
      targetOffset =
          -192.0f;
    }

    else
    {
      targetOffset =
          0.0f;
    }
  }


  return moveMotorToOffset(
      id,
      targetOffset,
      velocity,
      acceleration
  );
}


// ============================================================
// GO TO LIMIT
// ============================================================

void goToLimit(
    uint8_t id,
    float angle)
{
  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "CALIBRATION LIMIT TEST"
  );

  Serial.print(
      "ID "
  );

  Serial.print(
      id
  );

  Serial.print(
      " -> ENCODER "
  );

  Serial.print(
      angle,
      2
  );

  Serial.println(
      " deg"
  );

  Serial.println(
      "======================================"
  );


  moveMotorToAngle(
      id,
      angle,
      MOVE_PROFILE_VELOCITY,
      MOVE_PROFILE_ACCELERATION
  );
}


// ============================================================
// GO HOME ONE MOTOR
// ============================================================

bool goHomeMotor(
    uint8_t id)
{
  MotorConfig *motor =
      findMotor(id);


  if (motor == NULL)
    return false;


  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.print(
      "HOME ID "
  );

  Serial.print(
      id
  );

  Serial.print(
      " -> "
  );

  Serial.print(
      motor->homeAngle,
      2
  );

  Serial.println(
      " deg"
  );

  Serial.println(
      "======================================"
  );


  return moveMotorToOffset(
      id,
      0.0f,
      HOME_PROFILE_VELOCITY,
      HOME_PROFILE_ACCELERATION
  );
}


// ============================================================
// HOME ALL
// ============================================================
//
// Gripper is intentionally NOT moved to HOME.
// It has its own OPEN/CLOSED range and is controlled
// separately.
//
// ============================================================

void goHomeAll()
{
  Serial.println();

  Serial.println(
      "######################################"
  );

  Serial.println(
      "STARTING HOME SEQUENCE"
  );

  Serial.println(
      "######################################"
  );


  if (!goHomeMotor(12))
  {
    Serial.println(
        "HOME STOPPED AT ID12"
    );

    return;
  }


  if (!goHomeMotor(13))
  {
    Serial.println(
        "HOME STOPPED AT ID13"
    );

    return;
  }


  if (!goHomeMotor(14))
  {
    Serial.println(
        "HOME STOPPED AT ID14"
    );

    return;
  }


  if (!goHomeMotor(11))
  {
    Serial.println(
        "HOME STOPPED AT ID11"
    );

    return;
  }


  Serial.println();

  Serial.println(
      "######################################"
  );

  Serial.println(
      "HOME COMPLETE"
  );

  Serial.println(
      "######################################"
  );


  showPositions();
}


// ============================================================
// CLEAR TRAJECTORY
// ============================================================

void clearTrajectory()
{
  trajectorySampleCount =
      0;

  trajectoryRecorded =
      false;

  trajectoryValid =
      false;


  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "TRAJECTORY CLEARED"
  );

  Serial.println(
      "======================================"
  );
}


// ============================================================
// READ ALL FIVE POSITIONS
// ============================================================

bool readAllPositions(
    int32_t &id11,
    int32_t &id12,
    int32_t &id13,
    int32_t &id14,
    int32_t &id15)
{
  if (!readPosition(
          11,
          id11))
    return false;


  if (!readPosition(
          12,
          id12))
    return false;


  if (!readPosition(
          13,
          id13))
    return false;


  if (!readPosition(
          14,
          id14))
    return false;


  if (!readPosition(
          GRIPPER_ID,
          id15))
    return false;


  return true;
}


// ============================================================
// READ ALL ARM POSITIONS
// ============================================================

bool readAllArmPositions(
    int32_t &id11,
    int32_t &id12,
    int32_t &id13,
    int32_t &id14)
{
  if (!readPosition(
          11,
          id11))
    return false;


  if (!readPosition(
          12,
          id12))
    return false;


  if (!readPosition(
          13,
          id13))
    return false;


  if (!readPosition(
          14,
          id14))
    return false;


  return true;
}


// ============================================================
// VALIDATE ARM TRAJECTORY SAMPLE
// ============================================================

bool validateTrajectorySample(
    int32_t id11Raw,
    int32_t id12Raw,
    int32_t id13Raw,
    int32_t id14Raw)
{
  float offset =
      0.0f;


  if (!getCurrentHomePathOffset(
          motors[0],
          id11Raw,
          offset))
    return false;


  if (!getCurrentHomePathOffset(
          motors[1],
          id12Raw,
          offset))
    return false;


  if (!getCurrentHomePathOffset(
          motors[2],
          id13Raw,
          offset))
    return false;


  if (!getCurrentHomePathOffset(
          motors[3],
          id14Raw,
          offset))
    return false;


  return true;
}


// ============================================================
// VALIDATE GRIPPER POSITION
// ============================================================
//
// ID15 has its own 0-360 encoder system.
//
// It is NOT part of the arm HOME-path calculation.
//
// ============================================================

bool validateGripperPosition(
    int32_t gripperRaw)
{
  float angle =
      rawToEncoderDegrees(
          gripperRaw
      );


  const float tolerance =
      1.0f;


  if (
      angle <
          GRIPPER_OPEN_ANGLE -
          tolerance
      ||
      angle >
          GRIPPER_CLOSED_ANGLE +
          tolerance)
  {
    Serial.print(
        "WARNING: ID15 gripper position "
    );

    Serial.print(
        angle,
        2
    );

    Serial.println(
        " deg is outside calibrated gripper range."
    );

    return false;
  }


  return true;
}


// ============================================================
// STORE TRAJECTORY SAMPLE
// ============================================================
//
// ALL FIVE motors are captured at the same sampling point.
//
// ============================================================

bool storeTrajectorySample()
{
  if (
      trajectorySampleCount >=
      MAX_TRAJECTORY_SAMPLES)
  {
    return false;
  }


  int32_t id11Raw =
      0;

  int32_t id12Raw =
      0;

  int32_t id13Raw =
      0;

  int32_t id14Raw =
      0;

  int32_t id15Raw =
      0;


  // ----------------------------------------------------------
  // READ ALL FIVE
  // ----------------------------------------------------------

  if (!readAllPositions(
          id11Raw,
          id12Raw,
          id13Raw,
          id14Raw,
          id15Raw))
  {
    return false;
  }


  // ----------------------------------------------------------
  // STORE ID11
  // ----------------------------------------------------------

  trajectory[
      trajectorySampleCount
  ].id11 =
      id11Raw;


  // ----------------------------------------------------------
  // STORE ID12
  // ----------------------------------------------------------

  trajectory[
      trajectorySampleCount
  ].id12 =
      id12Raw;


  // ----------------------------------------------------------
  // STORE ID13
  // ----------------------------------------------------------

  trajectory[
      trajectorySampleCount
  ].id13 =
      id13Raw;


  // ----------------------------------------------------------
  // STORE ID14
  // ----------------------------------------------------------

  trajectory[
      trajectorySampleCount
  ].id14 =
      id14Raw;


  // ----------------------------------------------------------
  // STORE ID15 GRIPPER
  // ----------------------------------------------------------

  trajectory[
      trajectorySampleCount
  ].id15 =
      id15Raw;


  // ----------------------------------------------------------
  // Validate ARM
  // ----------------------------------------------------------

  if (!validateTrajectorySample(
          id11Raw,
          id12Raw,
          id13Raw,
          id14Raw))
  {
    trajectoryValid =
        false;
  }


  // ----------------------------------------------------------
  // Validate GRIPPER
  // ----------------------------------------------------------

  if (!validateGripperPosition(
          id15Raw))
  {
    trajectoryValid =
        false;
  }


  trajectorySampleCount++;


  return true;
}


// ============================================================
// PRINT TEACHING POSITION
// ============================================================

void printTeachingPosition()
{
  int32_t id11Raw =
      0;

  int32_t id12Raw =
      0;

  int32_t id13Raw =
      0;

  int32_t id14Raw =
      0;

  int32_t id15Raw =
      0;


  if (!readAllPositions(
          id11Raw,
          id12Raw,
          id13Raw,
          id14Raw,
          id15Raw))
  {
    return;
  }


  Serial.println();

  Serial.println(
      "FINAL TEACH POSITION"
  );

  Serial.println(
      "--------------------------------------"
  );


  Serial.print(
      "ID11 = "
  );

  Serial.print(
      rawToEncoderDegrees(
          id11Raw
      ),
      2
  );

  Serial.print(
      " deg | RAW="
  );

  Serial.println(
      id11Raw
  );


  Serial.print(
      "ID12 = "
  );

  Serial.print(
      rawToEncoderDegrees(
          id12Raw
      ),
      2
  );

  Serial.print(
      " deg | RAW="
  );

  Serial.println(
      id12Raw
  );


  Serial.print(
      "ID13 = "
  );

  Serial.print(
      rawToEncoderDegrees(
          id13Raw
      ),
      2
  );

  Serial.print(
      " deg | RAW="
  );

  Serial.println(
      id13Raw
  );


  Serial.print(
      "ID14 = "
  );

  Serial.print(
      rawToEncoderDegrees(
          id14Raw
      ),
      2
  );

  Serial.print(
      " deg | RAW="
  );

  Serial.println(
      id14Raw
  );


  Serial.print(
      "ID15 GRIPPER = "
  );

  Serial.print(
      rawToEncoderDegrees(
          id15Raw
      ),
      2
  );

  Serial.print(
      " deg | RAW="
  );

  Serial.println(
      id15Raw
  );
}


// ============================================================
// SAFE GOAL SYNCHRONIZATION
// ============================================================
//
// Read actual physical position.
//
// Then write the exact current position to Goal Position.
//
// This prevents jumps when torque is restored.
//
// ALL FIVE motors are synchronized.
//
// ============================================================

bool synchronizeGoalsToPresentPosition()
{
  int32_t id11Raw =
      0;

  int32_t id12Raw =
      0;

  int32_t id13Raw =
      0;

  int32_t id14Raw =
      0;

  int32_t id15Raw =
      0;


  if (!readAllPositions(
          id11Raw,
          id12Raw,
          id13Raw,
          id14Raw,
          id15Raw))
  {
    Serial.println(
        "FAILED: could not read final positions."
    );

    return false;
  }


  Serial.println();

  Serial.println(
      "SYNCHRONIZING ALL FIVE GOALS..."
  );


  // ----------------------------------------------------------
  // ID11
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          11,
          id11Raw,
          UNIT_RAW))
  {
    printBusError(
        "sync goal ID11",
        11
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID12
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          12,
          id12Raw,
          UNIT_RAW))
  {
    printBusError(
        "sync goal ID12",
        12
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID13
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          13,
          id13Raw,
          UNIT_RAW))
  {
    printBusError(
        "sync goal ID13",
        13
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID14
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          14,
          id14Raw,
          UNIT_RAW))
  {
    printBusError(
        "sync goal ID14",
        14
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID15 GRIPPER
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          GRIPPER_ID,
          id15Raw,
          UNIT_RAW))
  {
    printBusError(
        "sync goal ID15",
        GRIPPER_ID
    );

    return false;
  }


  Serial.println(
      "ALL FIVE GOALS SYNCHRONIZED."
  );


  return true;
}


// ============================================================
// CONFIGURE GRIPPER PROFILE
// ============================================================
//
// This function restores the working gripper settings.
//
// ============================================================

bool configureGripperProfile()
{
  // ----------------------------------------------------------
  // Goal Current
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::GOAL_CURRENT,
          GRIPPER_ID,
          GRIPPER_GOAL_CURRENT))
  {
    printBusError(
        "gripper GOAL_CURRENT",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Velocity
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          GRIPPER_ID,
          GRIPPER_PROFILE_VELOCITY))
  {
    printBusError(
        "gripper PROFILE_VELOCITY",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Acceleration
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          GRIPPER_ID,
          GRIPPER_PROFILE_ACCELERATION))
  {
    printBusError(
        "gripper PROFILE_ACCELERATION",
        GRIPPER_ID
    );

    return false;
  }


  return true;
}


// ============================================================
// TEACHING COUNTDOWN
// ============================================================

void teachingCountdown()
{
  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "PREPARING FOR MANUAL TEACHING"
  );

  Serial.println(
      "======================================"
  );


  Serial.println(
      "Keep your hand on the robot."
  );


  Serial.println(
      "ID11-ID15 will be released."
  );


  Serial.print(
      "Torque OFF in "
  );

  Serial.print(
      TEACH_START_DELAY_SECONDS
  );

  Serial.println(
      " seconds."
  );


  for (
      uint32_t i =
          TEACH_START_DELAY_SECONDS;
      i > 0;
      i--)
  {
    Serial.print(
        i
    );

    Serial.println(
        "..."
    );

    delay(
        1000
    );
  }
}


// ============================================================
// START TEACHING
// ============================================================

void startTeaching()
{
  if (!controllerReady)
  {
    Serial.println(
        "BLOCKED: controller not ready."
    );

    return;
  }


  if (
      teachModeActive ||
      replayModeActive)
  {
    Serial.println(
        "BLOCKED: another motion mode is active."
    );

    return;
  }


  if (emergencyStop)
  {
    Serial.println(
        "BLOCKED: emergency stop active."
    );

    Serial.println(
        "Press B first."
    );

    return;
  }


  // ----------------------------------------------------------
  // Clear old trajectory
  // ----------------------------------------------------------

  clearTrajectory();


  trajectoryValid =
      true;

  teachModeActive =
      true;


  // ----------------------------------------------------------
  // Start position
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "TEACH MODE START"
  );

  Serial.println(
      "======================================"
  );


  Serial.println(
      "START POSITION:"
  );


  showPositions();


  // ----------------------------------------------------------
  // Countdown
  // ----------------------------------------------------------

  teachingCountdown();


  // ----------------------------------------------------------
  // TORQUE OFF ALL FIVE
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "TORQUE OFF"
  );

  Serial.println(
      "MANUAL TEACHING ACTIVE"
  );

  Serial.println(
      "======================================"
  );


  torqueOffAll();


  Serial.println();

  Serial.println(
      "MOVE THE ARM AND GRIPPER MANUALLY NOW."
  );


  Serial.print(
      "Teaching duration = "
  );

  Serial.print(
      TEACH_DURATION_SECONDS
  );

  Serial.println(
      " seconds."
  );


  Serial.println(
      "Recording ID11, ID12, ID13, ID14 and ID15 GRIPPER..."
  );


  // ----------------------------------------------------------
  // RECORD TRAJECTORY
  // ----------------------------------------------------------

  uint32_t startTime =
      millis();


  uint32_t nextSampleTime =
      startTime;


  uint32_t nextDisplayTime =
      startTime;


  while (
      millis() -
      startTime <
      TEACH_DURATION_SECONDS *
      1000UL)
  {
    uint32_t now =
        millis();


    // --------------------------------------------------------
    // Record sample
    // --------------------------------------------------------

    if (
        now >=
        nextSampleTime)
    {
      if (!storeTrajectorySample())
      {
        Serial.println(
            "WARNING: failed to store trajectory sample."
        );
      }


      nextSampleTime +=
          TEACH_SAMPLE_INTERVAL_MS;
    }


    // --------------------------------------------------------
    // Display remaining time
    // --------------------------------------------------------

    if (
        now >=
        nextDisplayTime)
    {
      uint32_t elapsed =
          now -
          startTime;


      uint32_t remaining =
          TEACH_DURATION_SECONDS -
          (
              elapsed /
              1000UL
          );


      Serial.print(
          "Teaching... "
      );

      Serial.print(
          remaining
      );

      Serial.print(
          " sec remaining | samples="
      );

      Serial.println(
          trajectorySampleCount
      );


      nextDisplayTime =
          now +
          1000UL;
    }


    delay(
        2
    );
  }


  // ----------------------------------------------------------
  // Final sample
  // ----------------------------------------------------------

  storeTrajectorySample();


  // ----------------------------------------------------------
  // Teaching finished
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "TEACHING COMPLETE"
  );

  Serial.println(
      "======================================"
  );


  Serial.print(
      "Samples recorded = "
  );

  Serial.println(
      trajectorySampleCount
  );


  printTeachingPosition();


  // ----------------------------------------------------------
  // Synchronize ALL FIVE goals
  // ----------------------------------------------------------

  if (!synchronizeGoalsToPresentPosition())
  {
    Serial.println(
        "FAILED TO SYNCHRONIZE GOALS."
    );

    Serial.println(
        "Torque will remain OFF."
    );

    teachModeActive =
        false;

    return;
  }


  // ----------------------------------------------------------
  // Configure gripper
  // ----------------------------------------------------------

  if (!configureGripperProfile())
  {
    Serial.println(
        "FAILED TO CONFIGURE GRIPPER PROFILE."
    );

    Serial.println(
        "Torque will remain OFF."
    );

    teachModeActive =
        false;

    return;
  }


  // ----------------------------------------------------------
  // Torque ON ALL FIVE
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
      "ENABLING ALL FIVE MOTOR TORQUE..."
  );


  if (!torqueOnAll())
  {
    Serial.println(
        "TORQUE ON FAILED."
    );

    teachModeActive =
        false;

    return;
  }


  // ----------------------------------------------------------
  // Mark trajectory
  // ----------------------------------------------------------

  trajectoryRecorded =
      trajectorySampleCount > 1;


  if (
      trajectoryRecorded &&
      trajectoryValid)
  {
    Serial.println();

    Serial.println(
        "TRAJECTORY READY FOR REPLAY."
    );

    Serial.println(
        "Y = replay forward"
    );

    Serial.println(
        "U = replay backward"
    );
  }

  else if (trajectoryRecorded)
  {
    Serial.println();

    Serial.println(
        "WARNING: trajectory contains positions"
    );

    Serial.println(
        "outside the calibrated range."
    );

    Serial.println(
        "Replay is BLOCKED."
    );

    trajectoryValid =
        false;
  }


  teachModeActive =
      false;


  Serial.println();

  Serial.println(
      "TEACH MODE FINISHED."
  );
}


// ============================================================
// SET REPLAY PROFILE
// ============================================================

bool configureReplayProfile(
    uint8_t id)
{
  // ----------------------------------------------------------
  // Gripper
  // ----------------------------------------------------------

  if (id == GRIPPER_ID)
  {
    return configureGripperProfile();
  }


  // ----------------------------------------------------------
  // Arm
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          id,
          MOVE_PROFILE_VELOCITY))
  {
    printBusError(
        "replay PROFILE_VELOCITY",
        id
    );

    return false;
  }


  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          id,
          MOVE_PROFILE_ACCELERATION))
  {
    printBusError(
        "replay PROFILE_ACCELERATION",
        id
    );

    return false;
  }


  return true;
}


// ============================================================
// REPLAY ONE SAMPLE
// ============================================================
//
// ID11-ID15 are commanded together.
//
// ============================================================

bool replaySample(
    const TrajectorySample &sample)
{
  // ----------------------------------------------------------
  // Configure profiles
  // ----------------------------------------------------------

  if (!configureReplayProfile(11))
    return false;


  if (!configureReplayProfile(12))
    return false;


  if (!configureReplayProfile(13))
    return false;


  if (!configureReplayProfile(14))
    return false;


  if (!configureReplayProfile(GRIPPER_ID))
    return false;


  // ----------------------------------------------------------
  // ID11
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          11,
          sample.id11,
          UNIT_RAW))
  {
    printBusError(
        "replay ID11",
        11
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID12
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          12,
          sample.id12,
          UNIT_RAW))
  {
    printBusError(
        "replay ID12",
        12
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID13
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          13,
          sample.id13,
          UNIT_RAW))
  {
    printBusError(
        "replay ID13",
        13
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID14
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          14,
          sample.id14,
          UNIT_RAW))
  {
    printBusError(
        "replay ID14",
        14
    );

    return false;
  }


  // ----------------------------------------------------------
  // ID15 GRIPPER
  // ----------------------------------------------------------

  if (!dxl.setGoalPosition(
          GRIPPER_ID,
          sample.id15,
          UNIT_RAW))
  {
    printBusError(
        "replay ID15 GRIPPER",
        GRIPPER_ID
    );

    return false;
  }


  return true;
}


// ============================================================
// CHECK REPLAY HARDWARE
// ============================================================
//
// Check ALL FIVE motors.
//
// ============================================================

bool checkReplayHardware()
{
  // ----------------------------------------------------------
  // Arm motors
  // ----------------------------------------------------------

  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    uint8_t hardwareError =
        0;


    if (!readByte(
            ControlTableItem::HARDWARE_ERROR_STATUS,
            motors[i].id,
            hardwareError))
    {
      emergencyStop =
          true;

      return false;
    }


    if (hardwareError != 0)
    {
      Serial.print(
          "HARDWARE FAULT DURING REPLAY ID "
      );

      Serial.print(
          motors[i].id
      );

      Serial.print(
          " | error=0x"
      );

      Serial.println(
          hardwareError,
          HEX
      );


      torqueOffAll();

      emergencyStop =
          true;

      return false;
    }
  }


  // ----------------------------------------------------------
  // Gripper
  // ----------------------------------------------------------

  uint8_t gripperHardwareError =
      0;


  if (!readByte(
          ControlTableItem::HARDWARE_ERROR_STATUS,
          GRIPPER_ID,
          gripperHardwareError))
  {
    emergencyStop =
        true;

    return false;
  }


  if (gripperHardwareError != 0)
  {
    Serial.print(
        "HARDWARE FAULT DURING REPLAY ID15"
    );

    Serial.print(
        " | error=0x"
    );

    Serial.println(
        gripperHardwareError,
        HEX
    );


    torqueOffAll();

    emergencyStop =
        true;

    return false;
  }


  return true;
}


// ============================================================
// REPLAY TRAJECTORY
// ============================================================

void replayTrajectory(
    bool forward)
{
  if (!controllerReady)
  {
    Serial.println(
        "BLOCKED: controller not ready."
    );

    return;
  }


  if (emergencyStop)
  {
    Serial.println(
        "BLOCKED: emergency stop active."
    );

    Serial.println(
        "Press B first."
    );

    return;
  }


  if (teachModeActive)
  {
    Serial.println(
        "BLOCKED: teach mode active."
    );

    return;
  }


  if (!trajectoryRecorded)
  {
    Serial.println();

    Serial.println(
        "NO TRAJECTORY RECORDED."
    );

    Serial.println(
        "Press T to teach a trajectory first."
    );

    return;
  }


  if (!trajectoryValid)
  {
    Serial.println();

    Serial.println(
        "BLOCKED: trajectory contains invalid positions."
    );

    return;
  }


  if (
      trajectorySampleCount <
      2)
  {
    Serial.println(
        "BLOCKED: not enough trajectory samples."
    );

    return;
  }


  replayModeActive =
      true;


  Serial.println();

  Serial.println(
      "======================================"
  );


  if (forward)
  {
    Serial.println(
        "REPLAY FORWARD"
    );
  }

  else
  {
    Serial.println(
        "REPLAY BACKWARD"
    );
  }


  Serial.println(
      "======================================"
  );


  Serial.print(
      "Samples = "
  );

  Serial.println(
      trajectorySampleCount
  );


  Serial.print(
      "Replay interval = "
  );

  Serial.print(
      REPLAY_INTERVAL_MS
  );

  Serial.println(
      " ms"
  );


  Serial.println(
      "Replaying ID11 + ID12 + ID13 + ID14 + ID15."
  );


  Serial.println(
      "GRIPPER IS INCLUDED IN REPLAY."
  );


  // ----------------------------------------------------------
  // Configure gripper
  // ----------------------------------------------------------

  if (!configureGripperProfile())
  {
    Serial.println(
        "Could not configure gripper."
    );

    replayModeActive =
        false;

    return;
  }


  // ----------------------------------------------------------
  // Torque ON ALL FIVE
  // ----------------------------------------------------------

  if (!torqueOnAll())
  {
    Serial.println(
        "Could not enable all five motors."
    );

    replayModeActive =
        false;

    return;
  }


  // ----------------------------------------------------------
  // REPLAY FORWARD
  // ----------------------------------------------------------

  if (forward)
  {
    for (
        uint16_t i = 0;
        i < trajectorySampleCount;
        i++)
    {
      if (emergencyStop)
        break;


      if (!checkReplayHardware())
        break;


      if (!replaySample(
              trajectory[i]))
      {
        break;
      }


      delay(
          REPLAY_INTERVAL_MS
      );
    }
  }


  // ----------------------------------------------------------
  // REPLAY BACKWARD
  // ----------------------------------------------------------

  else
  {
    for (
        int32_t i =
            trajectorySampleCount - 1;
        i >= 0;
        i--)
    {
      if (emergencyStop)
        break;


      if (!checkReplayHardware())
        break;


      if (!replaySample(
              trajectory[i]))
      {
        break;
      }


      delay(
          REPLAY_INTERVAL_MS
      );
    }
  }


  replayModeActive =
      false;


  Serial.println();

  Serial.println(
      "======================================"
  );

  Serial.println(
      "REPLAY COMPLETE"
  );

  Serial.println(
      "======================================"
  );


  showPositions();
}


// ============================================================
// INITIALIZE ARM MOTOR
// ============================================================

bool initializeMotor(
    MotorConfig &motor)
{
  Serial.print(
      "Initializing ID "
  );

  Serial.println(
      motor.id
  );


  // ----------------------------------------------------------
  // Ping
  // ----------------------------------------------------------

  if (!dxl.ping(
          motor.id))
  {
    printBusError(
        "PING",
        motor.id
    );

    return false;
  }


  // ----------------------------------------------------------
  // Torque OFF
  // ----------------------------------------------------------

  if (!dxl.torqueOff(
          motor.id))
  {
    printBusError(
        "torque OFF",
        motor.id
    );

    return false;
  }


  // ----------------------------------------------------------
  // Extended Position Mode
  // ----------------------------------------------------------

  if (!dxl.setOperatingMode(
          motor.id,
          OP_EXTENDED_POSITION))
  {
    printBusError(
        "set operating mode",
        motor.id
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Velocity
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          motor.id,
          MOVE_PROFILE_VELOCITY))
  {
    printBusError(
        "profile velocity",
        motor.id
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Acceleration
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          motor.id,
          MOVE_PROFILE_ACCELERATION))
  {
    printBusError(
        "profile acceleration",
        motor.id
    );

    return false;
  }


  Serial.print(
      "ID "
  );

  Serial.print(
      motor.id
  );

  Serial.println(
      " READY"
  );


  return true;
}


// ============================================================
// INITIALIZE GRIPPER ID15
// ============================================================
//
// IMPORTANT:
//
// ID15 uses:
//
//   OP_CURRENT_BASED_POSITION
//
// unlike IDs 11-14.
//
// ============================================================

bool initializeGripper()
{
  Serial.println(
      "Initializing GRIPPER ID15"
  );


  // ----------------------------------------------------------
  // Ping
  // ----------------------------------------------------------

  if (!dxl.ping(
          GRIPPER_ID))
  {
    printBusError(
        "PING",
        GRIPPER_ID
    );

    return false;
  }


  Serial.println(
      "ID15 FOUND."
  );


  // ----------------------------------------------------------
  // Torque OFF
  // ----------------------------------------------------------

  if (!dxl.torqueOff(
          GRIPPER_ID))
  {
    printBusError(
        "gripper torque OFF",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Current-Based Position Mode
  // ----------------------------------------------------------

  if (!dxl.setOperatingMode(
          GRIPPER_ID,
          OP_CURRENT_BASED_POSITION))
  {
    printBusError(
        "gripper operating mode",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Goal Current
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::GOAL_CURRENT,
          GRIPPER_ID,
          GRIPPER_GOAL_CURRENT))
  {
    printBusError(
        "gripper GOAL_CURRENT",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Velocity
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_VELOCITY,
          GRIPPER_ID,
          GRIPPER_PROFILE_VELOCITY))
  {
    printBusError(
        "gripper PROFILE_VELOCITY",
        GRIPPER_ID
    );

    return false;
  }


  // ----------------------------------------------------------
  // Profile Acceleration
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::PROFILE_ACCELERATION,
          GRIPPER_ID,
          GRIPPER_PROFILE_ACCELERATION))
  {
    printBusError(
        "gripper PROFILE_ACCELERATION",
        GRIPPER_ID
    );

    return false;
  }


  Serial.println(
      "ID15 GRIPPER READY"
  );


  Serial.println(
      "Mode = CURRENT BASED POSITION"
  );


  Serial.print(
      "Goal Current = "
  );

  Serial.println(
      GRIPPER_GOAL_CURRENT
  );


  Serial.print(
      "OPEN = "
  );

  Serial.print(
      GRIPPER_OPEN_ANGLE,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "CLOSED = "
  );

  Serial.print(
      GRIPPER_CLOSED_ANGLE,
      2
  );

  Serial.println(
      " deg"
  );


  return true;
}


// ============================================================
// HELP
// ============================================================

void printHelp()
{
  Serial.println();

  Serial.println(
      "=============================================="
  );

  Serial.println(
      "OPENMANIPULATOR-X V9 COMMANDS"
  );

  Serial.println(
      "=============================================="
  );


  Serial.println(
      "JOINT STEPS:"
  );

  Serial.println(
      "Q / A = ID11 +1 / -1"
  );

  Serial.println(
      "W / Z = ID12 +1 / -1"
  );

  Serial.println(
      "E / D = ID13 +1 / -1"
  );

  Serial.println(
      "R / F = ID14 +1 / -1"
  );


  Serial.println();

  Serial.println(
      "ID11 LIMITS:"
  );

  Serial.println(
      "1 = ID11 -> 261 deg"
  );

  Serial.println(
      "2 = ID11 -> 81 deg THROUGH HOME"
  );


  Serial.println();

  Serial.println(
      "ID12 LIMITS:"
  );

  Serial.println(
      "3 = ID12 -> 83 deg"
  );

  Serial.println(
      "4 = ID12 -> 240 deg THROUGH HOME"
  );


  Serial.println();

  Serial.println(
      "ID13 LIMITS:"
  );

  Serial.println(
      "5 = ID13 -> 90 deg"
  );

  Serial.println(
      "6 = ID13 -> 240 deg THROUGH HOME"
  );


  Serial.println();

  Serial.println(
      "ID14 LIMITS:"
  );

  Serial.println(
      "7 = ID14 -> 125 deg"
  );

  Serial.println(
      "8 = ID14 -> 258 deg THROUGH HOME"
  );


  Serial.println();

  Serial.println(
      "GRIPPER ID15:"
  );

  Serial.print(
      "OPEN = "
  );

  Serial.print(
      GRIPPER_OPEN_ANGLE,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "CLOSED = "
  );

  Serial.print(
      GRIPPER_CLOSED_ANGLE,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.println(
      "ID15 IS INCLUDED IN TEACH / REPLAY"
  );


  Serial.println();

  Serial.println(
      "TEACH / REPLAY:"
  );

  Serial.println(
      "T = START MANUAL TEACHING"
  );

  Serial.println(
      "Y = REPLAY FORWARD"
  );

  Serial.println(
      "U = REPLAY BACKWARD"
  );

  Serial.println(
      "C = CLEAR TRAJECTORY"
  );


  Serial.println();

  Serial.println(
      "GENERAL:"
  );

  Serial.println(
      "H = HOME ARM"
  );

  Serial.println(
      "P = SHOW POSITIONS"
  );

  Serial.println(
      "X = TORQUE OFF"
  );

  Serial.println(
      "B = TORQUE ON"
  );


  Serial.println();

  Serial.println(
      "TEACH SETTINGS:"
  );


  Serial.print(
      "Preparation delay = "
  );

  Serial.print(
      TEACH_START_DELAY_SECONDS
  );

  Serial.println(
      " sec"
  );


  Serial.print(
      "Teaching duration = "
  );

  Serial.print(
      TEACH_DURATION_SECONDS
  );

  Serial.println(
      " sec"
  );


  Serial.print(
      "Sample interval = "
  );

  Serial.print(
      TEACH_SAMPLE_INTERVAL_MS
  );

  Serial.println(
      " ms"
  );


  Serial.print(
      "Replay interval = "
  );

  Serial.print(
      REPLAY_INTERVAL_MS
  );

  Serial.println(
      " ms"
  );


  Serial.println(
      "=============================================="
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
      115200
  );


  delay(
      1000
  );


  Serial.println();

  Serial.println(
      "=============================================="
  );

  Serial.println(
      "OPENMANIPULATOR-X"
  );

  Serial.println(
      "CALIBRATION + TEACH / REPLAY CONTROLLER V9"
  );

  Serial.println(
      "XM430-W350-T"
  );

  Serial.println(
      "=============================================="
  );


  // ----------------------------------------------------------
  // DYNAMIXEL BUS
  // ----------------------------------------------------------

  dxl.begin(
      BAUDRATE
  );

  dxl.setPortProtocolVersion(
      PROTOCOL_VERSION
  );


  // ----------------------------------------------------------
  // Initialize arm motors
  // ----------------------------------------------------------

  for (
      size_t i = 0;
      i < MOTOR_COUNT;
      i++)
  {
    if (!initializeMotor(
            motors[i]))
    {
      Serial.println();

      Serial.println(
          "ARM INITIALIZATION FAILED"
      );

      torqueOffAll();

      return;
    }
  }


  // ----------------------------------------------------------
  // Initialize gripper
  // ----------------------------------------------------------

  if (!initializeGripper())
  {
    Serial.println();

    Serial.println(
        "GRIPPER INITIALIZATION FAILED"
    );

    torqueOffAll();

    return;
  }


  // ----------------------------------------------------------
  // Synchronize initial goals
  // ----------------------------------------------------------
  //
  // This is especially important for ID15.
  //
  // ----------------------------------------------------------

  Serial.println();

  Serial.println(
      "SYNCHRONIZING INITIAL GOALS..."
  );


  if (!synchronizeGoalsToPresentPosition())
  {
    Serial.println(
        "INITIAL GOAL SYNCHRONIZATION FAILED"
    );

    torqueOffAll();

    return;
  }


  // ----------------------------------------------------------
  // Enable all five motors
  // ----------------------------------------------------------

  if (!torqueOnAll())
  {
    Serial.println(
        "MOTOR TORQUE FAILED"
    );

    return;
  }


  controllerReady =
      true;


  // ----------------------------------------------------------
  // Show current position
  // ----------------------------------------------------------

  showPositions();


  // ----------------------------------------------------------
  // Commands
  // ----------------------------------------------------------

  printHelp();


  Serial.println();

  Serial.println(
      "READY."
  );

  Serial.println(
      "No automatic movement."
  );

  Serial.println(
      "Press H to move the ARM to HOME."
  );

  Serial.println(
      "Press T to start manual teaching."
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  if (!Serial.available())
    return;


  char key =
      Serial.read();


  if (
      key == '\r' ||
      key == '\n')
    return;


  // ==========================================================
  // ID11
  // ==========================================================

  if (
      key == 'q' ||
      key == 'Q')
  {
    moveStep(
        11,
        +1
    );
  }


  else if (
      key == 'a' ||
      key == 'A')
  {
    moveStep(
        11,
        -1
    );
  }


  // ==========================================================
  // ID12
  // ==========================================================

  else if (
      key == 'w' ||
      key == 'W')
  {
    moveStep(
        12,
        +1
    );
  }


  else if (
      key == 'z' ||
      key == 'Z')
  {
    moveStep(
        12,
        -1
    );
  }


  // ==========================================================
  // ID13
  // ==========================================================

  else if (
      key == 'e' ||
      key == 'E')
  {
    moveStep(
        13,
        +1
    );
  }


  else if (
      key == 'd' ||
      key == 'D')
  {
    moveStep(
        13,
        -1
    );
  }


  // ==========================================================
  // ID14
  // ==========================================================

  else if (
      key == 'r' ||
      key == 'R')
  {
    moveStep(
        14,
        +1
    );
  }


  else if (
      key == 'f' ||
      key == 'F')
  {
    moveStep(
        14,
        -1
    );
  }


  // ==========================================================
  // ID11 LIMITS
  // ==========================================================

  else if (key == '1')
  {
    goToLimit(
        11,
        ID11_LIMIT_A
    );
  }


  else if (key == '2')
  {
    goToLimit(
        11,
        ID11_LIMIT_B
    );
  }


  // ==========================================================
  // ID12 LIMITS
  // ==========================================================

  else if (key == '3')
  {
    goToLimit(
        12,
        ID12_LIMIT_MIN
    );
  }


  else if (key == '4')
  {
    goToLimit(
        12,
        ID12_LIMIT_MAX
    );
  }


  // ==========================================================
  // ID13 LIMITS
  // ==========================================================

  else if (key == '5')
  {
    goToLimit(
        13,
        ID13_LIMIT_MIN
    );
  }


  else if (key == '6')
  {
    goToLimit(
        13,
        ID13_LIMIT_MAX
    );
  }


  // ==========================================================
  // ID14 LIMITS
  // ==========================================================

  else if (key == '7')
  {
    goToLimit(
        14,
        ID14_LIMIT_MIN
    );
  }


  else if (key == '8')
  {
    goToLimit(
        14,
        ID14_LIMIT_MAX
    );
  }


  // ==========================================================
  // HOME
  // ==========================================================

  else if (
      key == 'h' ||
      key == 'H')
  {
    goHomeAll();
  }


  // ==========================================================
  // POSITIONS
  // ==========================================================

  else if (
      key == 'p' ||
      key == 'P')
  {
    showPositions();
  }


  // ==========================================================
  // TEACH
  // ==========================================================

  else if (
      key == 't' ||
      key == 'T')
  {
    startTeaching();
  }


  // ==========================================================
  // REPLAY FORWARD
  // ==========================================================

  else if (
      key == 'y' ||
      key == 'Y')
  {
    replayTrajectory(
        true
    );
  }


  // ==========================================================
  // REPLAY BACKWARD
  // ==========================================================

  else if (
      key == 'u' ||
      key == 'U')
  {
    replayTrajectory(
        false
    );
  }


  // ==========================================================
  // CLEAR TRAJECTORY
  // ==========================================================

  else if (
      key == 'c' ||
      key == 'C')
  {
    clearTrajectory();
  }


  // ==========================================================
  // EMERGENCY TORQUE OFF
  // ==========================================================

  else if (
      key == 'x' ||
      key == 'X')
  {
    torqueOffAll();

    emergencyStop =
        true;


    Serial.println();

    Serial.println(
        "!!! TORQUE OFF !!!"
    );

    Serial.println(
        "Inspect robot before pressing B."
    );
  }


  // ==========================================================
  // TORQUE ON
  // ==========================================================

  else if (
      key == 'b' ||
      key == 'B')
  {
    // --------------------------------------------------------
    // Synchronize ALL FIVE before restoring torque.
    // --------------------------------------------------------

    if (synchronizeGoalsToPresentPosition())
    {
      if (configureGripperProfile())
      {
        if (torqueOnAll())
        {
          emergencyStop =
              false;

          motionHoldLatched =
              false;


          Serial.println(
              "All five motors enabled; motion hold cleared."
          );
        }
      }
    }
  }


  // ==========================================================
  // UNKNOWN COMMAND
  // ==========================================================

  else
  {
    printHelp();
  }
}