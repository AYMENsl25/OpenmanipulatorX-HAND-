#include <Dynamixel2Arduino.h>
#include <math.h>

// ============================================================
// OPENMANIPULATOR-X
// CALIBRATION CONTROLLER - STABLE HOME-PATH MOVEMENT (V6)
// XM430 TIME-BASED SLOW MOTION VERSION
// ============================================================
//
// IDs:
//   11 = Joint 1
//   12 = Joint 2
//   13 = Joint 3
//   14 = Joint 4
//   15 = Gripper
//
// Communication:
//   OpenCR Serial3
//   Direction pin 84
//   Baudrate 1,000,000
//   Protocol 2.0
//
// Motor:
//   DYNAMIXEL XM430-W350-T
//
// Operating Mode:
//   Extended Position Mode
//
// Profile:
//   TIME-BASED PROFILE
//
// HOME:
//
//   ID11 = 351°
//   ID12 = 1°
//   ID13 = 1°
//   ID14 = 90°
//
// Physical paths:
//
// ID11:
//   261° -> HOME 351° -> 81°
//
// ID12:
//   240° -> HOME 1° -> 83°
//
// ID13:
//   240° -> HOME 1° -> 90°
//
// ID14:
//   258° -> HOME 90° -> 125°
//
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
// XM430 TIME-BASED PROFILE
// ============================================================
//
// IMPORTANT:
//
// Drive Mode bit 2 = 1
//
// Therefore:
//
//   Profile Velocity     = TIME in milliseconds
//   Profile Acceleration = ACCELERATION TIME in milliseconds
//
// Example:
//
//   4000 ms = approximately 4 second profile
//
//   2000 ms = approximately 2 second acceleration
//
// ROBOTIS requires acceleration time to be no more than
// approximately 50% of the Profile Velocity time.
//
// ------------------------------------------------------------
//
// NORMAL STEP MOVEMENT:
//
//   4000 ms total movement
//   1800 ms acceleration
//
// HOME / LIMIT MOVEMENT:
//
//   6000 ms total movement
//   2500 ms acceleration
//
// ============================================================

static const int32_t MOVE_PROFILE_VELOCITY =
    3000;

static const int32_t MOVE_PROFILE_ACCELERATION =
    1400;


static const int32_t HOME_PROFILE_VELOCITY =
    3000;

static const int32_t HOME_PROFILE_ACCELERATION =
    2000;


// ============================================================
// TIME-BASED PROFILE DRIVE MODE
// ============================================================
//
// Drive Mode:
//
// Bit 2 = 1
//
// 0x04 = Time-Based Profile
//
// Bit 0 remains 0 = normal direction
//
// Bit 3 remains 0 = normal torque behavior
//
// ============================================================

static const uint8_t TIME_BASED_DRIVE_MODE =
    0x04;


// ============================================================
// STEP SIZE
// ============================================================
//
// Each Q/A/W/Z/E/D/R/F command moves exactly 1 degree.
//
// The TIME-BASED PROFILE controls how slowly that 1 degree
// movement is executed.
//
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
// CALIBRATED MECHANICAL LIMITS
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
// MOTOR TABLE
// ============================================================

MotorConfig motors[] =
{
  {11, 351.0f, 261.0f,  81.0f,  -90.0f,  +90.0f},
  {12,   1.0f,  83.0f, 240.0f,  +82.0f, -121.0f},
  {13,   1.0f,  90.0f, 240.0f,  +89.0f, -121.0f},
  {14,  90.0f, 125.0f, 258.0f,  +35.0f, -192.0f}
};


static const size_t MOTOR_COUNT =
    sizeof(motors) /
    sizeof(motors[0]);


static const uint8_t GRIPPER_ID =
    15;


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
  for (size_t i = 0;
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

  if (dxl.getLastLibErrCode() != DXL_LIB_OK)
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

  if (dxl.getLastLibErrCode() != DXL_LIB_OK)
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
// TORQUE OFF
// ============================================================

void torqueOffAll()
{
  for (size_t i = 0;
       i < MOTOR_COUNT;
       i++)
  {
    dxl.torqueOff(
        motors[i].id
    );
  }

  dxl.torqueOff(
      GRIPPER_ID
  );

  Serial.println(
      "ALL TORQUE OFF"
  );
}


// ============================================================
// TORQUE ON
// ============================================================

bool torqueOnArm()
{
  for (size_t i = 0;
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

  Serial.println(
      "ARM TORQUE ON"
  );

  return true;
}


// ============================================================
// HOLD ONE JOINT
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
      "Torque was left ON. Inspect the arm, then press B to clear the hold."
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
        "BUS READ FAILED: torque left unchanged; inspect bus/power, then press B."
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
        "BUS READ FAILED: torque left unchanged; inspect bus/power, then press B."
    );

    return false;
  }


  // Bit 3 = following error

  if ((movingStatus & 0x08U) != 0)
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
// GET CURRENT HOME-PATH OFFSET
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

  for (int k = -3;
       k <= 3;
       k++)
  {
    int32_t candidateRaw =
        currentRaw +
        k * COUNTS_PER_REV;


    if (candidateRaw >=
            lowerRaw -
            HOME_PATH_ENTRY_MARGIN_COUNTS &&
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


  for (int k = -3;
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
      distance =
          0;
    }


    if (distance < bestDistance)
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


  Serial.print(
      "HOME PATH CURRENT OFFSET = "
  );

  Serial.print(
      currentOffset,
      2
  );

  Serial.print(
      " deg | TARGET OFFSET = "
  );

  Serial.print(
      targetOffset,
      2
  );

  Serial.println(
      " deg"
  );


  Serial.print(
      "HOME RAW = "
  );

  Serial.print(
      homeRaw
  );

  Serial.print(
      " | CURRENT PATH RAW = "
  );

  Serial.print(
      currentCanonicalRaw
  );

  Serial.print(
      " | TARGET PATH RAW = "
  );

  Serial.println(
      targetRaw
  );


  return true;
}


// ============================================================
// DISPLAY OFFSET
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


  while (difference > 180.0f)
    difference -= 360.0f;


  while (difference < -180.0f)
    difference += 360.0f;


  return difference;
}


// ============================================================
// SHOW MOTOR
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
// SHOW ALL POSITIONS
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


  for (size_t i = 0;
       i < MOTOR_COUNT;
       i++)
  {
    showMotor(
        motors[i]
    );
  }


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
    // --------------------------------------------------------
    // Hardware Error check
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // Read actual position
    // --------------------------------------------------------

    int32_t currentRaw =
        0;


    if (!readPosition(
            id,
            currentRaw))
    {
      emergencyStop =
          true;

      Serial.println(
          "POSITION READ FAILED: inspect bus/power, then press B."
      );

      return false;
    }


    int32_t difference =
        currentRaw -
        targetRaw;


    if (difference < 0)
      difference =
          -difference;


    if (difference <=
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
        "Press B to re-enable torque."
    );

    return false;
  }


  MotorConfig *motor =
      findMotor(
          id
      );


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


  // ----------------------------------------------------------
  // Hardware fault check
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Calculate HOME target
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Extended position safety
  // ----------------------------------------------------------

  if (targetRaw <
          EXTENDED_MIN ||
      targetRaw >
          EXTENDED_MAX)
  {
    Serial.println(
        "BLOCKED: target outside extended position range."
    );

    return false;
  }


  float currentAngle =
      rawToEncoderDegrees(
          currentRaw
      );


  float targetEncoderAngle =
      rawToEncoderDegrees(
          targetRaw
      );


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
      currentAngle,
      2
  );

  Serial.print(
      " deg"
  );


  Serial.print(
      " | TARGET ENCODER="
  );

  Serial.print(
      targetEncoderAngle,
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


  // ----------------------------------------------------------
  // TIME-BASED PROFILE VELOCITY
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // TIME-BASED PROFILE ACCELERATION
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // SEND TARGET
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // WAIT
  // ----------------------------------------------------------

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
  // ----------------------------------------------------------
  // Find current calibrated HOME coordinate
  // ----------------------------------------------------------

  if (!getCurrentHomePathOffset(
          motor,
          currentRaw,
          currentOffset))
  {
    return false;
  }


  // ----------------------------------------------------------
  // Calculate new HOME offset
  // ----------------------------------------------------------

  targetOffset =
      currentOffset +
      direction *
      STEP_DEGREES;


  // ----------------------------------------------------------
  // Safety limits
  // ----------------------------------------------------------

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


  if (targetOffset <
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


  if (targetOffset >
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


  // ----------------------------------------------------------
  // Canonical HOME target
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Reconstruct current canonical HOME-path raw
  // ----------------------------------------------------------

  int32_t currentCanonicalRaw =
      homeRaw +
      (int32_t)lroundf(
          currentOffset *
          COUNTS_PER_DEGREE
      );


  // ----------------------------------------------------------
  // Find current encoder revolution
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Extended position safety
  // ----------------------------------------------------------

  if (targetRaw <
          EXTENDED_MIN ||
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
// MOVE 1-DEGREE STEP
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
      findMotor(
          id
      );


  if (motor == NULL)
    return;


  int32_t currentRaw =
      0;


  if (!readPosition(
          id,
          currentRaw))
    return;


  // ----------------------------------------------------------
  // Hardware fault check
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Calculate corrected step target
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // Display
  // ----------------------------------------------------------

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


  Serial.print(
      "TARGET ENCODER="
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


  // ----------------------------------------------------------
  // TIME-BASED PROFILE
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // SEND TARGET
  // ----------------------------------------------------------

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


  // ----------------------------------------------------------
  // WAIT
  // ----------------------------------------------------------

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
      findMotor(
          id
      );


  if (motor == NULL)
    return false;


  float targetOffset =
      0.0f;


  // ----------------------------------------------------------
  // ID11
  // ----------------------------------------------------------

  if (id == 11)
  {
    if (fabs(
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


  // ----------------------------------------------------------
  // ID12
  // ----------------------------------------------------------

  else if (id == 12)
  {
    if (fabs(
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


  // ----------------------------------------------------------
  // ID13
  // ----------------------------------------------------------

  else if (id == 13)
  {
    if (fabs(
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


  // ----------------------------------------------------------
  // ID14
  // ----------------------------------------------------------

  else if (id == 14)
  {
    if (fabs(
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
// GO TO CALIBRATION LIMIT
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
      HOME_PROFILE_VELOCITY,
      HOME_PROFILE_ACCELERATION
  );
}


// ============================================================
// GO HOME ONE MOTOR
// ============================================================

bool goHomeMotor(
    uint8_t id)
{
  MotorConfig *motor =
      findMotor(
          id
      );


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
// INITIALIZE MOTOR
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
  // TIME-BASED PROFILE
  // ----------------------------------------------------------
  //
  // Drive Mode bit 2 = 1
  //
  // 0x04 = Time-Based Profile
  //
  // Bit 0 = 0, therefore normal direction is preserved.
  //
  // ----------------------------------------------------------

  if (!dxl.writeControlTableItem(
          ControlTableItem::DRIVE_MODE,
          motor.id,
          TIME_BASED_DRIVE_MODE))
  {
    printBusError(
        "set TIME-BASED DRIVE MODE",
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
      " TIME-BASED PROFILE ENABLED"
  );


  // ----------------------------------------------------------
  // Profile Velocity
  // ----------------------------------------------------------
  //
  // In Time-Based Profile:
  //
  // value = milliseconds
  //
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
// HELP
// ============================================================

void printHelp()
{
  Serial.println();

  Serial.println(
      "=============================================="
  );

  Serial.println(
      "OPENMANIPULATOR-X COMMANDS"
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
      "TIME-BASED PROFILE:"
  );

  Serial.println(
      "Normal movement = 4000 ms"
  );

  Serial.println(
      "Home/limit movement = 6000 ms"
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
      "GENERAL:"
  );

  Serial.println(
      "H = HOME ALL"
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
      "HOME-PATH CALIBRATION CONTROLLER V6"
  );

  Serial.println(
      "XM430 TIME-BASED SLOW MOTION"
  );

  Serial.println(
      "=============================================="
  );


  // ----------------------------------------------------------
  // DYNAMIXEL BUS
  // ----------------------------------------------------------

  dxl.begin(
      1000000
  );

  dxl.setPortProtocolVersion(
      2.0
  );


  // ----------------------------------------------------------
  // Initialize motors
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
          "INITIALIZATION FAILED"
      );

      torqueOffAll();

      return;
    }
  }


  // ----------------------------------------------------------
  // Gripper torque OFF
  // ----------------------------------------------------------

  if (dxl.ping(
          GRIPPER_ID))
  {
    dxl.torqueOff(
        GRIPPER_ID
    );

    Serial.println(
        "Gripper ID15 torque OFF"
    );
  }


  // ----------------------------------------------------------
  // Enable arm torque
  // ----------------------------------------------------------

  if (!torqueOnArm())
  {
    Serial.println(
        "ARM TORQUE FAILED"
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
      "TIME-BASED SLOW MOTION ACTIVE."
  );

  Serial.println(
      "No automatic movement."
  );

  Serial.println(
      "Press H to move to HOME."
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


  if (key == '\r' ||
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
    if (torqueOnArm())
    {
      emergencyStop =
          false;

      motionHoldLatched =
          false;

      Serial.println(
          "Torque enabled; motion hold cleared."
      );
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