#include <Dynamixel2Arduino.h>
#include <math.h>

// OpenMANIPULATOR-X OpenCR motor bridge.
// Kinematics live in Python. This firmware handles DYNAMIXEL IO, torque,
// one-shot angle reads, and slow interpolated motor movement.

#define DXL_SERIAL Serial3

static const uint8_t DXL_DIR_PIN = 84;
static const uint32_t PC_BAUDRATE = 115200;
static const uint32_t DXL_BAUDRATE = 1000000;
static const float DXL_PROTOCOL_VERSION = 2.0f;

static const int32_t COUNTS_PER_REV = 4096;
static const float COUNTS_PER_DEGREE = COUNTS_PER_REV / 360.0f;
// A requested angle is rounded to the nearest integer encoder count. Permit
// half a count (plus a small floating-point guard) at configured boundaries.
// This does not widen the commanded joint range.
static const float JOINT_LIMIT_RAW_TOLERANCE_DEG =
  0.5f / COUNTS_PER_DEGREE + 0.001f;

static const uint8_t MOTOR_IDS[] = {11, 12, 13, 14};
static const size_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

// Straight raised REST / calibration zero: RAW {1917,2046,4049,0}
// means q1=q2=q3=q4=0. HOME remains a compatibility alias for REST.
static const int32_t HOME_RAW[] = {1917, 2046, 4049, 0};
static const float HOME_ANGLES[] = {
  1917.0f * 360.0f / 4096.0f,
  2046.0f * 360.0f / 4096.0f,
  4049.0f * 360.0f / 4096.0f,
  0.0f
};
// Downward WORK pose captured previously: q={0,0,0,82.880859} deg and
// calibrated center-between-fingers TCP approximately {179.668,0,47.023} mm.
static const float WORK_ANGLES[] = {
  168.486328125f,
  179.824218750f,
  355.869140625f,
  82.880859375f
};
static const float JOINT_DIRECTION[] = {+1.0f, +1.0f, +1.0f, +1.0f};
// ID11/q1 expanded for P01/P07: required angles are approximately
// -103.57/+103.57 deg. The other calibrated joint limits remain unchanged.
static const float JOINT_MIN_DEG[] = {-110.0f, -15.0f, -60.0f, -45.0f};
static const float JOINT_MAX_DEG[] = {110.0f, 85.0f, 90.0f, 100.0f};

static const float Z0 = 76.5f;
static const float BASE_X = 12.0f;
static const float L2 = 130.2305648f;
static const float ALPHA2_0 = 1.385105f;
static const float L3_X = 124.0f;
// Official gripper-frame origin is 126 mm. The experiment TCP is the center
// between the two finger tips. A ground check at X=180, Y=0 found the original
// 39.7 mm extension placed the real TCP 7 mm above ground when FK reported
// Z=0, so the calibrated axial extension is 32.7 mm and total L4 is 158.7 mm.
static const float L4_X = 158.7f;
static const uint32_t SAMPLE_INTERVAL_MS = 50;
static const uint32_t MAX_RECORDING_MS = 30000;
static const size_t MAX_TRAJECTORY_POINTS = 600;

// S-curve smooth interpolation parameters
static const uint32_t STEP_INTERVAL_MS = 20; // 50 Hz control loop
static const uint32_t MIN_MOVE_TIME_MS = 120;
static const uint32_t MAX_MOVE_TIME_MS = 4500;
static float motionSpeedScale = 1.0f;
static const int32_t FINAL_POSITION_TOLERANCE_COUNTS = 8; // ~0.7 degrees tolerance (tightened for precision)
// Try the precise threshold for the full settling window, then accept up to
// 10 degrees for data collection. Joint/motor limits remain independent.
static const float DATA_COLLECTION_ARRIVAL_TOLERANCE_DEG = 10.0f;
static const uint32_t FINAL_WAIT_TIMEOUT_MS = 3000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

bool torqueEnabled = false;
String rxLine = "";

struct TrajectoryPoint
{
  float time;
  float j1;
  float j2;
  float j3;
  float j4;
  float x;
  float y;
  float z;
};

struct MotorTelemetry
{
  int32_t positionRaw;
  int32_t velocityRaw;
  int32_t currentRaw;
  int32_t pwmRaw;
  int32_t voltageRaw;
  int32_t temperatureC;
  int32_t hardwareError;
  int32_t moving;
  int32_t movingStatus;
};

TrajectoryPoint trajectory[MAX_TRAJECTORY_POINTS];
size_t trajectoryCount = 0;
bool teaching = false;
uint32_t teachingStarted = 0;
uint32_t lastSample = 0;

int32_t normalizeRaw(int32_t raw)
{
  raw %= COUNTS_PER_REV;
  if (raw < 0)
    raw += COUNTS_PER_REV;
  return raw;
}

float rawToDegrees(int32_t raw)
{
  return normalizeRaw(raw) / COUNTS_PER_DEGREE;
}

int32_t nearestHomeDelta(int32_t raw, int32_t homeRaw)
{
  int32_t delta = raw - homeRaw;
  while (delta > COUNTS_PER_REV / 2) delta -= COUNTS_PER_REV;
  while (delta < -COUNTS_PER_REV / 2) delta += COUNTS_PER_REV;
  return delta;
}

float rawToCalibratedJoint(size_t index, int32_t raw)
{
  return JOINT_DIRECTION[index]
       * (float)nearestHomeDelta(raw, HOME_RAW[index])
       / COUNTS_PER_DEGREE;
}

bool safeCalibratedRaw(size_t index, int32_t raw)
{
  float joint = rawToCalibratedJoint(index, raw);
  return isfinite(joint)
      && joint >= JOINT_MIN_DEG[index] - JOINT_LIMIT_RAW_TOLERANCE_DEG
      && joint <= JOINT_MAX_DEG[index] + JOINT_LIMIT_RAW_TOLERANCE_DEG;
}

bool validDegrees(float value)
{
  return isfinite(value) && value >= 0.0f && value < 360.0f;
}

int32_t degreesToNearestRaw(float degrees, int32_t currentRaw)
{
  int32_t canonical = (int32_t)lroundf(degrees * COUNTS_PER_DEGREE);
  int32_t best = canonical;
  int32_t bestDistance = labs(best - currentRaw);

  for (int turn = -8; turn <= 8; turn++)
  {
    int32_t candidate = canonical + turn * COUNTS_PER_REV;
    int32_t distance = labs(candidate - currentRaw);
    if (distance < bestDistance)
    {
      best = candidate;
      bestDistance = distance;
    }
  }

  return best;
}

void printBusError(const char *operation, uint8_t id)
{
  Serial.print("ERROR,BUS,");
  Serial.print(operation);
  Serial.print(",ID");
  Serial.print(id);
  Serial.print(",LIB");
  Serial.print(dxl.getLastLibErrCode());
  Serial.print(",STATUS");
  Serial.println(dxl.getLastStatusPacketError());
}

bool readRawPosition(uint8_t id, int32_t &position)
{
  position = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK)
  {
    printBusError("READ_POSITION", id);
    return false;
  }
  return true;
}

bool readTelemetryItem(
  uint8_t id,
  uint8_t item,
  const char *itemName,
  int32_t &value)
{
  value = dxl.readControlTableItem(item, id);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK)
  {
    Serial.print("ERROR,BUS,READ_TELEMETRY_");
    Serial.print(itemName);
    Serial.print(",ID");
    Serial.print(id);
    Serial.print(",LIB");
    Serial.print(dxl.getLastLibErrCode());
    Serial.print(",STATUS");
    Serial.println(dxl.getLastStatusPacketError());
    return false;
  }
  return true;
}

bool readMotorTelemetry(uint8_t id, MotorTelemetry &sample)
{
  return readRawPosition(id, sample.positionRaw)
      && readTelemetryItem(id, ControlTableItem::PRESENT_VELOCITY, "VELOCITY", sample.velocityRaw)
      && readTelemetryItem(id, ControlTableItem::PRESENT_CURRENT, "CURRENT", sample.currentRaw)
      && readTelemetryItem(id, ControlTableItem::PRESENT_PWM, "PWM", sample.pwmRaw)
      && readTelemetryItem(id, ControlTableItem::PRESENT_INPUT_VOLTAGE, "VOLTAGE", sample.voltageRaw)
      && readTelemetryItem(id, ControlTableItem::PRESENT_TEMPERATURE, "TEMPERATURE", sample.temperatureC)
      && readTelemetryItem(id, ControlTableItem::HARDWARE_ERROR_STATUS, "HW_ERROR", sample.hardwareError)
      && readTelemetryItem(id, ControlTableItem::MOVING, "MOVING", sample.moving)
      && readTelemetryItem(id, ControlTableItem::MOVING_STATUS, "MOVING_STATUS", sample.movingStatus);
}

void sendTelemetry()
{
  MotorTelemetry samples[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!readMotorTelemetry(MOTOR_IDS[i], samples[i]))
      return;
  }

  Serial.print("TELEMETRY_V1");
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(","); Serial.print(MOTOR_IDS[i]);
    Serial.print(","); Serial.print(samples[i].positionRaw);
    Serial.print(","); Serial.print(samples[i].velocityRaw);
    Serial.print(","); Serial.print(samples[i].currentRaw);
    Serial.print(","); Serial.print(samples[i].pwmRaw);
    Serial.print(","); Serial.print(samples[i].voltageRaw);
    Serial.print(","); Serial.print(samples[i].temperatureC);
    Serial.print(","); Serial.print(samples[i].hardwareError);
    Serial.print(","); Serial.print(samples[i].moving);
    Serial.print(","); Serial.print(samples[i].movingStatus);
  }
  Serial.println();
}

bool setGoalRaw(uint8_t id, int32_t raw)
{
  if (!dxl.setGoalPosition(id, raw, UNIT_RAW))
  {
    printBusError("SET_GOAL", id);
    return false;
  }
  return true;
}

bool synchronizeGoalsToPresent()
{
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    int32_t currentRaw = 0;
    uint8_t id = MOTOR_IDS[i];
    if (!readRawPosition(id, currentRaw))
      return false;
    if (!setGoalRaw(id, currentRaw))
      return false;
  }
  return true;
}

bool torqueOffAll()
{
  bool ok = true;
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!dxl.torqueOff(MOTOR_IDS[i]))
    {
      printBusError("TORQUE_OFF", MOTOR_IDS[i]);
      ok = false;
    }
  }
  torqueEnabled = false;
  return ok;
}

bool torqueOnAll()
{
  if (!synchronizeGoalsToPresent())
    return false;

  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!dxl.torqueOn(MOTOR_IDS[i]))
    {
      printBusError("TORQUE_ON", MOTOR_IDS[i]);
      torqueOffAll();
      return false;
    }
  }
  torqueEnabled = true;
  return true;
}

bool stopMotionIfRequested()
{
  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == '\n')
    {
      rxLine.trim();
      rxLine.toUpperCase();
      bool stop = rxLine == "STOP";
      rxLine = "";
      if (stop)
      {
        synchronizeGoalsToPresent();
        Serial.println("STOPPED");
        return true;
      }
      Serial.println("ERROR,BUSY");
    }
    else if (ch != '\r' && rxLine.length() < 120)
    {
      rxLine += ch;
    }
  }
  return false;
}

bool configureMotor(uint8_t id)
{
  if (!dxl.ping(id))
  {
    printBusError("PING", id);
    return false;
  }

  if (!dxl.torqueOff(id))
  {
    printBusError("TORQUE_OFF", id);
    return false;
  }

  if (!dxl.setOperatingMode(id, OP_EXTENDED_POSITION))
  {
    printBusError("OPERATING_MODE", id);
    return false;
  }

  // Low internal profile values because this sketch performs explicit
  // interpolation. These values still soften each small DYNAMIXEL step.
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 80);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, 30);
  
  // Add integral gain to overcome steady-state gravity sag
  dxl.writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, 100);
  return true;
}

void sendAngles()
{
  float angles[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    int32_t raw = 0;
    if (!readRawPosition(MOTOR_IDS[i], raw))
      return;
    angles[i] = rawToDegrees(raw);
  }

  Serial.print("ANGLES");
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(",");
    Serial.print(angles[i], 3);
  }
  Serial.println();
}

void sendState()
{
  int32_t raw[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!readRawPosition(MOTOR_IDS[i], raw[i]))
      return;
  }

  // One coherent diagnostic sample: continuous raw positions first, then
  // canonical motor degrees and calibrated joints. Python derives both FK
  // coordinate frames from these same motor readings.
  Serial.print("STATE");
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(",");
    Serial.print(raw[i]);
  }
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(",");
    Serial.print(rawToDegrees(raw[i]), 3);
  }
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(",");
    Serial.print(rawToCalibratedJoint(i, raw[i]), 3);
  }
  Serial.println();
}

void sendJointLimits()
{
  // Lets the PC verify that offline IK and firmware motion protection use the
  // same limits before declaring points hardware-ready.
  Serial.print("JOINT_LIMITS");
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    Serial.print(",");
    Serial.print(JOINT_MIN_DEG[i], 3);
    Serial.print(",");
    Serial.print(JOINT_MAX_DEG[i], 3);
  }
  Serial.print(",RAW_TOLERANCE,");
  Serial.print(JOINT_LIMIT_RAW_TOLERANCE_DEG, 3);
  Serial.println();
}

bool waitForFinalTargets(const int32_t targets[])
{
  uint32_t started = millis();
  while (millis() - started < FINAL_WAIT_TIMEOUT_MS)
  {
    if (stopMotionIfRequested())
      return false;
    bool allReached = true;
    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
      int32_t current = 0;
      if (!readRawPosition(MOTOR_IDS[i], current))
        return false;
      if (labs(current - targets[i]) > FINAL_POSITION_TOLERANCE_COUNTS)
        allReached = false;
    }
    if (allReached)
      return true;
    delay(50);
  }
  float largestErrorDegrees = 0.0f;
  uint8_t largestErrorId = 0;
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    int32_t current = 0;
    if (!readRawPosition(MOTOR_IDS[i], current))
      return false;
    float errorDegrees = fabsf((float)(current - targets[i]) / COUNTS_PER_DEGREE);
    if (errorDegrees > largestErrorDegrees)
    {
      largestErrorDegrees = errorDegrees;
      largestErrorId = MOTOR_IDS[i];
    }
  }
  if (largestErrorDegrees <= DATA_COLLECTION_ARRIVAL_TOLERANCE_DEG)
    return true;
  Serial.print("ERROR,TARGET_NOT_REACHED,ID");
  Serial.print(largestErrorId);
  Serial.print(",ERROR_DEG,");
  Serial.println(largestErrorDegrees, 3);
  return false;
}

bool moveToDegrees(const float targetDegrees[])
{
  if (!torqueEnabled)
  {
    Serial.println("ERROR,TORQUE_OFF");
    return false;
  }

  int32_t currentRaw[MOTOR_COUNT];
  int32_t targetRaw[MOTOR_COUNT];
  int32_t delta[MOTOR_COUNT];
  int32_t maxDelta = 0;

  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!validDegrees(targetDegrees[i]))
    {
      Serial.println("ERROR,INVALID_ANGLE");
      return false;
    }
    if (!readRawPosition(MOTOR_IDS[i], currentRaw[i]))
      return false;
    targetRaw[i] = degreesToNearestRaw(targetDegrees[i], currentRaw[i]);
    if (!safeCalibratedRaw(i, targetRaw[i]))
    {
      Serial.print("ERROR,JOINT_LIMIT,ID");
      Serial.print(MOTOR_IDS[i]);
      Serial.print(",Q_DEG,");
      Serial.println(rawToCalibratedJoint(i, targetRaw[i]), 3);
      return false;
    }
    delta[i] = targetRaw[i] - currentRaw[i];
    if (labs(delta[i]) > maxDelta)
      maxDelta = labs(delta[i]);
  }

  // Calculate adaptive duration based on max motor angular distance
  float maxDegreesDisplacement = (float)maxDelta / COUNTS_PER_DEGREE;
  uint32_t moveDurationMs = (uint32_t)(1000.0f + maxDegreesDisplacement * 20.0f);
  if (moveDurationMs < MIN_MOVE_TIME_MS)
    moveDurationMs = MIN_MOVE_TIME_MS;
  if (moveDurationMs > MAX_MOVE_TIME_MS)
    moveDurationMs = MAX_MOVE_TIME_MS;
  moveDurationMs = (uint32_t)((float)moveDurationMs / motionSpeedScale);

  uint16_t steps = (uint16_t)(moveDurationMs / STEP_INTERVAL_MS);
  if (steps < 1)
    steps = 1;

  Serial.println("MOVING");

  // S-Curve (minimum-jerk cosine ramp) trajectory generation
  for (uint16_t step = 1; step <= steps; step++)
  {
    if (stopMotionIfRequested())
      return false;
    float ratio = (float)step / (float)steps;
    // Cosine S-curve profile: zero initial and final velocity
    float sRatio = 0.5f * (1.0f - cosf(3.14159265358979323846f * ratio));

    for (size_t i = 0; i < MOTOR_COUNT; i++)
    {
      int32_t intermediate = currentRaw[i] + (int32_t)lroundf(delta[i] * sRatio);
      if (!setGoalRaw(MOTOR_IDS[i], intermediate))
        return false;
    }
    delay(STEP_INTERVAL_MS);
  }

  if (!waitForFinalTargets(targetRaw))
    return false;

  Serial.println("DONE");
  return true;
}

float normalizeJoint(float angle)
{
  while (angle >= 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void calculateFK(float j1, float j2, float j3, float j4, float &x, float &y, float &z)
{
  float t1 = j1 * 3.14159265358979323846f / 180.0f;
  float t2 = j2 * 3.14159265358979323846f / 180.0f;
  float t3 = j3 * 3.14159265358979323846f / 180.0f;
  float t4 = j4 * 3.14159265358979323846f / 180.0f;
  float phi2 = ALPHA2_0 - t2;
  float phi3 = -(t2 + t3);
  float phi4 = -(t2 + t3 + t4);
  float radial = BASE_X + L2 * cosf(phi2) + L3_X * cosf(phi3) + L4_X * cosf(phi4);
  z = Z0 + L2 * sinf(phi2) + L3_X * sinf(phi3) + L4_X * sinf(phi4);
  x = radial * cosf(t1);
  y = radial * sinf(t1);
}

bool recordTrajectoryPoint()
{
  if (trajectoryCount >= MAX_TRAJECTORY_POINTS)
    return false;
  float motors[MOTOR_COUNT];
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    int32_t raw = 0;
    if (!readRawPosition(MOTOR_IDS[i], raw)) return false;
    motors[i] = rawToDegrees(raw);
  }
  float j1 = JOINT_DIRECTION[0] * normalizeJoint(motors[0] - HOME_ANGLES[0]);
  float j2 = JOINT_DIRECTION[1] * normalizeJoint(motors[1] - HOME_ANGLES[1]);
  float j3 = JOINT_DIRECTION[2] * normalizeJoint(motors[2] - HOME_ANGLES[2]);
  float j4 = JOINT_DIRECTION[3] * normalizeJoint(motors[3] - HOME_ANGLES[3]);
  TrajectoryPoint &point = trajectory[trajectoryCount++];
  point.time = (millis() - teachingStarted) / 1000.0f;
  point.j1 = j1 * 3.14159265358979323846f / 180.0f;
  point.j2 = j2 * 3.14159265358979323846f / 180.0f;
  point.j3 = j3 * 3.14159265358979323846f / 180.0f;
  point.j4 = j4 * 3.14159265358979323846f / 180.0f;
  calculateFK(j1, j2, j3, j4, point.x, point.y, point.z);
  return true;
}

void sendTrajectoryToPython()
{
  float duration = trajectoryCount > 0 ? trajectory[trajectoryCount - 1].time : 0.0f;
  for (size_t i = 0; i < trajectoryCount; i++)
  {
    const TrajectoryPoint &point = trajectory[i];
    Serial.print("TRAJ,"); Serial.print(point.time, 3); Serial.print(",");
    Serial.print(point.j1, 6); Serial.print(","); Serial.print(point.j2, 6); Serial.print(",");
    Serial.print(point.j3, 6); Serial.print(","); Serial.print(point.j4, 6); Serial.print(",");
    Serial.print(point.x, 3); Serial.print(","); Serial.print(point.y, 3); Serial.print(",");
    Serial.println(point.z, 3);
  }
  Serial.print("TRAJ_END,"); Serial.print(trajectoryCount); Serial.print(","); Serial.println(duration, 3);
}

void startTeaching()
{
  trajectoryCount = 0;
  teaching = false;
  torqueOffAll();
  teachingStarted = millis();
  lastSample = teachingStarted - SAMPLE_INTERVAL_MS;
  teaching = true;
  Serial.println("TEACHING");
}

void stopTeaching()
{
  if (!teaching) { Serial.println("TRAJ_END,0,0.000"); return; }
  teaching = false;
  recordTrajectoryPoint();
  Serial.print("TEACH_STOPPED,"); Serial.println(trajectoryCount);
  sendTrajectoryToPython();
}

bool parseMoveCommand(const String &line, float targets[])
{
  int start = 0;
  int part = 0;

  while (start <= line.length())
  {
    int comma = line.indexOf(',', start);
    String token = comma >= 0 ? line.substring(start, comma) : line.substring(start);
    token.trim();

    if (part == 0)
    {
      if (token != "MOVE_MOTORS")
        return false;
    }
    else if (part >= 1 && part <= 4)
    {
      targets[part - 1] = token.toFloat();
      if (!validDegrees(targets[part - 1]))
        return false;
    }
    else
    {
      return false;
    }

    if (comma < 0)
      break;
    start = comma + 1;
    part++;
  }

  return part == 4;
}

void handleCommand(String line)
{
  line.trim();
  line.toUpperCase();

  if (line.length() == 0)
    return;

  if (line == "PING")
  {
    Serial.println("OK,PONG");
  }
  else if (line == "START_TEACH")
  {
    startTeaching();
  }
  else if (line == "STOP_TEACH")
  {
    stopTeaching();
  }
  else if (line == "CLEAR_TRAJECTORY")
  {
    teaching = false;
    trajectoryCount = 0;
    Serial.println("TRAJECTORY_CLEARED");
  }
  else if (line == "STATUS")
  {
    Serial.print("STATUS,TEACHING="); Serial.print(teaching ? "ON" : "OFF");
    Serial.print(",POINTS="); Serial.println(trajectoryCount);
  }
  else if (line.startsWith("SET_SPEED,"))
  {
    float scale = line.substring(10).toFloat();
    if (!isfinite(scale) || scale < 0.25f || scale > 1.0f || teaching)
      Serial.println("ERROR,INVALID_SPEED");
    else
    {
      motionSpeedScale = scale;
      Serial.println("OK,SPEED");
    }
  }
  else if (line == "TORQUE_STATUS")
  {
    Serial.println(torqueEnabled ? "TORQUE:ON" : "TORQUE:OFF");
  }
  else if (line == "TORQUE_ON")
  {
    if (torqueOnAll())
      Serial.println("TORQUE:ON");
  }
  else if (line == "TORQUE_OFF")
  {
    torqueOffAll();
    Serial.println("TORQUE:OFF");
  }
  else if (line == "READ_ANGLES")
  {
    sendAngles();
  }
  else if (line == "READ_STATE")
  {
    sendState();
  }
  else if (line == "JOINT_LIMITS")
  {
    sendJointLimits();
  }
  else if (line == "CAPABILITIES")
  {
    Serial.println("CAPABILITIES,STATE,TELEMETRY_V1");
  }
  else if (line == "READ_TELEMETRY")
  {
    sendTelemetry();
  }
  else if (line == "REST" || line == "HOME")
  {
    moveToDegrees(HOME_ANGLES);
  }
  else if (line == "WORK")
  {
    moveToDegrees(WORK_ANGLES);
  }
  else if (line == "STOP")
  {
    Serial.println("IDLE");
  }
  else if (line.startsWith("MOVE_MOTORS"))
  {
    float targets[MOTOR_COUNT];
    if (!parseMoveCommand(line, targets))
    {
      Serial.println("ERROR,BAD_COMMAND");
      return;
    }
    moveToDegrees(targets);
  }
  else
  {
    Serial.println("ERROR,UNKNOWN_COMMAND");
  }
}

void setup()
{
  Serial.begin(PC_BAUDRATE);
  DXL_SERIAL.begin(DXL_BAUDRATE);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  delay(500);

  bool ok = true;
  for (size_t i = 0; i < MOTOR_COUNT; i++)
  {
    if (!configureMotor(MOTOR_IDS[i]))
      ok = false;
  }

  torqueOffAll();

  if (ok)
    Serial.println("OK,READY");
  else
    Serial.println("ERROR,INIT_FAILED");
}

void loop()
{
  if (teaching && millis() - teachingStarted >= MAX_RECORDING_MS)
    stopTeaching();
  if (teaching && millis() - lastSample >= SAMPLE_INTERVAL_MS)
  {
    lastSample = millis();
    if (!recordTrajectoryPoint())
    {
      teaching = false;
      Serial.println("ERROR,RECORDING_BUFFER_FULL");
    }
  }
  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == '\n')
    {
      handleCommand(rxLine);
      rxLine = "";
    }
    else if (ch != '\r')
    {
      if (rxLine.length() < 120)
        rxLine += ch;
      else
      {
        rxLine = "";
        Serial.println("ERROR,LINE_TOO_LONG");
      }
    }
  }
}
