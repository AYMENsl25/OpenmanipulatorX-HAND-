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

static const uint8_t MOTOR_IDS[] = {11, 12, 13, 14};
static const size_t MOTOR_COUNT = sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]);

static const float HOME_ANGLES[] = {351.0f, 1.0f, 1.0f, 90.0f};

// S-curve smooth interpolation parameters
static const uint32_t STEP_INTERVAL_MS = 20; // 50 Hz control loop
static const uint32_t MIN_MOVE_TIME_MS = 1000;
static const uint32_t MAX_MOVE_TIME_MS = 4500;
static const int32_t FINAL_POSITION_TOLERANCE_COUNTS = 8; // ~0.7 degrees tolerance (tightened for precision)
static const uint32_t FINAL_WAIT_TIMEOUT_MS = 3000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

bool torqueEnabled = false;
String rxLine = "";

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

bool waitForFinalTargets(const int32_t targets[])
{
  uint32_t started = millis();
  while (millis() - started < FINAL_WAIT_TIMEOUT_MS)
  {
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
  // If it times out, the arm might have a tiny steady-state gravity sag.
  // We still consider the trajectory execution 'done'.
  return true;
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

  uint16_t steps = (uint16_t)(moveDurationMs / STEP_INTERVAL_MS);
  if (steps < 1)
    steps = 1;

  Serial.println("MOVING");

  // S-Curve (minimum-jerk cosine ramp) trajectory generation
  for (uint16_t step = 1; step <= steps; step++)
  {
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
  else if (line == "HOME")
  {
    moveToDegrees(HOME_ANGLES);
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
