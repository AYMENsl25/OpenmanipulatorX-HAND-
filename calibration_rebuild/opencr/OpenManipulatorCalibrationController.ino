#include <Dynamixel2Arduino.h>
#include <math.h>

// Measurement-only firmware: no FK, IK, sweeps, or automatic movement.
// No EEPROM configuration registers are changed.
#define DXL_SERIAL Serial3
#define DXL_DIR_PIN 84
#define DEBUG_SERIAL Serial

static const uint32_t USB_BAUD = 115200;
static const uint32_t DXL_BAUD = 1000000;
static const float DXL_PROTOCOL = 2.0;
static const int32_t HOME_TOLERANCE_RAW = 10;
static const int32_t DIRECTION_MOVED_THRESHOLD_RAW = 12;
static const int32_t DIRECTION_MAX_TEST_MOVE_RAW = 256;
static const float DEGREES_PER_RAW = 360.0f / 4096.0f;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

struct MotorCalibration {
  uint8_t id;
  const char *name;
  int32_t min_raw;
  int32_t home_raw;
  int32_t max_raw;
  bool min_is_provisional;
};

// Exact measured RAW values are the source of truth.
static const MotorCalibration CALIBRATIONS[] = {
  {11, "J1 / BASE",      915, 1917, 2961, false},
  {12, "J2 / SHOULDER", 2040, 2046, 3000, false},
  {13, "J3 / ELBOW",    3570, 4049, 4062, true },
  {14, "J4 / WRIST",      71,  943,  995, false},
  // HOME=1210 is one count below OPEN=1211, so verification includes both.
  {15, "GRIPPER",        1210, 1210, 2672, false}
};

static const size_t MOTOR_COUNT = sizeof(CALIBRATIONS) / sizeof(CALIBRATIONS[0]);
static const int32_t ID15_OPEN_RAW = 1211;
static const int32_t ID15_HOME_RAW = 1210;
static const int32_t ID15_CLOSE_RAW = 2672;

int32_t directionReference[4] = {0, 0, 0, 0};
bool directionReferenceValid = false;

const MotorCalibration *findCalibration(uint8_t id) {
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
    if (CALIBRATIONS[i].id == id) return &CALIBRATIONS[i];
  return nullptr;
}

float rawToDegrees(int32_t raw) { return ((float)raw) * DEGREES_PER_RAW; }
float deltaRawToDegrees(int32_t raw) { return ((float)raw) * DEGREES_PER_RAW; }

int32_t canonicalRaw(int32_t raw) {
  int32_t value = raw % 4096;
  return value < 0 ? value + 4096 : value;
}

int32_t shortestRawDelta(int32_t current, int32_t reference) {
  int32_t delta = canonicalRaw(current) - canonicalRaw(reference);
  if (delta > 2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}

void printSignedInt(int32_t value) {
  if (value >= 0) DEBUG_SERIAL.print('+');
  DEBUG_SERIAL.print(value);
}

void printSignedFloat(float value) {
  if (value >= 0.0f) DEBUG_SERIAL.print('+');
  DEBUG_SERIAL.print(value, 3);
}

bool readRaw(uint8_t id, int32_t &raw) {
  float reading = dxl.getPresentPosition(id, UNIT_RAW);
  uint8_t libError = dxl.getLastLibErrCode();
  if (libError != 0 || isnan(reading)) {
    DEBUG_SERIAL.print("ERROR: BUS READ FAILED FOR ID");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print("; LIB ERROR = ");
    DEBUG_SERIAL.println(libError);
    return false;
  }
  raw = (int32_t)lroundf(reading);
  return true;
}

bool readAllRaw(int32_t values[MOTOR_COUNT]) {
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
    if (!readRaw(CALIBRATIONS[i].id, values[i])) return false;
  return true;
}

void printSeparator() { DEBUG_SERIAL.println("--------------------------------------"); }

void printMotorData(const MotorCalibration &cal, int32_t currentRaw,
                    bool includeHomeStatus) {
  int32_t currentCanonical = canonicalRaw(currentRaw);
  int32_t deltaHome = shortestRawDelta(currentRaw, cal.home_raw);
  int32_t distanceMin = currentCanonical - cal.min_raw;
  int32_t distanceMax = cal.max_raw - currentCanonical;
  bool inside = currentCanonical >= cal.min_raw && currentCanonical <= cal.max_raw;

  printSeparator();
  DEBUG_SERIAL.print("ID"); DEBUG_SERIAL.print(cal.id);
  DEBUG_SERIAL.print(" / "); DEBUG_SERIAL.println(cal.name);
  printSeparator();
  DEBUG_SERIAL.print("CONTINUOUS RAW       = "); DEBUG_SERIAL.println(currentRaw);
  DEBUG_SERIAL.print("CANONICAL RAW 0..4095= "); DEBUG_SERIAL.println(currentCanonical);
  DEBUG_SERIAL.print("CURRENT DEG          = "); DEBUG_SERIAL.println(rawToDegrees(currentCanonical), 3);
  DEBUG_SERIAL.println();

  if (cal.id == 15) {
    DEBUG_SERIAL.print("OPEN RAW             = "); DEBUG_SERIAL.println(ID15_OPEN_RAW);
    DEBUG_SERIAL.print("HOME RAW             = "); DEBUG_SERIAL.println(ID15_HOME_RAW);
    DEBUG_SERIAL.print("CLOSE RAW            = "); DEBUG_SERIAL.println(ID15_CLOSE_RAW);
    DEBUG_SERIAL.println("NOTE: verification minimum includes HOME RAW 1210.");
  } else {
    DEBUG_SERIAL.print("MIN RAW              = "); DEBUG_SERIAL.print(cal.min_raw);
    if (cal.min_is_provisional) DEBUG_SERIAL.print("  PROVISIONAL");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("HOME RAW             = "); DEBUG_SERIAL.println(cal.home_raw);
    DEBUG_SERIAL.print("MAX RAW              = "); DEBUG_SERIAL.println(cal.max_raw);
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("DELTA RAW FROM HOME  = "); printSignedInt(deltaHome); DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("DELTA DEG FROM HOME  = "); printSignedFloat(deltaRawToDegrees(deltaHome)); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("DISTANCE TO MIN      = "); DEBUG_SERIAL.print(distanceMin); DEBUG_SERIAL.println(" RAW");
  DEBUG_SERIAL.print("DISTANCE TO MAX      = "); DEBUG_SERIAL.print(distanceMax); DEBUG_SERIAL.println(" RAW");
  DEBUG_SERIAL.print("INSIDE LIMITS        = "); DEBUG_SERIAL.println(inside ? "YES" : "NO");
  if (!inside) DEBUG_SERIAL.println("!!! OUTSIDE MEASURED LIMIT !!!");
  if (cal.min_is_provisional) {
    DEBUG_SERIAL.println("WARNING:");
    DEBUG_SERIAL.println("ID13 MIN LIMIT IS PROVISIONAL");
  }
  if (includeHomeStatus) {
    DEBUG_SERIAL.print("HOME STATUS          = ");
    DEBUG_SERIAL.println(labs(deltaHome) <= HOME_TOLERANCE_RAW ? "NEAR HOME" : "NOT HOME");
  }
  DEBUG_SERIAL.println();
}

void checkHome() {
  int32_t current[MOTOR_COUNT];
  DEBUG_SERIAL.println("\n====================================");
  DEBUG_SERIAL.println("HOME CHECK - READ ONLY; NO MOVEMENT");
  DEBUG_SERIAL.println("====================================");
  if (!readAllRaw(current)) return;
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
    printMotorData(CALIBRATIONS[i], current[i], true);
}

void printOneMotor(uint8_t id) {
  const MotorCalibration *cal = findCalibration(id);
  if (cal == nullptr) { DEBUG_SERIAL.println("ERROR: UNKNOWN MOTOR ID"); return; }
  int32_t current;
  if (!readRaw(id, current)) return;
  printMotorData(*cal, current, false);
  int32_t delta = shortestRawDelta(current, cal->home_raw);
  DEBUG_SERIAL.print("RAW FROM STORED HOME = ");
  if (delta > 0) DEBUG_SERIAL.println("INCREASED");
  else if (delta < 0) DEBUG_SERIAL.println("DECREASED");
  else DEBUG_SERIAL.println("NO CHANGE");
  DEBUG_SERIAL.println("This does not define mathematical joint direction.\n");
}

void printAllMotors() {
  int32_t current[MOTOR_COUNT];
  DEBUG_SERIAL.println("\n====================================");
  DEBUG_SERIAL.println("ALL MOTOR CALIBRATION DATA");
  DEBUG_SERIAL.println("====================================");
  if (!readAllRaw(current)) return;
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
    printMotorData(CALIBRATIONS[i], current[i], false);
}

void torqueOffAll() {
  bool allOk = true;
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!dxl.torqueOff(CALIBRATIONS[i].id)) {
      allOk = false;
      DEBUG_SERIAL.print("WARNING: TORQUE OFF COMMAND FAILED FOR ID");
      DEBUG_SERIAL.println(CALIBRATIONS[i].id);
    }
  }
  DEBUG_SERIAL.println("\n====================================");
  DEBUG_SERIAL.println(allOk ? "TORQUE OFF" : "TORQUE OFF - CHECK WARNINGS");
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("Robot can now be moved carefully by hand.");
  DEBUG_SERIAL.println("Support the gravity-loaded arm before moving it.\n");
  DEBUG_SERIAL.println("JOINT DIRECTION TEST:");
  DEBUG_SERIAL.println("1. Put the joint at HOME.");
  DEBUG_SERIAL.println("2. Press S to save the current reference.");
  DEBUG_SERIAL.println("3. Move ONLY one joint slightly.");
  DEBUG_SERIAL.println("4. Press D to compare RAW direction.\n");
}

void safeTorqueOn() {
  int32_t present[MOTOR_COUNT];
  DEBUG_SERIAL.println("\nSAFE TORQUE ON - PREPARING PRESENT-POSITION HOLD");

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!dxl.torqueOff(CALIBRATIONS[i].id)) {
      DEBUG_SERIAL.print("ERROR: CANNOT CONFIRM TORQUE OFF FOR ID");
      DEBUG_SERIAL.println(CALIBRATIONS[i].id);
      return;
    }
  }
  if (!readAllRaw(present)) {
    DEBUG_SERIAL.println("SAFE TORQUE ON ABORTED: POSITION READ FAILED.");
    return;
  }
  // Goal Position is RAM; no EEPROM setting is changed.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    uint8_t id = CALIBRATIONS[i].id;
    if (!dxl.setGoalPosition(id, present[i], UNIT_RAW)) {
      DEBUG_SERIAL.print("ERROR: CANNOT COPY PRESENT POSITION TO GOAL FOR ID");
      DEBUG_SERIAL.println(id);
      return;
    }
  }
  delay(100);
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    uint8_t id = CALIBRATIONS[i].id;
    if (!dxl.torqueOn(id)) {
      DEBUG_SERIAL.print("ERROR: TORQUE ON FAILED FOR ID"); DEBUG_SERIAL.println(id);
      for (size_t j = 0; j < MOTOR_COUNT; ++j) dxl.torqueOff(CALIBRATIONS[j].id);
      DEBUG_SERIAL.println("SAFE TORQUE ON ABORTED; TORQUE OFF REQUESTED FOR ALL.");
      return;
    }
  }
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SAFE TORQUE ON");
  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("Current positions copied to Goal Position.");
  DEBUG_SERIAL.println("Torque enabled.");
  DEBUG_SERIAL.println("Robot should hold its current position.\n");
}

void saveDirectionReference() {
  for (size_t i = 0; i < 4; ++i) {
    if (!readRaw(CALIBRATIONS[i].id, directionReference[i])) {
      directionReferenceValid = false;
      DEBUG_SERIAL.println("DIRECTION REFERENCE NOT SAVED.");
      return;
    }
  }
  directionReferenceValid = true;
  DEBUG_SERIAL.println("\nDirection reference saved in RAM only:");
  for (size_t i = 0; i < 4; ++i) {
    DEBUG_SERIAL.print("ID"); DEBUG_SERIAL.print(CALIBRATIONS[i].id);
    DEBUG_SERIAL.print(" = "); DEBUG_SERIAL.print(directionReference[i]);
    DEBUG_SERIAL.print(" (canonical "); DEBUG_SERIAL.print(canonicalRaw(directionReference[i]));
    DEBUG_SERIAL.println(")");
  }
  DEBUG_SERIAL.println();
}

void compareDirectionReference() {
  if (!directionReferenceValid) {
    DEBUG_SERIAL.println("ERROR: PRESS S TO SAVE A DIRECTION REFERENCE FIRST.");
    return;
  }
  DEBUG_SERIAL.println("\n====================================");
  DEBUG_SERIAL.println("DIRECTION-TEST COMPARISON");
  DEBUG_SERIAL.println("====================================");
  int32_t current[4];
  int32_t change[4];
  size_t movedCount = 0;
  size_t movedIndex = 0;
  for (size_t i = 0; i < 4; ++i) {
    uint8_t id = CALIBRATIONS[i].id;
    if (!readRaw(id, current[i])) return;
    change[i] = shortestRawDelta(current[i], directionReference[i]);
    if (labs(change[i]) > DIRECTION_MOVED_THRESHOLD_RAW) {
      ++movedCount;
      movedIndex = i;
    }
    DEBUG_SERIAL.print("ID"); DEBUG_SERIAL.println(id);
    DEBUG_SERIAL.print("REFERENCE RAW = "); DEBUG_SERIAL.print(directionReference[i]);
    DEBUG_SERIAL.print(" (canonical "); DEBUG_SERIAL.print(canonicalRaw(directionReference[i])); DEBUG_SERIAL.println(")");
    DEBUG_SERIAL.print("CURRENT RAW   = "); DEBUG_SERIAL.print(current[i]);
    DEBUG_SERIAL.print(" (canonical "); DEBUG_SERIAL.print(canonicalRaw(current[i])); DEBUG_SERIAL.println(")");
    DEBUG_SERIAL.print("CHANGE RAW    = "); printSignedInt(change[i]); DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("CHANGE DEG    = "); printSignedFloat(deltaRawToDegrees(change[i])); DEBUG_SERIAL.println(" deg");
    DEBUG_SERIAL.print("RAW DIRECTION = ");
    if (change[i] > DIRECTION_MOVED_THRESHOLD_RAW) DEBUG_SERIAL.println("INCREASED");
    else if (change[i] < -DIRECTION_MOVED_THRESHOLD_RAW) DEBUG_SERIAL.println("DECREASED");
    else DEBUG_SERIAL.println("NO SIGNIFICANT CHANGE");
    DEBUG_SERIAL.println();
  }

  DEBUG_SERIAL.println("====================================");
  DEBUG_SERIAL.println("SIMPLE RESULT");
  DEBUG_SERIAL.println("====================================");
  if (movedCount == 0) {
    DEBUG_SERIAL.println("NO JOINT MOVED ENOUGH.");
    DEBUG_SERIAL.println("REDO WITH ONE SMALL 3 TO 10 DEGREE MOVE.");
  } else if (movedCount > 1) {
    DEBUG_SERIAL.print("INVALID TEST: "); DEBUG_SERIAL.print(movedCount);
    DEBUG_SERIAL.println(" JOINTS MOVED.");
    DEBUG_SERIAL.println("RETURN HOME, HOLD OTHER LINKS, PRESS S, AND REDO.");
  } else if (labs(change[movedIndex]) > DIRECTION_MAX_TEST_MOVE_RAW) {
    DEBUG_SERIAL.print("INVALID TEST: ID"); DEBUG_SERIAL.print(CALIBRATIONS[movedIndex].id);
    DEBUG_SERIAL.println(" MOVED MORE THAN 22.5 DEG.");
    DEBUG_SERIAL.println("RETURN HOME AND REDO WITH A SMALLER MOVE.");
  } else {
    DEBUG_SERIAL.print("VALID SINGLE-JOINT TEST: ID"); DEBUG_SERIAL.println(CALIBRATIONS[movedIndex].id);
    DEBUG_SERIAL.print("RAW ");
    DEBUG_SERIAL.println(change[movedIndex] > 0 ? "INCREASED" : "DECREASED");
    DEBUG_SERIAL.println("RECORD WHICH PHYSICAL DIRECTION YOU MOVED THAT JOINT.");
  }
  DEBUG_SERIAL.println("Do not assign mathematical sign until the physical direction is recorded.\n");
}

void printCalibrationSummary() {
  DEBUG_SERIAL.println("\n====================================");
  DEBUG_SERIAL.println("CALIBRATION TABLE - RAW SOURCE OF TRUTH");
  DEBUG_SERIAL.println("====================================");
  for (size_t i = 0; i < 4; ++i) {
    const MotorCalibration &cal = CALIBRATIONS[i];
    DEBUG_SERIAL.print("ID"); DEBUG_SERIAL.println(cal.id);
    DEBUG_SERIAL.print("MIN  = "); DEBUG_SERIAL.print(cal.min_raw);
    if (cal.min_is_provisional) DEBUG_SERIAL.print(" PROVISIONAL");
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("HOME = "); DEBUG_SERIAL.println(cal.home_raw);
    DEBUG_SERIAL.print("MAX  = "); DEBUG_SERIAL.println(cal.max_raw);
    DEBUG_SERIAL.println();
  }
  DEBUG_SERIAL.println("ID15");
  DEBUG_SERIAL.print("OPEN  = "); DEBUG_SERIAL.println(ID15_OPEN_RAW);
  DEBUG_SERIAL.print("HOME  = "); DEBUG_SERIAL.println(ID15_HOME_RAW);
  DEBUG_SERIAL.print("CLOSE = "); DEBUG_SERIAL.println(ID15_CLOSE_RAW);
  DEBUG_SERIAL.println("\nNo q2/q3/q4 signs or mathematical joint angles are defined.\n");
}

void printMenu() {
  DEBUG_SERIAL.println("\n====================================================");
  DEBUG_SERIAL.println("OPENMANIPULATOR-X NEW CALIBRATION VERIFICATION");
  DEBUG_SERIAL.println("====================================================\n");
  DEBUG_SERIAL.println("TEST ORDER:\n");
  DEBUG_SERIAL.println("1. HOME TEST");
  DEBUG_SERIAL.println("   Put robot at HOME and press H\n");
  DEBUG_SERIAL.println("2. LIMIT TEST");
  DEBUG_SERIAL.println("   Press X for torque OFF");
  DEBUG_SERIAL.println("   Move one joint manually");
  DEBUG_SERIAL.println("   Press 1/2/3/4/5 to read it\n");
  DEBUG_SERIAL.println("3. DIRECTION TEST");
  DEBUG_SERIAL.println("   Press X and physically support the robot");
  DEBUG_SERIAL.println("   Put ALL joints at HOME and press S");
  DEBUG_SERIAL.println("   Hold every other link still");
  DEBUG_SERIAL.println("   Move ONE joint only 3 to 10 degrees");
  DEBUG_SERIAL.println("   Press D once and read SIMPLE RESULT\n");
  DEBUG_SERIAL.println("COMMANDS:");
  DEBUG_SERIAL.println("H = Check HOME (read only)");
  DEBUG_SERIAL.println("P = Print all motors");
  DEBUG_SERIAL.println("1 = Read ID11");
  DEBUG_SERIAL.println("2 = Read ID12");
  DEBUG_SERIAL.println("3 = Read ID13");
  DEBUG_SERIAL.println("4 = Read ID14");
  DEBUG_SERIAL.println("5 = Read ID15");
  DEBUG_SERIAL.println("X = Torque OFF");
  DEBUG_SERIAL.println("B = Safe torque ON");
  DEBUG_SERIAL.println("S = Save direction-test reference");
  DEBUG_SERIAL.println("D = Compare direction-test reference");
  DEBUG_SERIAL.println("C = Print calibration summary");
  DEBUG_SERIAL.println("M = Print this menu again");
  DEBUG_SERIAL.println("====================================================\n");
}

void handleCommand(char command) {
  if (command >= 'a' && command <= 'z') command -= ('a' - 'A');
  switch (command) {
    case 'H': checkHome(); break;
    case 'P': printAllMotors(); break;
    case '1': printOneMotor(11); break;
    case '2': printOneMotor(12); break;
    case '3': printOneMotor(13); break;
    case '4': printOneMotor(14); break;
    case '5': printOneMotor(15); break;
    case 'X': torqueOffAll(); break;
    case 'B': safeTorqueOn(); break;
    case 'S': saveDirectionReference(); break;
    case 'D': compareDirectionReference(); break;
    case 'C': printCalibrationSummary(); break;
    case 'M': printMenu(); break;
    case '\r': case '\n': case ' ': case '\t': break;
    default:
      DEBUG_SERIAL.print("UNKNOWN COMMAND: "); DEBUG_SERIAL.println(command);
      DEBUG_SERIAL.println("Press M to print the menu.");
  }
}

void setup() {
  DEBUG_SERIAL.begin(USB_BAUD);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  delay(500);
  printMenu();
}

void loop() {
  while (DEBUG_SERIAL.available() > 0)
    handleCommand((char)DEBUG_SERIAL.read());
}
