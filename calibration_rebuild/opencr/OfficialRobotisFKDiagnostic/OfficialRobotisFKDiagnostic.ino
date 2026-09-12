#include <open_manipulator_libs.h>
#include <DynamixelWorkbench.h>

// Read-only diagnostic for OpenMANIPULATOR-X on OpenCR.
// PC USB Serial: 115200 baud. DYNAMIXEL bus: 1,000,000 baud.
// This sketch never enables/disables torque, changes operating mode, writes a
// goal position, or starts a trajectory.

#if defined(__OPENCR__)
  #define DXL_DEVICE_NAME ""
#else
  #error "This diagnostic sketch is intended for the ROBOTIS OpenCR board."
#endif

constexpr uint32_t PC_BAUD = 115200;
constexpr uint32_t DXL_BAUD = 1000000;
constexpr uint8_t MOTOR_COUNT = 5;
const uint8_t MOTOR_IDS[MOTOR_COUNT] = {11, 12, 13, 14, 15};
const char *JOINT_NAMES[4] = {"joint1", "joint2", "joint3", "joint4"};

// PROVISIONAL calibration trial. These are the previously captured home RAW
// readings. For this first comparison, that pose is treated as q1..q4 = 0.
// The signs come from the physical direction observations and can be changed
// after comparing several measured poses.
const int32_t CAL_HOME_RAW[4] = {1917, 2046, 4049, 943};
const float CAL_DIRECTION[4] = {+1.0f, +1.0f, -1.0f, -1.0f};

OpenManipulator open_manipulator;
DynamixelWorkbench dxl_wb;

void printDivider()
{
  Serial.println("============================================================");
}

void printMenu()
{
  Serial.println();
  Serial.println("P = read motors and print official ROBOTIS FK");
  Serial.println("M = print this menu");
  Serial.println("This program does not command motion or change torque.");
  Serial.print("> ");
}

// Return the encoder displacement nearest to the saved home reference.
// The original Present_Position is never changed or wrapped for display.
// OpenMANIPULATOR-X joints cannot physically rotate more than half a turn away
// from their calibrated home, so equivalent values separated by 4096 counts
// must use the nearest representation (for example, -49 is 4047 near 4049).
int32_t nearestHomeDelta(int32_t raw, int32_t home_raw)
{
  int32_t delta = raw - home_raw;
  while (delta > 2048)
  {
    delta -= 4096;
  }
  while (delta < -2048)
  {
    delta += 4096;
  }
  return delta;
}

bool readPresentPositions(int32_t raw[MOTOR_COUNT])
{
  bool all_ok = true;

  for (uint8_t index = 0; index < MOTOR_COUNT; index++)
  {
    const char *log = nullptr;
    if (!dxl_wb.itemRead(MOTOR_IDS[index], "Present_Position", &raw[index], &log))
    {
      Serial.print("ERROR: Could not read Present_Position from ID");
      Serial.print(MOTOR_IDS[index]);
      if (log != nullptr)
      {
        Serial.print(" - ");
        Serial.print(log);
      }
      Serial.println();
      all_ok = false;
    }
  }

  return all_ok;
}

void printOfficialFK()
{
  int32_t raw[MOTOR_COUNT] = {0, 0, 0, 0, 0};
  if (!readPresentPositions(raw))
  {
    Serial.println("FK was not calculated because one or more motor reads failed.");
    return;
  }

  float motor_rad[MOTOR_COUNT];
  float encoder_deg[MOTOR_COUNT];
  for (uint8_t index = 0; index < MOTOR_COUNT; index++)
  {
    // Official DynamixelWorkbench conversion. Raw values are not wrapped.
    motor_rad[index] = dxl_wb.convertValue2Radian(MOTOR_IDS[index], raw[index]);
    // Absolute encoder-count convention used in the measured-pose notes.
    // Deliberately no modulo: raw 4357 prints as 382.939 degrees.
    encoder_deg[index] = (float)raw[index] * 360.0f / 4096.0f;
  }

  // Put the current real arm positions into the official ROBOTIS model.
  for (uint8_t index = 0; index < 4; index++)
  {
    robotis_manipulator::JointValue value = {};
    value.position = motor_rad[index];
    open_manipulator.getManipulator()->setJointValue(JOINT_NAMES[index], value);
  }

  // This calls the official SolverCustomizedforOMChain created by
  // initOpenManipulator(false). No handwritten FK is used here.
  open_manipulator.solveForwardKinematics();

  const robotis_manipulator::KinematicPose tcp =
      open_manipulator.getKinematicPose("gripper");
  const Eigen::Vector3d rpy =
      robotis_manipulator::math::convertRotationMatrixToRPYVector(tcp.orientation);

  // Second pass through the SAME official ROBOTIS FK solver, but with our
  // provisional zero offsets and direction signs applied to the measured RAW.
  float calibrated_q[4];
  int32_t calibrated_delta_raw[4];
  for (uint8_t index = 0; index < 4; index++)
  {
    calibrated_delta_raw[index] = nearestHomeDelta(raw[index], CAL_HOME_RAW[index]);
    calibrated_q[index] = CAL_DIRECTION[index]
                        * (float)calibrated_delta_raw[index]
                        * (2.0f * PI / 4096.0f);

    robotis_manipulator::JointValue value = {};
    value.position = calibrated_q[index];
    open_manipulator.getManipulator()->setJointValue(JOINT_NAMES[index], value);
  }
  open_manipulator.solveForwardKinematics();

  const robotis_manipulator::KinematicPose calibrated_tcp =
      open_manipulator.getKinematicPose("gripper");
  const Eigen::Vector3d calibrated_rpy =
      robotis_manipulator::math::convertRotationMatrixToRPYVector(
          calibrated_tcp.orientation);

  Serial.println();
  printDivider();
  Serial.println("RAW / ENCODER (continuous Present_Position; no modulo)");
  printDivider();
  for (uint8_t index = 0; index < MOTOR_COUNT; index++)
  {
    Serial.print("ID");
    Serial.print(MOTOR_IDS[index]);
    Serial.print(" RAW = ");
    Serial.print(raw[index]);
    Serial.print("   motor angle = ");
    Serial.print(encoder_deg[index], 3);
    Serial.println(" deg");
  }

  Serial.println();
  printDivider();
  Serial.println("ROBOTIS JOINT VALUES");
  printDivider();
  for (uint8_t index = 0; index < 4; index++)
  {
    // motor_rad is the official joint value captured before the calibrated
    // comparison pass changed the in-memory model state.
    const float q = motor_rad[index];
    Serial.print("q");
    Serial.print(index + 1);
    Serial.print(" = ");
    Serial.print(q, 6);
    Serial.print(" rad = ");
    Serial.print(q * RAD_TO_DEG, 3);
    Serial.println(" deg");
  }

  Serial.println();
  printDivider();
  Serial.println("OFFICIAL ROBOTIS FORWARD KINEMATICS");
  printDivider();
  Serial.print("X = ");
  Serial.print(tcp.position(0), 6);
  Serial.print(" m = ");
  Serial.print(tcp.position(0) * 1000.0, 3);
  Serial.println(" mm");
  Serial.print("Y = ");
  Serial.print(tcp.position(1), 6);
  Serial.print(" m = ");
  Serial.print(tcp.position(1) * 1000.0, 3);
  Serial.println(" mm");
  Serial.print("Z = ");
  Serial.print(tcp.position(2), 6);
  Serial.print(" m = ");
  Serial.print(tcp.position(2) * 1000.0, 3);
  Serial.println(" mm");

  Serial.println();
  Serial.println("ORIENTATION (official rotation matrix converted to RPY)");
  Serial.print("Roll  = ");
  Serial.print(rpy(0), 6);
  Serial.print(" rad = ");
  Serial.print(rpy(0) * RAD_TO_DEG, 3);
  Serial.println(" deg");
  Serial.print("Pitch = ");
  Serial.print(rpy(1), 6);
  Serial.print(" rad = ");
  Serial.print(rpy(1) * RAD_TO_DEG, 3);
  Serial.println(" deg");
  Serial.print("Yaw   = ");
  Serial.print(rpy(2), 6);
  Serial.print(" rad = ");
  Serial.print(rpy(2) * RAD_TO_DEG, 3);
  Serial.println(" deg");

  Serial.println();
  printDivider();
  Serial.println("PROVISIONAL CALIBRATED JOINT VALUES");
  Serial.println("Home RAW is temporarily treated as q1=q2=q3=q4=0");
  Serial.println("Candidate signs: q1 +, q2 +, q3 -, q4 -");
  printDivider();
  for (uint8_t index = 0; index < 4; index++)
  {
    Serial.print("q");
    Serial.print(index + 1);
    Serial.print(" calibrated = ");
    Serial.print(calibrated_q[index], 6);
    Serial.print(" rad = ");
    Serial.print(calibrated_q[index] * RAD_TO_DEG, 3);
    Serial.print(" deg   (home RAW ");
    Serial.print(CAL_HOME_RAW[index]);
    Serial.print(", nearest delta ");
    Serial.print(calibrated_delta_raw[index]);
    Serial.println(")");
  }

  Serial.println();
  printDivider();
  Serial.println("PROVISIONAL CALIBRATED FK (official ROBOTIS solver)");
  printDivider();
  Serial.print("X = ");
  Serial.print(calibrated_tcp.position(0), 6);
  Serial.print(" m = ");
  Serial.print(calibrated_tcp.position(0) * 1000.0, 3);
  Serial.println(" mm");
  Serial.print("Y = ");
  Serial.print(calibrated_tcp.position(1), 6);
  Serial.print(" m = ");
  Serial.print(calibrated_tcp.position(1) * 1000.0, 3);
  Serial.println(" mm");
  Serial.print("Z = ");
  Serial.print(calibrated_tcp.position(2), 6);
  Serial.print(" m = ");
  Serial.print(calibrated_tcp.position(2) * 1000.0, 3);
  Serial.println(" mm");
  Serial.print("Roll  = ");
  Serial.print(calibrated_rpy(0), 6);
  Serial.print(" rad = ");
  Serial.print(calibrated_rpy(0) * RAD_TO_DEG, 3);
  Serial.println(" deg");
  Serial.print("Pitch = ");
  Serial.print(calibrated_rpy(1), 6);
  Serial.print(" rad = ");
  Serial.print(calibrated_rpy(1) * RAD_TO_DEG, 3);
  Serial.println(" deg");
  Serial.print("Yaw   = ");
  Serial.print(calibrated_rpy(2), 6);
  Serial.print(" rad = ");
  Serial.print(calibrated_rpy(2) * RAD_TO_DEG, 3);
  Serial.println(" deg");

  Serial.println();
  Serial.println("GRIPPER (not used to calculate arm TCP position)");
  Serial.print("ID15 RAW = ");
  Serial.println(raw[4]);
  Serial.print("ID15 official actuator conversion = ");
  Serial.print(motor_rad[4], 6);
  Serial.print(" rad = ");
  Serial.print(motor_rad[4] * RAD_TO_DEG, 3);
  Serial.println(" deg");
  printDivider();
}

void setup()
{
  Serial.begin(PC_BAUD);
  delay(1000);

  Serial.println();
  printDivider();
  Serial.println("OpenMANIPULATOR-X official FK read-only diagnostic");
  printDivider();

  // false builds the official robot model and SolverCustomizedforOMChain,
  // but deliberately skips the OpenManipulator actuator initialization path.
  // That avoids its mode/profile-register writes and automatic torque enable.
  open_manipulator.initOpenManipulator(false);

  const char *log = nullptr;
  if (!dxl_wb.init(DXL_DEVICE_NAME, DXL_BAUD, &log))
  {
    Serial.print("FATAL: DYNAMIXEL bus initialization failed");
    if (log != nullptr)
    {
      Serial.print(" - ");
      Serial.print(log);
    }
    Serial.println();
    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("DYNAMIXEL bus opened at 1000000 baud.");
  Serial.println("Checking IDs 11, 12, 13, 14, 15...");
  for (uint8_t index = 0; index < MOTOR_COUNT; index++)
  {
    uint16_t model_number = 0;
    log = nullptr;
    Serial.print("ID");
    Serial.print(MOTOR_IDS[index]);
    if (dxl_wb.ping(MOTOR_IDS[index], &model_number, &log))
    {
      Serial.print(" OK, model ");
      Serial.println(model_number);
    }
    else
    {
      Serial.print(" NOT FOUND");
      if (log != nullptr)
      {
        Serial.print(" - ");
        Serial.print(log);
      }
      Serial.println();
    }
  }

  printMenu();
}

void loop()
{
  if (Serial.available() <= 0)
  {
    return;
  }

  const char command = (char)Serial.read();
  if (command == '\r' || command == '\n' || command == ' ' || command == '\t')
  {
    return;
  }

  if (command == 'P' || command == 'p')
  {
    printOfficialFK();
  }
  else if (command == 'M' || command == 'm' || command == '?')
  {
    printMenu();
    return;
  }
  else
  {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }

  Serial.print("> ");
}
