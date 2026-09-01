import serial
import time
import msvcrt

PORT = "COM7"      # Change if OpenCR uses another COM port
BAUD = 115200

ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.1
)

# OpenCR may reset when serial connection opens
time.sleep(2)

print("GRIPPER TELEOP")
print("----------------")
print("O : open 5 degrees")
print("C : close 5 degrees")
print("A : fully open  (106°)")
print("Z : fully close (234°)")
print("S : show motor position")
print("X : torque off")
print("Q : quit")
print()

while True:

    key = msvcrt.getwch().upper()

    # Quit Python
    if key == "Q":
        break

    # Send valid commands to OpenCR
    if key in ["O", "C", "A", "Z", "S", "X"]:

        ser.write(key.encode())
        ser.flush()

        # Give OpenCR time to respond
        time.sleep(0.2)

        # Print all data returned by OpenCR
        while ser.in_waiting:

            line = ser.readline().decode(
                errors="ignore"
            ).strip()

            if line:
                print(line)

ser.close()