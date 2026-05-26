import sys
sys.path.insert(0, '/home/nilamari/zobot_ws/ros/sowmiya_ws/FTServo_Python')
import math
import time
from scservo_sdk import *

DEVICENAME = "/dev/ttyACM1"
BAUDRATE = 1000000

port = PortHandler(DEVICENAME)
packet = sms_sts(port)

port.openPort()
port.setBaudRate(BAUDRATE)

for motor_id in range(1, 7):
    try:
        model, comm_result, error = packet.ping(motor_id)
        print(
            f"ID={motor_id} "
            f"model={model} "
            f"comm={comm_result} "
            f"err={error}"
        )
    except Exception as e:
        print(f"ID={motor_id}: {e}")