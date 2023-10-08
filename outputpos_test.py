from os import stat
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import sys, time
from pymavlink.mavextra import mode
from serial.serialutil import SerialException
import math
import os

#Conexion Serial con el autopiloto
ap_uav = mavutil.mavlink_connection('/dev/ttyACM0',baud=57600) # Cambiar a 115200, revisar puerto en dmesg
ap_uav.wait_heartbeat()
print("Heartbeat from ap_uav (system %u component %u)" % (ap_uav.target_system, ap_uav.target_component))

while True:
    try:
        print(ap_uav.recv_match().to_dict())
    except:
        pass
    time.sleep(0.1)