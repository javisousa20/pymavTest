# ======================= INPUT TELEMTRY PETITION ======================
# Envío de petición MAVLINK de telemetría y recepción de datos
#   - Satisface requisito BVLL03-LLR075 del proyecto GEOSUB
#   jsousa@catec.aero
# ======================================================================

from os import stat
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import sys, time
from pymavlink.mavextra import mode
from serial.serialutil import SerialException
import math
import os

ifacecomm='/dev/ttyACM0'

#Conexion Serial con el autopiloto
ap_uav = mavutil.mavlink_connection('/dev/ttyACM0',baud=57600) # Cambiar a 115200, revisar puerto en dmesg
while(ap_uav.target_system == 0):
    print("-- Checking Heartbeat")
    ap_uav.wait_heartbeat()
    print("Heartbeat from uav (system %u component %u)" % (ap_uav.target_system, ap_uav.target_component))

while True:
    try:
        ap_uav.mav.request_data_stream_send(ap_uav.target_system, ap_uav.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
        print(ap_uav.recv_match())
    except:
        break
        print("Comprobe the AP conected to the Ground PC")
    time.sleep(1)
