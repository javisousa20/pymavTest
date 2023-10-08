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

#Conexion Serial con el autopiloto
ap_uav = mavutil.mavlink_connection('/dev/ttyACM0',baud=57600) # Cambiar a 115200, revisar puerto en dmesg
ap_uav.wait_heartbeat()
print("Heartbeat from ap_uav (system %u component %u)" % (ap_uav.target_system, ap_uav.target_component))

# Define la posición GPS deseada (latitud, longitud y altitud)
desired_latitude = 37.7749  # Cambia esto a la latitud deseada
desired_longitude = -122.4194  # Cambia esto a la longitud deseada
desired_altitude = 100  # Cambia esto a la altitud deseada en metros

timeout = 2

# Envía la posición GPS deseada al dispositivo
msg = ap_uav.mav.set_position_target_global_int_encode(
    0,  # Time boot_ms
    ap_uav.target_system,  # Target system
    ap_uav.target_component,  # Target component
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # Frame
    0b0000111111111000,  # Type bitmask
    int(desired_latitude * 1e7),  # Latitud en grados * 1e7
    int(desired_longitude * 1e7),  # Longitud en grados * 1e7
    desired_altitude,  # Altitud en metros
    0, 0, 0,  # Velocidad en m/s (no se usa) vx, vy, vz
    0, 0, 0,  # Aceleración (no se usa) afx, afy, afz
    0, 0)  # Velocidad angular (no se usa) yaw, yaw_rate

ap_uav.mav.send(msg)
print(msg)

message = ap_uav.mav.command_long_encode(
        ap_uav.target_system,  # Target system ID
        ap_uav.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,  # param1: Message ID to be streamed
        1000000, # param2: Interval in microseconds
        0,       # param3 (unused)
        0,       # param4 (unused)
        0,       # param5 (unused)
        0,       # param5 (unused)
        0        # param6 (unused)
        )

# Send the COMMAND_LONG     
ap_uav.mav.send(message)

# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
response = ap_uav.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")
