# =========================== OUTPUT MISSION ===========================
# Envío de trama MAVLINK MISSION Waypoint y comprobación de llegada al UAV
#   - Satisface requisito BVLL03-LLR074 del proyecto GEOSUB
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
ap_uav = mavutil.mavlink_connection('/dev/tty0',baud=57600) # Cambiar a 115200, revisar puerto en dmesg
ap_uav.wait_heartbeat()
print("Heartbeat from ap_uav (system %u component %u)" % (ap_uav.target_system, ap_uav.target_component))

waypoints = []

# Waypoint 1: Latitud, longitud y altitud
waypoint1 = (37.7749, -122.4194, 100)
waypoints.append(waypoint1)

# Waypoint 2: Latitud, longitud y altitud
waypoint2 = (37.7750, -122.4195, 150)
waypoints.append(waypoint2)

# Itera a través de los waypoints y envíalos al autopiloto
seq = 0
for lat, lon, alt in waypoints:
    # Crea un mensaje MAVLink de tipo MISSION_ITEM
    msg = ap_uav.mav.mission_item_encode(
        ap_uav.target_system,  # ID del sistema objetivo
        ap_uav.target_component,  # ID del componente objetivo
        seq,  # Secuencia del waypoint
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Marco de referencia (global con altitud relativa)
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Comando de navegación para waypoints
        0,  # Máscara de parámetros (ignorar todos los demás parámetros)
        0, 0,  # Parametros de la misión personalizada
        int(lat*1e7), int(lon*1e7), int(alt),  # Latitud, longitud y altitud del waypoint
        0, 0, 0)  # Más parámetros de la misión personalizada

    # Envía el mensaje MAVLink al autopiloto
    ap_uav.mav.send(msg)

    # Incrementa la secuencia
    seq += 1

# Espera una respuesta ACK del autopiloto
while True:
    time.sleep(1)
    ack_msg = ap_uav.recv_match(type='MISSION_ACK', blocking=True)
    if ack_msg:
        if ack_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("Waypoints aceptados por el autopiloto")
        else:
            print("El autopiloto rechazó los waypoints")
        break

#while True:
#    try:
#        print(ap_uav.recv_match().to_dict())
#    except:
#        pass
#    time.sleep(0.1)
