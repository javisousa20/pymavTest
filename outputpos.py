# =========================== OUTPUT POSITION ==========================
# Envío de trama MAVLINK GNSS para navegación y comprobación de posición del UAV
#   - Satisface requisito BVLL03-LLR073 del proyecto GEOSUB
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
gpslat=int(37.7749*1e7)
gpslon=int(-122.4194*1e7)
gpsalt=-80
hdop=0

#Conexion Serial con el autopiloto
ap_uav = mavutil.mavlink_connection(ifacecomm,baud=56700)
ap_uav.wait_heartbeat()
print("Heartbeat from ap_uav (system %u component %u)" % (ap_uav.target_system, ap_uav.target_component))

# Envía la posición GPS deseada al dispositivo
gpsmsg = ap_uav.mav.gps_input_send(
    0,  # Timestamp (micros since boot or Unix epoch)
    0,  # ID of the GPS for multiple GPS inputs
    # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
    # All other fields must be provided.
    8 | 16 | 32,
    100,  # GPS time (milliseconds from start of GPS week)
    0,  # GPS week number
    3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
    gpslat,  # Latitude (WGS84), in degrees * 1E7
    gpslon,  # Longitude (WGS84), in degrees * 1E7
    gpsalt,  # Altitude (AMSL, not WGS84), in m (positive for up)
    1,  # GPS HDOP horizontal dilution of position in m
    1,  # GPS VDOP vertical dilution of position in m
    0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
    0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
    0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
    0,  # GPS speed accuracy in m/s
    0,  # GPS horizontal accuracy in m
    0,  # GPS vertical accuracy in m
    7   # Number of satellites visible.
)
try:
    #Llegada de mensajes
    pos = ap_uav.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
    if not pos:
            print("no msg")
    if pos.get_type() == "BAD_DATA":
        if mavutil.all_printable(pos.data):
            sys.stdout.write(pos.data)
            sys.stdout.flush()
    elif pos.get_type() == "GLOBAL_POSITION_INT":
        #Guardar atributos
        print(pos)
        time_boot_ms=0 #no se usa
        coordinate_frame=5 #el tipo que corresponde a las coordenadas recibidas por el ap_car
        type_mask=0b111111000000 #bitmap para indicar los parámetros que enviaremos nulos
        #type_mask=0b11011111100
        lat_int=pos.lat
        lon_int=pos.lon
        alt=-60
        #alt=msg.alt*0.001 + float(sys.argv[1]) #altura del ap_car(mm) + la entrada(en metros)
        # vx=msg.vx*0.01 #de cm/s a m/s
        # vy=msg.vx*0.01 #de cm/s a m/s
        # vz=msg.vx*0.01 #de cm/s a m/s
        vx=0
        vy=0
        vz=0
        #vz=0
        #yaw=msg.hdg*(math.pi/180) #de grados a radianes
        yaw=0

    else:
        pass
except SerialException:
    print("Comprobe the AP conected to the Ground PC")

ap_uav.close()
