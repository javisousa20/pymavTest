# =========================== OUTPUT POSITION ==========================
# Envío de trama MAVLINK GNSS para navegación y comprobación de posición del UAV
#   - Satisface requisito BVLL03-LLR073 del proyecto GEOSUB
#   jsousa@catec.aero
# ======================================================================

from pymavlink import mavutil
import time, sys

ifacecomm='/dev/ttyACM0'

# Start a connection listening on a UDP port
connection = mavutil.mavlink_connection('/dev/ttyACM0',baud=57600) # Cambiar a 115200, revisar puerto en dmesg
while(connection.target_system == 0):
   print("-- Checking Heartbeat")
   connection.wait_heartbeat()
   print("Heartbeat from uav (system %u component %u)" % (connection.target_system, connection.target_component))

connection.mav.request_data_stream_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)
        
# Send the COMMAND_LONG     
# connection.mav.send(message)
n=0
while n<20:
   time.sleep(0.5)
   try:
      n=n+1
      msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
      msg2 = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
      print(msg)
   except Exception as error:
      print(error)
      connection.close()
      sys.exit(0)