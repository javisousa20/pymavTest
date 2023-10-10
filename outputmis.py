# =========================== OUTPUT MISSION ===========================
# Envío de trama MAVLINK MISSION Waypoint y comprobación de llegada al UAV
#   - Satisface requisito BVLL03-LLR074 del proyecto GEOSUB
#   jsousa@catec.aero
# ======================================================================

import argparse
import math
from pymavlink import mavutil

ifacecomm='/dev/ttyACM0'

class mission_item:
   def __init__(self, i, current, x,y,z):
      
      self.seq = 1
      self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
      self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
      self.current = current
      self.auto = 1
      self.param1 = 1
      self.param2 = 0.0
      self.param3 = 2.00
      self.param4 = math.nan
      self.param5 = x
      self.param6 = y
      self.param7 = z
      self.mission_type = 0

def arm(the_connection):
   print("-- Arming")

   the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
   ack(the_connection, "COMMAND ACK")

def takeoff(the_connection):
   print("-- Taking off")

   the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, math.nan, 0, 0, 10)
   ack(the_connection, "COMMAND ACK")

def upload_mission(the_connection, mission_items):
   n=len(mission_items)
   print("-- Sending message out")

   the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component,n,0)

   ack(the_connection, "MISSION_REQUEST")

   for waypoint in mission_items:
      print("-- Creating waypoint")

      the_connection.mav.mission_item_send(the_connection.target_system,
                                           the_connection.target_component,
                                           waypoint.seq,
                                           waypoint.frame,
                                           waypoint.command,
                                           waypoint.current,
                                           waypoint.auto,
                                           waypoint.param1,
                                           waypoint.param2,
                                           waypoint.param3,
                                           waypoint.param4,
                                           waypoint.param5,
                                           waypoint.param6,
                                           waypoint.param7,
                                           waypoint.mission_type)
   if waypoint != mission_items[n-1]:
         ack(the_connection, "MISSION_REQUEST")
   ack(the_connection, "MISSION_ACK")

def set_return(the_connection):
   print("-- Set Return To Launch")

   the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
   ack(the_connection, "COMMAND ACK")

def start_mission(the_connection):
   print("-- Mission Start")

   the_connection.mav.command_long_send(the_connection.target_system,the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
   ack(the_connection, "COMMAND ACK")

def ack(the_connection, keyword):
   print("-- Message Read" + str(the_connection.recv_match(type=keyword, blocking = True)))

if __name__=="__main__":
   print("-- Program Started")
   the_connection = mavutil.mavlink_connection(ifacecomm,57600)

   while(the_connection.target_system == 0):
      print("-- Checking Heartbeat")
      the_connection.wait_heartbeat()
      print("Heartbeat from uav (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

   mission_waypoints = []

   mission_waypoints.append(mission_item(0, 0, 42.121212121212, -122.232323232323, 10))
   mission_waypoints.append(mission_item(1, 0, 42.232323232323, -122.454545454545, 10))
   mission_waypoints.append(mission_item(2, 0, 42.787878787878, -122.454545454545, 5))

   upload_mission(the_connection, mission_waypoints)

   # arm(the_connection)

   # takeoff(the_connection)
   
   # start_mission(the_connection)

   # for mission_item in mission_waypoints:
   #    print("-- Message Read " + str(the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking = True)))

   # set_return(the_connection)


