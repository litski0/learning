from pymavlink import mavutil
from pymavlink.dialects.v20 import common
import time
import threading
import cv2
import math
import argparse

from Tut_redCircle import RedCircleDetector


# --- CONFIGURATION ---
CONNECTION_STRING = 'tcp:127.0.0.1:5760'
BAUD_RATE = 57600
CAMERA_FOV_X = 60.0  # Degrees
CAMERA_FOV_Y = 45.0  # Degrees
RES_X, RES_Y = 640, 480 # Standard Webcams

class SharedState:
    """Thread-safe data storage"""
    def __init__(self):
        self.lock = threading.Lock()
        self.found = False
        self.x = 0
        self.y = 0
        self.radius = 0

    def update(self, found, x, y, r):
        with self.lock:
            self.found = found
            self.x = x
            self.y = y
            self.radius = r

    def get(self):
        with self.lock:
            return self.found, self.x, self.y, self.radius

def mavlink_loop(stop_event, shared_state):
    pass

def mavlink_own():
    AP_GUIDED=4

    the_connection = mavutil.mavlink_connection('udpin:localhost:14550') #need to check with port

    the_connection.wait_heartbeat()
    print(f"Heartbeat from system (system :{the_connection.target_system} component :{the_connection.target_component})")

    the_connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
        0, 0, 0
    )
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        common.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        1,
        100000,
        0,0,0,0,0
    )
    while True:
        msg= the_connection.recv_match(type="SYS_STATUS",blocking=True)
        sensors_health=msg.onboard_control_sensors_health
        is_gyro_healthy = sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO
        is_gps_healthy = sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS

        if(is_gps_healthy and is_gyro_healthy ):
            print("Pre Arm Good")
            break
    # next steps
    # TODO: mode guided

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        common.MAV_CMD_DO_SET_MODE,
        0,
        1,
        AP_GUIDED,
        0,0,0,0,0
    )
        # check for pre-arm good or need positon estiamte type of shit 
    # TODO: arm throttle
    arm_result=-1
    print("Waiting for Arm_Result")
    while arm_result != common.MAV_RESULT_ACCEPTED :
        print(f"Arm_result:{arm_result}")
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            common.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            common.MAV_BOOL_TRUE,
            0,0,0,0,0,0
        )
        arm_msg = the_connection.recv_match(type="COMMAND_ACK",blocking=True)
        print(arm_msg)
        if(arm_msg and arm_msg.command == common.MAV_CMD_COMPONENT_ARM_DISARM):
            arm_result=arm_msg.result
    print("Armed Successfully")
    time.sleep(2)
        

    takeoff_result=-1
    x=10

    while takeoff_result!=common.MAV_RESULT_ACCEPTED:
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            common.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,x
        )

        takeoff_msg=the_connection.recv_match(type="COMMAND_ACK",blocking=True)
        
        if(takeoff_msg and takeoff_msg.command==common.MAV_CMD_NAV_TAKEOFF):
            takeoff_result=takeoff_msg.result

    # trying to make a square
    def wait_for_altitude(target_alt, tolerance=0.5):
        print(f"Waiting for altitude: {target_alt}m")
        while True:
            # Check Global Position (faster update rate usually)
            msg = the_connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            
            # relative_alt is in millimeters, so divide by 1000
            current_alt = msg.relative_alt / 1000.0
            
            print(f"Current Alt: {current_alt:.2f}m")
            
            if current_alt >= (target_alt - tolerance):
                print("Target Altitude Reached!")
                break
            
            time.sleep(0.1) # Don't spam the CPU
    wait_for_altitude(10)
    length=100 
    def ForwardandYaw(dist,angle):
        # send forward command
        the_connection.mav.set_position_target_local_ned_send(
            0,
            the_connection.target_system,
            the_connection.target_component,
            common.MAV_FRAME_BODY_FRD, 
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
            dist,0,0,
            0,0,0,
            0,0,0,
            0,0
        )
        time.sleep(1)   
        the_connection.mav.command_long_send(
            the_connection.target_system,
            the_connection.target_component,
            common.MAV_CMD_CONDITION_YAW,0,
            angle,0,
            1,
            1,
            0,0,0
            )
        time.sleep(dist/10)

    for i in range (4):
        ForwardandYaw(length,90)
