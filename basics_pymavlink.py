from pymavlink import mavutil
from pymavlink.dialects.v20 import common
import time

# Need Basic functionality built in 

class DroneDrive:
    def __init__(self,connection_string='udpin:localhost:14550',baud=115200):
        self.connection = mavutil.mavlink_connection(connection_string,baud)
        print("[DRONE] Waiting for Heartbeat...")
        self.connection.wait_heartbeat()
       
    def set_mode(self,mode_name):
        # Check if mode exists in the map
        if mode_name not in self.connection.mode_mapping():
            print(f"[DRONE] Unknown Mode: {mode_name}")
            return

        mode_id = self.connection.mode_mapping()[mode_name]
        self.connection.set_mode(mode_id)
        
        # Verify the mode actually changed (Robustness)
        ack = False
        while not ack:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                print(f"[DRONE] Mode switched to {mode_name}")
                ack = True

    def arm(self, timeout=30):
        """
        Attempts to arm. 
        - Prints 'STATUSTEXT' from drone (e.g. 'PreArm: Compass Variance')
        - Retries sending command every 1 second
        - Times out after 'timeout' seconds
        """
        print(f"[DRONE] Attempting to ARM (Timeout: {timeout}s)...")
        start_time = time.time()
        next_send_time = 0

        while time.time() - start_time < timeout:
            # 1. Send Arm Command (Once every second)
            if time.time() > next_send_time:
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
                next_send_time = time.time() + 1.0

            # 2. Check for Incoming Messages (Non-blocking)
            msg = self.connection.recv_match(type=['HEARTBEAT', 'STATUSTEXT', 'COMMAND_ACK'], blocking=True, timeout=0.1)
            
            if not msg:
                continue

            # CASE A: We are Armed! (Success)
            if msg.get_type() == 'HEARTBEAT':
                # Check the "Safety Switch" bit in base_mode
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("[DRONE] >>> ARMING SUCCESSFUL! <<<")
                    return True

            # CASE B: The Drone is complaining (Print the Error)
            elif msg.get_type() == 'STATUSTEXT':
                # This catches "PreArm: GPS HDOP too high" etc.
                print(f"[DRONE STATUS] {msg.text}")
                
            
            # CASE C: Explicit Rejection
            elif msg.get_type() == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"[DRONE] Arming Denied (Result Code: {msg.result})")
                        return False

        # 3. Timeout Reached
        print(f"[DRONE] FAILURE: Could not arm within {timeout} seconds.")
        return False

    def takeoff(self, target_altitude, timeout=30):
        """
        Robust Takeoff:
        1. Sends TAKEOFF command and ensures the drone accepted it (ACK).
        2. Monitors altitude in a loop until target is reached.
        3. Fails safely if timeout is exceeded.
        """
        print(f"[DRONE] Initiating Takeoff to {target_altitude}m (Timeout: {timeout}s)...")
        
        # --- STEP 1: COMMAND & ACKNOWLEDGMENT (The Handshake) ---
        # We need to make sure the drone actually HEARD the takeoff command.
        
        ack_received = False
        attempt_start = time.time()
        
        while not ack_received:
            # Check Timeout (Global for the whole function or just this step)
            if time.time() - attempt_start > 5: # Give 5 seconds to get an ACK
                print("[DRONE] ERROR: Takeoff command ignored (No ACK).")
                return False

            # Send Command
            self.connection.mav.command_long_send(
                self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, target_altitude)
            
            # Wait for ACK (Briefly block)
            msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
            
            if msg and msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("[DRONE] Takeoff Command Accepted.")
                    ack_received = True
                else:
                    print(f"[DRONE] Takeoff Denied! Result Code: {msg.result}")
                    return False
        
        # --- STEP 2: PHYSICAL VERIFICATION (The Feedback Loop) ---
        # The command was accepted, but is the drone actually moving?
        # Replaces 'time.sleep(5)' with active monitoring.
        
        print("[DRONE] Climbing...")
        start_climb = time.time()
        
        while True:
            # Global Timeout Check
            if time.time() - start_climb > timeout:
                print(f"[DRONE] ERROR: Takeoff Timed Out (Did not reach {target_altitude}m).")
                return False

            # Get Altitude Data
            # GLOBAL_POSITION_INT gives relative_alt in millimeters
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            
            if msg:
                current_alt = msg.relative_alt / 1000.0 # Convert mm to meters
                
                # Check if we are close enough (95% of target)
                if current_alt >= target_altitude * 0.95:
                    print(f"[DRONE] Target Altitude Reached ({current_alt:.2f}m).")
                    return True
    
    def send_velocity(self, vx, vy, vz):
        # Sends velocity in BODY FRAME (Forward, Right, Down)
        type_mask = (
            common.POSITION_TARGET_TYPEMASK_X_IGNORE |
            common.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            common.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            # common.POSITION_TARGET_TYPEMASK_VX_IGNORE | # Keep this OFF (0) to enable control
            # common.POSITION_TARGET_TYPEMASK_VY_IGNORE | # Keep this OFF (0) to enable control
            # common.POSITION_TARGET_TYPEMASK_VZ_IGNORE | # Keep this OFF (0) to enable control
            common.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            common.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            common.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            common.POSITION_TARGET_TYPEMASK_FORCE_SET | # Ignore Force
            common.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            common.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.connection.mav.set_position_target_local_ned_send(
            0, self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
            type_mask, 
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0, 0, 0
        )