import sys
import json
import re
import time
import threading

import numpy as np

from dobot_api.dashboard import DobotDashboard
from dobot_api.feedback import DobotApiFeedBack

class DobotController:
    def __init__(
        self, 
        dashboard_ip="127.0.0.1", dashboard_port=29999, 
        feedback_ip="127.0.0.1", feedback_port=30004
        ):
        self.client_dash = None
        self.client_feed = None
        self.robot_state = {}

        self.home_joint = [90.0, 0.0, 130.0, -40.0, -90.0, 0.0]
        self.home_position = [-647.06, 131.33, -303.88, 132.32, 0.32, 0.30]

        class item:
            def __init__(self):
                self.robotMode = -1     # Robot mode
                self.MessageSize = -1
                self.Qactual = [-1]*6
                self.DigitalInputs =-1
                self.DigitalOutputs = -1
                self.robotCurrentCommandID = -1

        self.feedData = item()  # Define the data structure object

        self.__globalLockValue = threading.Lock()

        try:
            self.client_dash = DobotDashboard(dashboard_ip, dashboard_port)
            self.client_feed = DobotApiFeedBack(feedback_ip, feedback_port)
        except Exception as e:
            print(f"Failed to initialize DobotController: {e}")

    ##########################
    # Main Control Functions #
    ##########################
    def start(self):
        if not self.client_dash or not self.client_feed:
            print("Clients are not properly initialized.")
            return

        result = self.enable()
        print(f"EnableRobot result: {result}")
        if result is None:
            print("EnableRobot failed due to uninitialized client.")
            return
        if result[0] != 0:
            print("Enable failed: check whether port 29999 is in use")
            return
        print("Enable succeeded")
        
        self.robot_state['ready'] = True

        self.feed_thread = threading.Thread(target=self.get_feedback)
        self.feed_thread.daemon = True
        self.feed_thread.start()

    def shutdown(self):
        if self.client_dash and self.robot_state.get('enable', False):
            self.disable()
        self.robot_state['ready'] = False

    ## **** 
    def move_trajectory_smooth(self, trajectory_points: np.ndarray, control_frequency: float = 80.0, threshold: float = 0.25, speed_scale: float = 1.0):
        """
        Executes a trajectory by strictly following the path defined by the points.
        
        Parameters:
        - trajectory_points: np.ndarray of shape (N, 7) where [:6] is joint deltas and [6] is IO state.
        - control_frequency: Frequency (Hz) at which to update the robot commands.
        - threshold: Distance threshold to consider the final position reached.
        - speed_scale: Scaling factor for maximum speed (0.0 to 1.0).
        """
        if not self.client_dash or not self.robot_state.get('enable', False):
            print("Cannot move trajectory: client not initialized or robot not enabled.")
            return

        delay_time = 1.0 / control_frequency

        # Max Speed: 1.0 deg/loop @ 80Hz = 80 deg/s
        MAX_VELOCITY = 1.0 * speed_scale
        # Acceleration: Lower = Smoother, Higher = More precise corners
        ACCEL = 0.05 

        # We freeze the entire path geometry now so it cannot drift later.
        # This prevents noise accumulation over long trajectories.
        current_real_joints = np.array(self.feedData.Qactual)

        # This list will hold [J1, J2, J3, J4, J5, J6, IO_State] for every waypoint as 2d array
        absolute_waypoints = []

        # Create absolute waypoints from relative deltas
        temp_joints = np.copy(current_real_joints)
        for point in trajectory_points:
            
            if len(point) < 6: continue

            # Relative to Absolute conversion
            temp_joints = temp_joints + point[:6]
            # Saving IO state if provided            
            io = int(point[6]) if len(point) > 6 else -1
            
            # Store the absolute trajectory
            absolute_waypoints.append((np.copy(temp_joints), io))
        
        del temp_joints  # Free memory

        total_points = len(absolute_waypoints)
        print(f"Path Locked: {total_points} waypoints.")

        # The 'virtual_joints' starts at the robot's current position
        virtual_joints = np.copy(current_real_joints)
        current_velocity = 0.0
        last_io_state = -1

        # Iterate through the fixed absolute waypoints
        for i, (target_joints, target_io) in enumerate(absolute_waypoints):

            is_last_point = (i == total_points - 1)

            # Calculate full distance of this specific segment
            segment_vector = target_joints - virtual_joints
            segment_total_dist = np.max(np.abs(segment_vector))
            # If segment is tiny, just skip to next to avoid divide-by-zero errors
            if segment_total_dist < 0.001:
                continue

            while True:
                loop_start = time.perf_counter()

                # Calculate distance from VIRTUAL point to TARGET waypoint
                error_vector = target_joints - virtual_joints
                dist_to_target = np.max(np.abs(error_vector))

                # Check if Virtual Point has reached the target
                # STRICTNESS: We use a very small tolerance (0.05) for the virtual point
                # so it essentially hits the exact coordinate.
                if dist_to_target < 0.05:
                    # Snap exactly to target to prevent float drift
                    virtual_joints = np.copy(target_joints)
                    break
                
                # Decelerate ONLY if it's the very last point
                if is_last_point:
                    # Braking distance logic
                    max_reachable_vel = np.sqrt(2 * ACCEL * dist_to_target) # v = u^2 + 2as -> u = sqrt(v^2 - 2as), here v=0
                    target_vel = min(MAX_VELOCITY, max_reachable_vel)
                    target_vel = max(target_vel, 0.05) # Minimum creep speed
                else:
                    # Cruise speed for intermediate points
                    target_vel = MAX_VELOCITY

                # Ramp Velocity
                if current_velocity < target_vel:
                    current_velocity += ACCEL
                elif current_velocity > target_vel:
                    current_velocity -= ACCEL

                # Clip velocity so we don't overshoot the target in 1 step
                step_magnitude = min(current_velocity, dist_to_target)

                # Normalize vector to get direction
                direction = error_vector / dist_to_target
                step_vector = direction * step_magnitude

                virtual_joints += step_vector
                self.client_dash.ServoJ(*virtual_joints)

                if target_io != -1 and target_io != last_io_state:
                    self.client_dash.ToolDOInstant(0, target_io)
                    last_io_state = target_io

                elapsed = time.perf_counter() - loop_start
                if elapsed < delay_time:
                    time.sleep(delay_time - elapsed)

        # Ensure the PHYSICAL robot actually catches up to the final virtual point
        start_wait = time.time()
        while time.time() - start_wait < 0.5: # 0.5s timeout
            real_pos = np.array(self.feedData.Qactual)
            # Check error between Real Robot and Final Waypoint
            final_target = absolute_waypoints[-1][0]
            if np.max(np.abs(final_target - real_pos)) < threshold:
                break
            # Keep holding the position
            self.client_dash.ServoJ(*final_target)
            time.sleep(delay_time)

    def move_trajectory_RelMovJ(self, trajectory_points: np.ndarray):
        """
        Executes a trajectory using `RelMovJ` for each point.
        """
        if not self.client_dash or not self.robot_state.get('enable', False):
            print("Cannot move trajectory: client not initialized or robot not enabled.")
            return
        
        joints_trajectory = trajectory_points[:, :6]
        end_effector_state = trajectory_points[:, 6:]

        self.client_dash.CP(0)  # Set smooth transition ratio

        for i, point in enumerate(joints_trajectory):
            if len(point) != 6:
                print(f"Invalid trajectory point: {point}")
                continue

            self.client_dash.RelJointMovJ(*point)

            if end_effector_state.size > 0:
                print(end_effector_state.shape, end_effector_state[i, 0])
                self.client_dash.ToolDOInstant(0, int(end_effector_state[i, 0]))

            self.wait_robot_completed()

    ##########################
    # Robot Actuator Methods #
    ##########################
    def enable(self):
        if self.client_dash and not self.robot_state.get('enable', False):
            self.robot_state['enable'] = True
            return self.parseResultId(self.client_dash.EnableRobot())
        elif self.robot_state.get('enable', False):
            print("Robot is already enabled.")
        else:
            print("Dashboard client is not initialized.")
        return None
    
    def disable(self):
        if self.client_dash and self.robot_state.get('enable', False):
            self.robot_state['enable'] = False
            self.client_dash.DisableRobot()
        elif not self.robot_state.get('enable', False):
            print("Robot is already disabled.")
        else:
            print("Dashboard client is not initialized.")
    
    #############################
    # Feedback Handling Methods #
    #############################
    def get_feedback(self):
        # Get robot status
        while self.robot_state.get('ready', False):
            feedInfo = self.client_feed.feedBackData()
            with self.__globalLockValue:
                if feedInfo is not None:   
                    if hex((feedInfo['TestValue'][0])) == '0x123456789abcdef':
                        # Basic fields
                        self.feedData.MessageSize = feedInfo['len'][0]
                        self.feedData.robotMode = feedInfo['RobotMode'][0]
                        self.feedData.Qactual = feedInfo['QActual'][0]
                        self.feedData.DigitalInputs = feedInfo['DigitalInputs'][0]
                        self.feedData.DigitalOutputs = feedInfo['DigitalOutputs'][0]
                        self.feedData.robotCurrentCommandID = feedInfo['CurrentCommandId'][0]

                        # print(f"Robot Mode: {self.feedData.robotMode}, Digital Inputs: {self.feedData.DigitalInputs}, Qactual: {feedInfo['QActual'][0]}")
    
    def parseResultId(self, valueRecv):
        # Parse return value to ensure robot is in TCP control mode
        if "not tcp" in str(valueRecv).lower():
            print("Control Mode Is Not Tcp")
            return [1]
        return [int(num) for num in re.findall(r'-?\d+', valueRecv)] or [2]
    
    # Function that parallelly robot completion signal (robot mode = 5)
    def wait_robot_completed(self, idle_mode: int = 5, timeout: float = 5):
        self.robot_state['mode_number'] = self.feedData.robotMode
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Check if robot mode has changed to idle_mode
            if self.feedData.robotMode == idle_mode and self.robot_state['mode_number'] != idle_mode:
                print("Robot has completed the action.")
                return True
            self.robot_state['mode_number'] = self.feedData.robotMode
        return False
    
    #############################
    #   Error Handling Methods  #
    ############################# 
    def display_error_info(self):
        # Try to use GetError interface first
        try:
            error_info = self.client_dash.GetError("en")  # Use English for UI display
            if error_info and "errMsg" in error_info and error_info["errMsg"]:
                # Use new GetError interface
                for error in error_info["errMsg"]:
                    self.form_error_new(error)
                return
        except Exception as e:
            print(f"GetError interface failed, using fallback method: {e}")
        
        # Fallback to original method
        try:
            error_list = self.client_dash.GetErrorID().split("{")[1].split("}")[0]
            error_list =    json.loads(error_list)
            print("error_list:", error_list)
            if error_list[0]:
                for i in error_list[0]:
                    self.form_error(i, self.alarm_controller_dict,
                                    "Controller Error")
            for m in range(1, len(error_list)):
                if error_list[m]:
                    for n in range(len(error_list[m])):
                        self.form_error(n, self.alarm_servo_dict, "Servo Error")
        except Exception as e:
            print(f"Both error retrieval methods failed: {e}")

    def form_error_new(self, error_data):
        """Handle error data from GetError interface"""
        try:
            error_info = f"Time Stamp:{error_data.get('date', 'N/A')} {error_data.get('time', 'N/A')}\n"
            error_info += f"ID:{error_data.get('id', 'N/A')}\n"
            error_info += f"Type:{error_data.get('mode', 'N/A')}\n"
            error_info += f"Level:{error_data.get('level', 'N/A')}\n"
            error_info += f"Description:{error_data.get('description', 'N/A')}\n"
            error_info += f"Solution:{error_data.get('solution', 'N/A')}\n\n"
        except Exception as e:
            print(f"Error formatting new error data: {e}")
    
    def form_error(self, index, alarm_dict: dict, type_text):
        if index in alarm_dict.keys():
            date = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            error_info = f"Time Stamp:{date}\n"
            error_info = error_info + f"ID:{index}\n"
            error_info = error_info + \
                f"Type:{type_text}\nLevel:{alarm_dict[index]['level']}\n" + \
                f"Solution:{alarm_dict[index]['en']['solution']}\n"

    ###################
    # Helper Function #
    ###################
    def post_process_trajectory(self, file_path: str, output_path: str = None, strict_mode: bool = False):
        # 1. Load Data
        try:
            data = np.load(file_path)
        except FileNotFoundError:
            print(f"File not found: {file_path}")
            return

        print(f"Original shape: {data.shape}")

        if data.shape[0] < 2:
            print("Trajectory too short to process.")
            return
        
        # Calculate difference between current row [1:] and previous row [:-1]
        diffs = np.abs(data[1:] - data[:-1])

        # Check if row is identical to the previous one (max diff is near zero)
        is_duplicate = np.max(diffs, axis=1) < 1e-6

        is_static_movement = np.max(np.abs(data[1:, :6]), axis=1) < 1e-6

        # Check if IO changed (we usually want to keep frames where IO changes)
        # IO is column 6
        io_diff = np.abs(data[1:, 6] - data[:-1, 6])
        is_io_stable = io_diff < 1e-6

        # 4. Filter Logic
        if strict_mode:
            mask_to_keep = ~is_duplicate
        else:
            should_remove = is_duplicate & is_static_movement & is_io_stable
            mask_to_keep = ~should_remove
        
        final_mask = np.concatenate(([True], mask_to_keep))

        cleaned_data = data[final_mask]

        if output_path is None:
            output_path = file_path

        np.save(output_path, cleaned_data)

        removed_count = data.shape[0] - cleaned_data.shape[0]
        percent = (removed_count / data.shape[0]) * 100

        print(f"Processing Complete.")
        print(f"Removed {removed_count} frames ({percent:.1f}% reduction).")
        print(f"New shape: {cleaned_data.shape}")
        print(f"Saved to: {output_path}")

        return cleaned_data

    def record_trajectory(self, record_duration: float = 10.0, record_hz: float = 50.0, threshold: float = 0.01, save_path: str = "recorded_trajectory.npy"):
        """
        Records trajectory with a 'Time Remaining' warning system.
        """
        if not self.client_dash:
            print("Client not connected.")
            return

        num_steps = int(record_duration * record_hz)
        recorded_data = np.zeros((num_steps, 7)) 

        print(f"Recording started: {record_duration}s at {record_hz}Hz")
        print("Move the robot now!")

        self.client_dash.StartDrag()
        time.sleep(0.5)

        period = 1.0 / record_hz
        last_stable_joints = np.array(self.feedData.Qactual)

        start_time = time.perf_counter()
        next_wake_time = start_time + period

        # Warning State
        last_printed_second = -1

        try:
            for i in range(num_steps):
                # --- 1. WARNING SYSTEM ---
                # Calculate exact time remaining
                now = time.perf_counter()
                elapsed_total = now - start_time
                remaining = record_duration - elapsed_total

                # A. 3-Second Countdown (Standard Print)
                if remaining <= 3.1 and remaining > 0:
                    current_int_sec = int(remaining)
                    if current_int_sec != last_printed_second:
                        print(f"⚠️  ENDING IN {current_int_sec + 1}...")
                        last_printed_second = current_int_sec

                # B. Visual Progress Bar (Overwrites same line)
                # Update bar every ~0.5s or so to avoid console flicker
                if i % int(record_hz / 2) == 0:
                    progress = elapsed_total / record_duration
                    bar_len = 30
                    filled = int(bar_len * progress)
                    bar = '█' * filled + '-' * (bar_len - filled)
                    sys.stdout.write(f"\rRecording: [{bar}] {remaining:.1f}s left ")
                    sys.stdout.flush()

                # --- 2. DATA RECORDING ---
                current_joints = np.array(self.feedData.Qactual)
                io_state = int(self.feedData.DigitalOutputs)

                raw_delta = current_joints - last_stable_joints

                # Deadband Filter
                if np.max(np.abs(raw_delta)) < threshold:
                    clean_delta = np.zeros(6)
                else:
                    clean_delta = raw_delta
                    last_stable_joints = current_joints

                recorded_data[i, :6] = clean_delta
                recorded_data[i, 6] = io_state

                # --- 3. TIMING ---
                now = time.perf_counter()
                sleep_time = next_wake_time - now

                if sleep_time > 0:
                    time.sleep(sleep_time)

                next_wake_time += period

        except KeyboardInterrupt:
            print("\nRecording interrupted by user.")
            recorded_data = recorded_data[:i]

        finally:
            sys.stdout.write("\n") # Move to next line after progress bar
            self.client_dash.StopDrag()
            np.save(save_path, recorded_data)
            print(f"✅ Saved {len(recorded_data)} frames to {save_path}")

def reset_position(controller: DobotController):
    time.sleep(1)
    controller.client_dash.MovJ(*controller.home_position, 0)
    controller.wait_robot_completed()

if __name__ == "__main__":

    controller = DobotController(
        dashboard_ip="192.168.5.1",
        feedback_ip="192.168.5.1"
    )

    controller.start()
    time.sleep(1)  # Let it run for a while

    if controller.robot_state.get('ready', False):

        reset_position(controller)

        trajectory = np.array([
            [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1],
            [-10, 0.0, 0.0, 0.0, 0.0, 0.0, 0]
        ])

        controller.move_trajectory_smooth(
            trajectory_points=trajectory,
            control_frequency=40.0,
            threshold=0.25,
            speed_scale=0.25
        )

        time.sleep(1)

        reset_position(controller)

    controller.shutdown()