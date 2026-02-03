import os
import time

from dobot_controller import DobotController

import numpy as np
import cv2

def reset_position(controller: DobotController):
    time.sleep(1)
    controller.client_dash.MovJ(*controller.home_position, 0)
    controller.wait_robot_completed()

if __name__ == "__main__":

    # Define Important Variables #
    trajectory_len = 80

    filename = "or-rbr_3"

    image_path = f"dataset/image/{filename}/"
    state_path = f"dataset/state/{filename}/"
    trajectory_path = f"dataset/trajectory/{filename}/"

    cap = cv2.VideoCapture(0) # Change the index if you have multiple cameras

    controller = DobotController(
        dashboard_ip="192.168.5.1",
        feedback_ip="192.168.5.1"
    )

    controller.start()
    time.sleep(1)  # Let it run for a while

    if controller.robot_state.get('ready', False):

        # Create directory to save dataset if it doesn't exist
        os.makedirs(image_path, exist_ok=True)
        os.makedirs(state_path, exist_ok=True)
        os.makedirs(trajectory_path, exist_ok=True)

        # Load Trajectory from File created by `dataset_collection.py`
        trajectory = np.load(f"{filename}.npy")
        print(f"Trajectory shape: {trajectory.shape}")

        # Loop through Trajectory in Segments to save the Robot State, IO State and Environment Image
        indices = list(range(0, len(trajectory), trajectory_len))
        for i, points in enumerate(indices):

            # Save Robot and IO State Before Movement
            robot_state = np.array(controller.feedData.Qactual)
            io_state = np.array(controller.feedData.DigitalOutputs)
            combined_state = np.concatenate((robot_state, io_state), axis=0)
            np.save(f"{state_path}{str(i)}.npy", combined_state)

            # Save Environment Image
            ret, frame = cap.read()
            if ret:
                cv2.imwrite(f"{image_path}{str(i)}.png", frame)

            # Cut Trajectory into Fragments
            fragment = trajectory[points:points+trajectory_len]
            print(f"Joints {trajectory[points:points+trajectory_len, :6].shape}, IO {trajectory[points:points+trajectory_len, 6].shape}")

            # Save Trajectory Fragment
            np.save(f"{trajectory_path}{str(i)}.npy", fragment)

            # Move Robot along Trajectory Fragment
            controller.move_trajectory_smooth(
                trajectory_points=fragment,
                control_frequency=80.0,
                threshold=0.25,
                speed_scale=1.0
            )

            time.sleep(1) # Make sure Robot has stopped for clear image

        # reset_position(controller)

    controller.shutdown()