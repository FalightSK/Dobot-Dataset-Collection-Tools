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
    save = True

    trajectory_len = 40
    buffer_size = trajectory_len // 2

    cate= "free"
    raw_path = f"./dataset_raw/{cate}/6/"
    filename = "up_to_yellow_f_blue_6"

    image_path = f"dataset/image/{filename}_{cate}/"
    state_path = f"dataset/state/{filename}_{cate}/"
    trajectory_path = f"dataset/trajectory/{filename}_{cate}/"

    cam_url = r"http://192.168.137.21:81/stream"
    overview = cv2.VideoCapture(1) # Change the index if you have multiple cameras
    endview = cv2.VideoCapture(cam_url)
    _, frame = overview.read()
    del frame
    _, frame = endview.read()
    del frame

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
        trajectory = np.load(f"{raw_path}{filename}.npy")
        print(f"Trajectory shape: {trajectory.shape}")

        # Loop through Trajectory in Segments to save the Robot State, IO State and Environment Image
        indices = list(range(0, len(trajectory), trajectory_len))
        for i, points in enumerate(indices):

            # Save Robot and IO State Before Movement
            robot_state = np.array(controller.feedData.Qactual)
            io_state = np.array([controller.feedData.DigitalOutputs])
            combined_state = np.concatenate((robot_state, io_state), axis=0)
            if save:
                np.save(f"{state_path}{str(i)}.npy", combined_state)

            # Save Environment Image
            if save:
                ret_1, frame_1 = endview.read()
                for j in range(3):
                    ret_1, frame_1 = endview.read()
                    if not ret_1:
                        print("Failed to capture end view image, retrying...")
                        endview.release()
                        endview = cv2.VideoCapture(cam_url)
                        time.sleep(0.25)
                ret, frame = overview.read()
                print("Saving Image...")
                if ret:
                    cv2.imwrite(f"{image_path}{str(i)}.png", frame)
                if ret_1:
                    cv2.imwrite(f"{image_path}{str(i)}_end.png", frame_1)
                print("Done with file path: ", f"{image_path}{str(i)}.png")
                

            # Cut Trajectory into Fragments
            trajectory_w_buffer = min(len(trajectory) - points, trajectory_len + buffer_size)
            fragment = trajectory[points:points+trajectory_w_buffer]
            steering_fragment = trajectory[points:points+trajectory_len]
            print(f"Joints {trajectory[points:points+trajectory_w_buffer, :6].shape}, IO {trajectory[points:points+trajectory_w_buffer, 6].shape}")

            if save:
                # Save Trajectory Fragment
                np.save(f"{trajectory_path}{str(i)}.npy", fragment)

            # Move Robot along Trajectory Fragment
            controller.move_trajectory_smooth(
                trajectory_points=steering_fragment,
                control_frequency=80.0,
                threshold=0.5,
                speed_scale=0.5
            )

            if save:
                time.sleep(1) # Make sure Robot has stopped for clear image

        # reset_position(controller)

    controller.shutdown()