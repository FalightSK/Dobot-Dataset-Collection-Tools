import time
from dobot_controller import DobotController

def create_dataset(controller: DobotController, final_file: str = "null.npy"):

    controller.record_trajectory(
            record_duration=5.0,
            record_hz=80.0,
            threshold=0.025,
            save_path="recorded_trajectory.npy"
    )

    time.sleep(1)

    controller.post_process_trajectory(
            file_path="recorded_trajectory.npy",
            output_path=final_file,
            strict_mode=False
    )

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

        create_dataset(controller, final_file="bl-flf_1.npy")
        # reset_position(controller)
        pass

    controller.shutdown()