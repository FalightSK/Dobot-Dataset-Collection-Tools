# Dobot Dataset Collection

Python tools to control a Dobot robot, record joint/IO trajectories, and build a multimodal dataset (trajectory, state, and camera images) for downstream learning tasks.

## What this project does
- Connects to a Dobot controller via TCP dashboard (port 29999) and feedback (port 30004).
- Records **relative joint deltas + tool digital output** at a fixed rate.
- Post-processes trajectories to remove redundant frames.
- Replays trajectories and captures **robot state + IO state + camera images** to build a dataset.

## Project structure
- dobot_controller.py: Core control logic (start/stop, feedback thread, trajectory record/replay).
- dataset_collection.py: Record and post-process a trajectory file.
- dataset_creation.py: Replay trajectories, capture images, and save dataset fragments.
- dobot_api/: Low-level TCP dashboard and feedback API wrappers.
- dataset/: Output data folders (images, state, trajectory fragments).

## Requirements
- Python 3.8+
- Dobot robot reachable over TCP
- A connected camera (OpenCV-compatible)

Python packages:
- numpy
- opencv-python
- requests

## Setup
1. Connect the Dobot controller on your network.
2. Update the IP address in scripts if your Dobot is not at 192.168.5.1.
3. Ensure your camera index is correct (default is 0).

## Usage
### 1) Record a trajectory
Edit the target file name in dataset_collection.py and run:
- This records a trajectory to recorded_trajectory.npy and then post-processes it to the final output file.

### 2) Create the dataset
Edit the `filename` in dataset_creation.py and run:
- This loads the trajectory file, replays it in segments, and saves:
  - dataset/image/<name>/<i>.png
  - dataset/state/<name>/<i>.npy
  - dataset/trajectory/<name>/<i>.npy

## Data formats
### Trajectory file
Each row is a 7D vector:
- [0..5] = joint deltas (degrees)
- [6] = tool digital output (0/1)

### State file
Each state file is a 1D vector:
- First 6 values = current joint angles
- Remaining values = digital output states

## Notes and safety
- Always keep the robot workspace clear before running scripts.
- The scripts assume the robot is enabled and ready. If not, check controller status and alarms.
- Adjust thresholds, speed scales, and frequencies in dobot_controller.py as needed.

## Quick script overview
- dataset_collection.py
  - `record_trajectory()` with configurable duration and rate
  - `post_process_trajectory()` to remove static duplicate frames
- dataset_creation.py
  - Splits the trajectory into fixed-length segments
  - Captures camera frames and robot states before each segment
  - Executes the segment with `move_trajectory_smooth()`

## Troubleshooting
- If `EnableRobot` fails, ensure port 29999 is free.
- If feedback drops, check network stability and firewall rules.
- If the camera returns blank images, confirm the camera index and permissions.
