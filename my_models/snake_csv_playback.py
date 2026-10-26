"""
Drive the planar snake's 10 body-hinge actuators from a CSV file of angles.

CSV format expected:
  - A header row (contents ignored -- columns are read by POSITION, not name)
  - 10 numeric columns, one per actuator (joint01 .. joint10, in that order)
  - One row per time point, in radians
  - Consecutive rows are DT_BETWEEN_POINTS seconds apart (see constant below)

Between CSV rows, actuator targets are linearly interpolated so the motion
is smooth rather than snapping. After the last row, the snake holds that
final pose indefinitely.

Run:
    python snake_csv_playback.py path/to/angles.csv
(assumes planar_snake_cars_servo.xml is in the same directory --
 edit XML_PATH below if it lives elsewhere)
"""

import ctypes

import argparse
import csv
import time

import numpy as np
import mujoco
import mujoco.viewer

XML_PATH = "planar_snake_cars_servo.xml"
NUM_JOINTS = 10
DT_BETWEEN_POINTS = 0.05  # seconds between consecutive CSV rows


def load_angles_csv(path):
    """Read a CSV (header row + 10 numeric columns) into an (N, NUM_JOINTS) array."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)  # skip header row
        rows = [[float(v) for v in row[1:]] for row in reader if row]

    if not rows:
        raise ValueError(f"No data rows found in {path}")

    angles = np.array(rows, dtype=float)
    if angles.shape[1] < NUM_JOINTS:
        raise ValueError(
            f"Expected at least {NUM_JOINTS} columns, found {angles.shape[1]} in {path}"
        )
    if angles.shape[1] > NUM_JOINTS:
        print(
            f"Warning: {path} has {angles.shape[1]} columns; "
            f"using only the first {NUM_JOINTS}."
        )
    return angles[:, :NUM_JOINTS]


def get_target_angles(angles, t):
    """Linearly interpolate actuator targets at simulation time t (seconds).
    Holds the final row's pose once t exceeds the last time point."""
    num_points = angles.shape[0]
    last_time = (num_points - 1) * DT_BETWEEN_POINTS

    if num_points == 1 or t >= last_time:
        return angles[-1]

    idx = int(t // DT_BETWEEN_POINTS)
    frac = (t - idx * DT_BETWEEN_POINTS) / DT_BETWEEN_POINTS
    return angles[idx] * (1.0 - frac) + angles[idx + 1] * frac


def main():
    parser = argparse.ArgumentParser(
        description="Play back a CSV of actuator angles on the planar snake robot."
    )
    parser.add_argument(
        "csv_path",
        help="Path to CSV file: header row + 10 columns of actuator angles "
        "(radians), one row per %.0fms time point." % (DT_BETWEEN_POINTS * 1000),
    )
    args = parser.parse_args()

    angles = load_angles_csv(args.csv_path)
    duration = (angles.shape[0] - 1) * DT_BETWEEN_POINTS
    print(
        f"Loaded {angles.shape[0]} time points from {args.csv_path} "
        f"(~{duration:.2f}s of motion, then holds the last pose)"
    )

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    actuator_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_joint{i:02d}")
        for i in range(1, NUM_JOINTS + 1)
    ]
    assert all(aid >= 0 for aid in actuator_ids), "Actuator name mismatch -- check XML."

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 12
        viewer.cam.lookat[:] = [0, 0, 0]

        sim_start_time = time.time()

        user32 = ctypes.windll.user32
        hWnd = user32.GetForegroundWindow()
        SW_MAXIMIZE = 3
        user32.ShowWindow(hWnd, SW_MAXIMIZE)

        while viewer.is_running():
            # Step physics until simulation time catches up with real elapsed time
            real_time_elapsed = time.time() - sim_start_time

            while data.time < real_time_elapsed:
                target = get_target_angles(angles, data.time)
                for idx, aid in enumerate(actuator_ids):
                    data.ctrl[aid] = target[idx]
                mujoco.mj_step(model, data)

            viewer.sync()
            time.sleep(0.001)


if __name__ == "__main__":
    main()
