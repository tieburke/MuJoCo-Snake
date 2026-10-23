import csv
import os
import time
import numpy as np
import mujoco
import mujoco.viewer

# Get absolute path of current script directory to ensure the CSV is created alongside it
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(SCRIPT_DIR, "actuator_torques.csv")
XML_PATH = os.path.join(SCRIPT_DIR, "planar_snake_cars_servo.xml")

# ---- Optimized Gait Parameters ----
NUM_JOINTS = 10
AMPLITUDE = 0.33
FREQUENCY_HZ = 0.5
TOTAL_PHASE_SPREAD = 4*np.pi
DIRECTION = -1

OMEGA = 2 * np.pi * FREQUENCY_HZ
PHASE = np.array(
    [DIRECTION * i * TOTAL_PHASE_SPREAD / NUM_JOINTS for i in range(1, NUM_JOINTS + 1)]
)

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

# Resolve actuator IDs
actuator_ids = [
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator_joint{i:02d}")
    for i in range(1, NUM_JOINTS + 1)
]
assert all(aid >= 0 for aid in actuator_ids), "Actuator name mismatch -- check XML."

# Initialize CSV file and write header
csv_file = open(CSV_PATH, mode="w", newline="")
csv_writer = csv.writer(csv_file)
headers = ["time_s"] + [f"joint_{i+1}_torque_Nm" for i in range(NUM_JOINTS)]
csv_writer.writerow(headers)

try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 12
        viewer.cam.lookat[:] = [0, 0, 0]

        sim_start_time = time.time()

        while viewer.is_running():
            # Step physics until simulation time catches up with real elapsed time
            real_time_elapsed = time.time() - sim_start_time

            while data.time < real_time_elapsed:
                # Calculate targets using exact simulation time (data.time)
                for idx, aid in enumerate(actuator_ids):
                    data.ctrl[aid] = AMPLITUDE * np.sin(OMEGA * data.time + PHASE[idx])

                mujoco.mj_step(model, data)

                # Record time and joint torques at every simulation timestep (500 Hz)
                torques = [data.actuator_force[aid] for aid in actuator_ids]
                csv_writer.writerow([round(data.time, 4)] + torques)

            viewer.sync()
            time.sleep(0.001)

finally:
    csv_file.close()
    print(f"Torque data successfully saved to: {CSV_PATH}")