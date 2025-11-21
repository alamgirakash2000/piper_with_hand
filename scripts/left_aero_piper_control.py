import mujoco
import mujoco.viewer
import numpy as np
import os
import sys

# Allow importing from scripts/module
sys.path.append(os.path.join(os.path.dirname(__file__), "module"))
from physical_to_mujoco import physical_to_mujoco

# Load the scene (which includes the robot model)
model = mujoco.MjModel.from_xml_path("assets/scene_left.xml")
data = mujoco.MjData(model)

print(f"Number of actuators: {model.nu}")
print(f"Actuator names: {[model.actuator(i).name for i in range(model.nu)]}")
ctrlrange = np.array(model.actuator_ctrlrange)
print("Ctrl ranges (min, max):")
for i in range(model.nu):
    print(f"  {i:2d}: {model.actuator(i).name:>24s} -> {ctrlrange[i]}")

def set_positions(data, positions):
    """Set target positions for the 13 actuators (6 arm + 7 hand)"""
    assert len(positions) == 13
    data.ctrl[:] = positions

# Physical trajectory straight from run_aero_sequence.py
# Format: (name, [7 DOF physical values], duration_seconds)
PHYSICAL_TRAJECTORY_NAMED = [
    ("Open Palm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
    ("Touch Pinkie", [100.0, 35.0, 23.0, 0.0, 0.0, 0.0, 50.0], 0.5),
    ("Touch Pinkie - Hold", [100.0, 35.0, 23.0, 0.0, 0.0, 0.0, 50.0], 0.25),
    ("Touch Ring", [100.0, 42.0, 23.0, 0.0, 0.0, 52.0, 0.0], 0.5),
    ("Touch Ring - Hold", [100.0, 42.0, 23.0, 0.0, 0.0, 52.0, 0.0], 0.25),
    ("Touch Middle", [83.0, 42.0, 23.0, 0.0, 50.0, 0.0, 0.0], 0.5),
    ("Touch Middle - Hold", [83.0, 42.0, 23.0, 0.0, 50.0, 0.0, 0.0], 0.25),
    ("Touch Index", [75.0, 25.0, 30.0, 50.0, 0.0, 0.0, 0.0], 0.5),
    ("Touch Index - Hold", [75.0, 25.0, 30.0, 50.0, 0.0, 0.0, 0.0], 0.25),
    ("Open Palm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ("Open Palm - Hold", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ("Peace Sign", [90.0, 0.0, 0.0, 0.0, 0.0, 90.0, 90.0], 0.5),
    ("Peace Sign - Thumb Flex", [90.0, 45.0, 60.0, 0.0, 0.0, 90.0, 90.0], 0.5),
    ("Peace Sign - Hold", [90.0, 45.0, 60.0, 0.0, 0.0, 90.0, 90.0], 1.0),
    ("Open Palm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ("Open Palm - Hold", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
    ("Rockstar Sign", [0.0, 0.0, 0.0, 0.0, 90.0, 90.0, 0.0], 0.5),
    ("Rockstar Sign - Hold", [0.0, 0.0, 0.0, 0.0, 90.0, 90.0, 0.0], 1.0),
    ("Open Palm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
]

ACTIVE_TRAJECTORY_NAMED = PHYSICAL_TRAJECTORY_NAMED

def get_trajectory_control(t: float, trajectory_named, transition_time=0.3):
    """
    Convert the physical trajectory at time t into MuJoCo controls.
    """
    total_time = sum(duration for _, _, duration in trajectory_named)
    t_loop = t % total_time

    accumulated_time = 0.0
    for idx, (_, pose, duration) in enumerate(trajectory_named):
        if accumulated_time + duration > t_loop:
            time_in_segment = t_loop - accumulated_time
            current_pose = np.array(pose, dtype=float)
            next_idx = (idx + 1) % len(trajectory_named)
            next_pose = np.array(trajectory_named[next_idx][1], dtype=float)

            current_ctrl = physical_to_mujoco(current_pose)
            next_ctrl = physical_to_mujoco(next_pose)

            if duration - time_in_segment < transition_time:
                trans = (transition_time - (duration - time_in_segment)) / transition_time
                trans = min(1.0, max(0.0, trans))
                smooth_trans = 3 * trans**2 - 2 * trans**3
                return current_ctrl + (next_ctrl - current_ctrl) * smooth_trans
            return current_ctrl
        accumulated_time += duration

    return physical_to_mujoco(trajectory_named[0][1])

# Interactive viewer with finger sequence + random arm movement
with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0
    np.random.seed(42)  # For reproducible random movements
    arm_change_interval = 3.0  # Change arm target every 3 seconds
    next_arm_change = arm_change_interval
    current_arm_targets = np.zeros(6)
    next_arm_targets = np.random.uniform(-0.5, 0.5, 6)
    
    while viewer.is_running():
        # Smooth random arm movement
        if t >= next_arm_change:
            current_arm_targets = next_arm_targets.copy()
            next_arm_targets = np.random.uniform(-0.5, 0.5, 6)
            next_arm_change = t + arm_change_interval
        
        # Interpolate between current and next arm targets
        transition = (t - (next_arm_change - arm_change_interval)) / arm_change_interval
        transition = min(1.0, max(0.0, transition))
        # Use smooth interpolation
        smooth_transition = 3 * transition**2 - 2 * transition**3  # Smoothstep
        arm_targets = current_arm_targets + (next_arm_targets - current_arm_targets) * smooth_transition
        
        # Follow the physical trajectory exactly
        hand_targets = get_trajectory_control(t, ACTIVE_TRAJECTORY_NAMED)
        
        # Combine arm and hand targets
        targets = np.concatenate([arm_targets, hand_targets])
        set_positions(data, targets)

        mujoco.mj_step(model, data)
        viewer.sync()
        t += 0.01
