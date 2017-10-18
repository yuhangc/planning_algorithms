import numpy as np

# dynamic simulation tolerance
TOL_DYN = 1e-3


def set_cmd_vel(vel_curr, vel_cmd, max_vel, max_acc, dt):
    """
    Set robot velocity with velocity and acceleration constraint
    """
    if np.abs(vel_cmd - vel_curr) < max_acc * dt:
        vel_next = vel_cmd
    else:
        vel_next = vel_curr + np.sign(vel_cmd - vel_curr) * max_acc * dt

    vel_next = np.clip(vel_next, -max_vel, max_vel)
    return vel_next
