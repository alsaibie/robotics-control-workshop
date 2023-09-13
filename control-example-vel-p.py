import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle

import numpy as np

def setup_plot():
    fig, ax = plt.subplots()
    ax.set_xlim(0, 12)
    ax.set_ylim(-6, 6)
    ax.set_aspect('equal', 'box')
    
    return fig, ax

def setup_plot_elements(ax):
    line1, = ax.plot([], [], 'r-', label='Desired Path')
    line2, = ax.plot([], [], 'b-', label='Robot Path')
    ax.legend()
    robot_body = Circle((0, 0), 0.4, fill=False, color='blue')
    robot_wheel1 = Circle((0, 0), 0.1, fill=True, color='black')
    robot_wheel2 = Circle((0, 0), 0.1, fill=True, color='black')
    
    ax.add_patch(robot_body)
    ax.add_patch(robot_wheel1)
    ax.add_patch(robot_wheel2)
    
    return line1, line2, robot_body, robot_wheel1, robot_wheel2

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2,


# Initialize parameters and variables
dt = 0.1
m, r, I = 0.8, .1, 0.008;

k_p_pos = 4 # 4
k_p_yaw = 4 # 4
x, y, yaw, v, omega = 0.0, 0.0, 0.0, 0.0, 0.0

# S-shaped trajectory
t_points = np.linspace(0, 10, 100)
x_trajectory = t_points
y_trajectory = 2 * np.sin(t_points)


robot_path_x = []
robot_path_y = []

def robot_update(frame):
    global x, y, yaw, v, omega

    # Target and current positions
    target_x, target_y = x_trajectory[frame], y_trajectory[frame]
    dx, dy = target_x - x, target_y - y

    # Position error and control signal for position
    e_pos = np.sqrt(dx ** 2 + dy ** 2)
    F_sum = k_p_pos * e_pos

    # Yaw error and control signal for yaw
    r_yaw = np.arctan2(dy, dx)
    e_yaw = r_yaw - yaw
    F_yaw = k_p_yaw * e_yaw

    # omega (+ ccw)
    v_l = F_sum - F_yaw * r / 2
    v_r = F_sum + F_yaw * r / 2
    
    # Apply control signals to update robot state
    v = (v_l + v_r) / 2  # Linear speed
    omega = (v_r - v_l) / r  # Angular speed
    
    x += v * np.cos(yaw) * dt
    y += v * np.sin(yaw) * dt
    yaw += omega * dt

    # Save robot's path for plotting
    robot_path_x.append(x)
    robot_path_y.append(y)

    line1.set_data(x_trajectory[:frame + 1], y_trajectory[:frame + 1])
    line2.set_data(robot_path_x, robot_path_y)
    
    # Update robot figure 
    robot_body.center = (x, y)
    robot_wheel1.center = (x - 0.5 * np.sin(yaw), y + 0.5 * np.cos(yaw))
    robot_wheel2.center = (x + 0.5 * np.sin(yaw), y - 0.5 * np.cos(yaw))

    return line1, line2, robot_body, robot_wheel1, robot_wheel2,


fig, ax = setup_plot()
line1, line2, robot_body, robot_wheel1, robot_wheel2 = setup_plot_elements(ax)
ani = animation.FuncAnimation(fig, robot_update, frames=len(t_points),
                              init_func=init, blit=True, repeat=False)
plt.show()