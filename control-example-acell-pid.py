# Expand the robot model to command force not velocity
# Add PID since the model is now second order
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
dt = 0.01
m, r, I = 0.8, .1, 0.008;

k_p_pos = 8 # 8
k_i_pos = 1 # 0
k_d_pos = 4 # 0

k_p_yaw = 80 # 80
k_i_yaw = 3 # 0
k_d_yaw = 35 # 35

x, y, theta, v, omega = 0.0, 0.0, 0.0, 0.0, 0.0

# S-shaped trajectory
t_points = np.linspace(0, 10, 100)
x_trajectory = t_points
y_trajectory = 2 * np.sin(t_points)

robot_path_x = []
robot_path_y = []

e_pos_int, e_pos_dot, e_yaw_int, e_yaw_dot, e_pos_prev, e_yaw_prev = 0, 0, 0, 0, 0, 0

def robot_update(frame):
    global x, y, theta, v, omega
    global e_pos_int, e_pos_dot, e_yaw_int, e_yaw_dot, e_pos_prev, e_yaw_prev
    # Target and current positions
    target_x, target_y = x_trajectory[frame], y_trajectory[frame]
    dx, dy = target_x - x, target_y - y

    # Position error and control signal for position
    e_pos = np.sqrt(dx ** 2 + dy ** 2)
    e_pos_dot = (e_pos - e_pos_prev) / dt
    e_pos_prev = e_pos
    e_pos_int += e_pos * dt
    F_pos = k_p_pos * e_pos + k_i_pos * e_pos_int + k_d_pos * e_pos_dot
    
    # Yaw error and control signal for yaw
    r_theta = np.arctan2(dy, dx)
    e_yaw = r_theta - theta
    e_yaw_dot = (e_yaw - e_yaw_prev) / dt
    e_yaw_prev = e_yaw
    e_yaw_int += e_yaw * dt
    F_yaw = k_p_yaw * e_yaw + k_i_yaw * e_yaw_int + k_d_yaw * e_yaw_dot
    
    F_l = F_pos - F_yaw * r / 2
    F_r = F_pos + F_yaw * r / 2
    
    v += (F_l+F_r)/m * dt 
    omega += (F_r-F_l)*r / I * dt  
    
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt

    # Save robot's path for plotting
    robot_path_x.append(x)
    robot_path_y.append(y)

    line1.set_data(x_trajectory[:frame + 1], y_trajectory[:frame + 1])
    line2.set_data(robot_path_x, robot_path_y)
    
    # Update robot figure 
    robot_body.center = (x, y)
    robot_wheel1.center = (x - 0.5 * np.sin(theta), y + 0.5 * np.cos(theta))
    robot_wheel2.center = (x + 0.5 * np.sin(theta), y - 0.5 * np.cos(theta))

    return line1, line2, robot_body, robot_wheel1, robot_wheel2,


fig, ax = setup_plot()
line1, line2, robot_body, robot_wheel1, robot_wheel2 = setup_plot_elements(ax)
ani = animation.FuncAnimation(fig, robot_update, frames=len(t_points),
                              init_func=init, blit=True, repeat=False)
plt.show()