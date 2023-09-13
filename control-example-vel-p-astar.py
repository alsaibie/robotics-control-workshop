# Example of combining the A* algorithm with trajectory following

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from astar import a_star_algorithm

import numpy as np
grid_size = 9
def setup_plot():
    fig, ax = plt.subplots()
    ax.set_xlim(0, grid_size)
    ax.set_ylim(-grid_size, 0)
    ax.set_aspect('equal', 'box')
    
    return fig, ax

grid = [[0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0]]

def setup_plot_elements(ax):
    line0, = ax.plot([], [], 'g-', label='A* Path')
    line1, = ax.plot([], [], 'r-', label='Interpolated Desired Path')
    line2, = ax.plot([], [], 'b-', label='Robot Path')
    ax.legend()
    robot_body = Circle((0, 0), 0.4, fill=False, color='blue')
    robot_wheel1 = Circle((0, 0), 0.1, fill=True, color='black')
    robot_wheel2 = Circle((0, 0), 0.1, fill=True, color='black')
    
    ax.add_patch(robot_body)
    ax.add_patch(robot_wheel1)
    ax.add_patch(robot_wheel2)
    
    # Plot obstacles
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1:
                obstacle = plt.Rectangle((j, -i - 1), 1, 1, color='black')  
                ax.add_patch(obstacle)
    
    
    
    return line0, line1, line2, robot_body, robot_wheel1, robot_wheel2

def init():
    line0.set_data([], [])
    line1.set_data([], [])
    line2.set_data([], [])
    return line0, line1, line2,

# Initialize parameters and variables
dt = 0.1
m, r, I = 0.8, .1, 0.008;

k_p_pos = 1.2 # 4
k_p_yaw = 1.2 # 4
x, y, yaw, v, omega = 0.0, 0.0, 0.0, 0.0, 0.0

robot_path_x = []
robot_path_y = []

# Your start and goal definition here
start = (0, 0)
goal = (8, 8)

# Use the A* algorithm to get waypoints
waypoints = a_star_algorithm(start, goal, grid)

from scipy.interpolate import interp1d
x_points, y_points = waypoints[:, 1]+.5, -(waypoints[:, 0]+0.5)

t_points = np.linspace(0, 1, len(waypoints))
interpolator_x = interp1d(t_points, x_points, kind='cubic')
interpolator_y = interp1d(t_points, y_points, kind='cubic')
t_smooth = np.linspace(0, 1, 100)
x_trajectory = interpolator_x(t_smooth)
y_trajectory = interpolator_y(t_smooth)


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
    line0.set_data(x_points, y_points)
    line1.set_data(x_trajectory[:frame + 1], y_trajectory[:frame + 1])
    line2.set_data(robot_path_x, robot_path_y)
    
    # Update robot figure 
    robot_body.center = (x, y)
    robot_wheel1.center = (x - 0.5 * np.sin(yaw), y + 0.5 * np.cos(yaw))
    robot_wheel2.center = (x + 0.5 * np.sin(yaw), y - 0.5 * np.cos(yaw))
    

                
    return line0, line1, line2, robot_body, robot_wheel1, robot_wheel2,


fig, ax = setup_plot()
line0, line1, line2, robot_body, robot_wheel1, robot_wheel2 = setup_plot_elements(ax)

ani = animation.FuncAnimation(fig, robot_update, frames=len(x_trajectory),
                              init_func=init, blit=True, repeat=False)
plt.show()