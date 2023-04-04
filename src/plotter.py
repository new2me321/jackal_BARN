import os
import matplotlib.pyplot as plt
import numpy as np



cwd = os.getcwd()
point_clouds = np.load(os.path.join(cwd, "data/original_pointclouds.npy"))
jackal_poses = np.load(os.path.join(cwd, "data/jackal_poses.npy"))
jackal_odometry = np.load(os.path.join(cwd, "data/jackal_odometry.npy"))
robot_x = jackal_odometry[:, 0]
robot_y = jackal_odometry[:, 1]
robot_yaw = jackal_odometry[:, 2]


print(point_clouds[:, :, 0].shape)
print(jackal_poses.shape)
plt.axis([-10, 10, -5, 5])
for i in range(point_clouds.shape[0]):
    # Quit the plot 
    if len(plt.get_fignums()) == 0:
        plt.close()
        break
    
    plt.clf()
    plt.title(f"Scan {i+1}")
    plt.axis([-1, 10, -3, 3])
    plt.scatter(point_clouds[i, :, 0] + robot_x[i], point_clouds[i, :, 1] + robot_y[i], s=3, c='r', label='point_cloud')
    plt.scatter(jackal_poses[i, 0], jackal_poses[i, 1])
    plt.scatter([8], [0], s=100, c='g', marker='*', label='goal')
    dx = 0.1*np.cos(robot_yaw[i])
    dy = 0.1*np.sin(robot_yaw[i])
    plt.arrow(jackal_poses[i, 0], jackal_poses[i, 1], dx, dy, head_width=0.2, head_length=0.3, fc='lightblue',  ec='b', label=['robot', 'rp'])
    plt.legend(loc='lower left')
    plt.pause(0.01)

    if i+1 == len(point_clouds):
        plt.title(f"Goal reached!")
        plt.show()
    

