#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import os
from geometry_msgs.msg import Point
import numpy as np

rospy.init_node('point_publisher')
rospy.loginfo_once('points_visualizer started')
# create publisher
publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

cwd = os.getcwd()
obs_points = np.load(os.path.join(cwd, "data/original_pointclouds.npy"))
print(obs_points.shape)

# flatten the point array and broadcast the z values
flat_points = obs_points.reshape(-1, 2)
z_values = 0.3 * np.ones_like(flat_points[:, 0])
z_values = z_values.reshape(obs_points.shape[0], -1)
z_broadcasted = np.broadcast_to(
    z_values, (obs_points.shape[0], obs_points.shape[1]))

# # create Point objects
points = [Point(x=float(p[0]), y=float(p[1]), z=float(z))
          for p, z in zip(flat_points, z_broadcasted.flatten())]


z_values = z_values[:,:, np.newaxis]
obs_points = np.concatenate((obs_points, z_values), axis=2)
print(obs_points.shape)

points_list = []
for i in range(obs_points.shape[0]):
    flat_points = obs_points[i, :, :].reshape(-1, 3)
    # print(len(flat_points))
    points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
              for p in (flat_points)]
    points_list.append(points)

print(points_list[i])

# # Reshape the array to shape (778*200, 3)
# arr_reshaped = np.reshape(obs_points, (-1, 3))
# print(arr_reshaped.shape)

# # # Create a new PointCloud message
# # pc_msg = PointCloud()

# # Loop through the points in the reshaped array
# for point_arr in arr_reshaped:
#     print(len(point_arr))
#     # Create a new Point32 message for each point
#     point = Point()
#     point.x = point_arr[0]
#     point.y = point_arr[1]
#     point.z = point_arr[2]
#     break
    
#     # # Append the Point32 message to the PointCloud message
#     # pc_msg.points.append(point)
        
# # # Publish the PointCloud message
# # publisher.publish(pc_msg)