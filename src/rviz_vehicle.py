#!/usr/bin/env python3

import rospy
import os
import numpy as np
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud


cwd = os.getcwd()
jackal_odometry = np.load(os.path.join(cwd, "data/jackal_odometry.npy"))
obs_points = np.load(os.path.join(cwd, "data/original_pointclouds.npy"))

rospy.init_node('robot_marker_publisher')

robot_marker_pub = rospy.Publisher('robot_visualizer', Marker, queue_size=1)
pcl_pub = rospy.Publisher('my_point_cloud_topic', PointCloud, queue_size=10)

loop_rate = 10  # in hertz
x_offset = 3  # -2
y_offset = 2

# jackal pose.position
robot_x = jackal_odometry[:, 0]
robot_y = jackal_odometry[:, 1]
robot_yaw = jackal_odometry[:, 2]

# Prepare pointcloud data
flat_points = obs_points.reshape(-1, 2)
z_values = 0.3 * np.ones_like(flat_points[:, 0])
z_values = z_values.reshape(obs_points.shape[0], -1)
z_values = z_values[:, :, np.newaxis]
obs_points = np.concatenate((obs_points, z_values), axis=2)

# pre generate pointscloud points message
points_list = []
for i in range(obs_points.shape[0]):
    flat_points = obs_points[i, :, :].reshape(-1, 3)
    # print(len(flat_points))
    points = [Point(x=float(p[0] + robot_x[i] + x_offset), y=float(p[1] + robot_y[i] + y_offset), z=float(p[2]))
              for p in (flat_points)]
    points_list.append(points)

rospy.loginfo_once('robot visualizer started')
while not rospy.is_shutdown():

    for i in range(jackal_odometry.shape[0]):
        if rospy.is_shutdown():
            break

        # Pointcloud
        point_cloud_msg = PointCloud()
        point_cloud_msg.header.stamp = rospy.Time.now()
        point_cloud_msg.header.frame_id = 'world'
        point_cloud_msg.points = points_list[i]

        # Robot
        marker_msg = Marker()
        marker_msg.header.frame_id = 'world'
        marker_msg.ns = 'jackal_robot'
        marker_msg.id = 0
        marker_msg.type = Marker.CUBE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = robot_x[i] + x_offset
        marker_msg.pose.position.y = robot_y[i] + y_offset
        marker_msg.pose.position.z = 0+0.250/2

        #  get quartenion orientation
        Q = quaternion_from_euler(0, 0, robot_yaw[i])
        marker_msg.pose.orientation.x = Q[0]
        marker_msg.pose.orientation.y = Q[1]
        marker_msg.pose.orientation.z = Q[2]
        marker_msg.pose.orientation.w = Q[3]

        # jackal robot dimensions
        marker_msg.scale.x = 0.508
        marker_msg.scale.y = 0.430
        marker_msg.scale.z = 0.250

        # color - yellow
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0

        # Publish
        robot_marker_pub.publish(marker_msg)
        pcl_pub.publish(point_cloud_msg)

        rospy.sleep(1/loop_rate)
