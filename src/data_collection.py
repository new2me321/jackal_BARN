#!/usr/bin/env python3
import time
import numpy as np
import rospy
import rospkg
import tf2_ros
import tf
import threading
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import PointCloud
import message_filters
from nav_msgs.msg import Odometry

import open3d
import os

cwd = os.getcwd()

is_received = False

obstacle_points_vehicle = None

min_dis_points_vehicle = None

pointcloud_mutex = threading.Lock()
odom_mutex = threading.Lock()
publish_traj_mutex = threading.Lock()

num_laser_points = 720
num_down_sampled = 200

x_obs_pointcloud_vehicle = np.ones((num_laser_points, 1)) * 100
y_obs_pointcloud_vehicle = np.ones((num_laser_points, 1)) * 100

x_obs_down_sampled = np.ones((num_down_sampled, 1)) * 100
y_obs_down_sampled = np.ones((num_down_sampled, 1)) * 100
xyz = np.random.rand(720, 3)


jackal_velocities = []
closet_pointclouds_used_for_planning_vehicle = []
original_pointclouds = []
downsampled_points = []
jackal_poses = []
jackal_orientation = []
jackal_odometry = []


def odomCallback(odom_msg, pointcloud):

    global is_received, odom_mutex, obstacle_points_vehicle, jackal_poses

    tf_listener = buffer.lookup_transform(odom_msg.header.frame_id, pointcloud.header.frame_id, rospy.Time(), rospy.Duration(1.0))
    trans = [tf_listener.transform.translation.x, tf_listener.transform.translation.y, tf_listener.transform.translation.z]
    rot = [tf_listener.transform.rotation.x, tf_listener.transform.rotation.y, tf_listener.transform.rotation.z, tf_listener.transform.rotation.w]
    transformation_matrix = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

    print("transformation", tf_listener)

    odom_mutex.acquire()

    jackal_q = odom_msg.pose.pose.orientation
    jackal_list = [jackal_q.x, jackal_q.y, jackal_q.z, jackal_q.w]
    (jackal_roll, jackal_pitch, jackal_yaw) = euler_from_quaternion(jackal_list)

    # getting original pointclouds data
    msg_len = len(pointcloud.points)
    pointcloud_mutex.acquire()
    increment_value = 1
    inner_counter = 0

    for nn in range(0, msg_len, increment_value):
        x_obs_pointcloud_vehicle[inner_counter] = pointcloud.points[nn].x
        y_obs_pointcloud_vehicle[inner_counter] = pointcloud.points[nn].y
        inner_counter += 1
    
    
    xyz[:, 0] = x_obs_pointcloud_vehicle.flatten()
    xyz[:, 1] = y_obs_pointcloud_vehicle.flatten()
    xyz[:, 2] = 1

    # now transform pointclouds
    xyz_transformed = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
    xyz_transformed = np.dot(transformation_matrix, xyz_transformed.T).T[:, :3]
    print("Transform done")
    
    # downsample transformed pointclouds
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(xyz_transformed)
    downpcd = pcd.voxel_down_sample(voxel_size=0.9)
    downpcd_array = np.asarray(downpcd.points)

    num_down_sampled_points = downpcd_array[:, 0].shape[0]
    x_obs_down_sampled = np.ones((200, 1)) * 1000
    y_obs_down_sampled = np.ones((200, 1)) * 1000
    x_obs_down_sampled[0:num_down_sampled_points, 0] = downpcd_array[:, 0]
    y_obs_down_sampled[0:num_down_sampled_points, 0] = downpcd_array[:, 1]
    obstacle_points_vehicle = np.hstack(
        (x_obs_down_sampled, y_obs_down_sampled))
    pointcloud_mutex.release()
    inner_counter = 0

    idxes = np.argwhere(obstacle_points_vehicle[:, :] >= 150)
    obstacle_points_vehicle[idxes, 0] = obstacle_points_vehicle[20, 0]
    obstacle_points_vehicle[idxes, 1] = obstacle_points_vehicle[20, 1]
    obstacle_points_vehicle[:, 0] = obstacle_points_vehicle[:,
                                                            0] - odom_msg.pose.pose.position.x
    obstacle_points_vehicle[:, 1] = obstacle_points_vehicle[:,
                                                            1] - odom_msg.pose.pose.position.y
    print(f"obs {obstacle_points_vehicle.shape}")

    # apply transformation to the point cloud

    # Gather data
    downsampled_points.append(obstacle_points_vehicle)

    jackal_poses.append([odom_msg.pose.pose.position.x,
                        odom_msg.pose.pose.position.y])
    jackal_odometry.append(
        [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, jackal_yaw])
    odom_mutex.release()


if __name__ == "__main__":

    rospy.init_node('data_collection')
    rospy.loginfo("data collection initialized!")
    rospack = rospkg.RosPack()
    
    buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(buffer)
    
    jackal_pointcloud_sub = message_filters.Subscriber(
        '/pointcloud', PointCloud)
    jackal_odom_sub = message_filters.Subscriber(
        '/odometry/filtered', Odometry)

    ts = message_filters.ApproximateTimeSynchronizer(
        [jackal_odom_sub, jackal_pointcloud_sub], 1, 1, allow_headerless=True)
    ts.registerCallback(odomCallback)

    rospy.spin()
    # print("\n",jackal_poses)
    np.save(os.path.join(cwd, "data/jackal_odometry.npy"), jackal_odometry)
    np.save(os.path.join(cwd, "data/jackal_poses.npy"), jackal_poses)
    np.save(os.path.join(cwd, "data/closet_pointclouds_used_for_planning_vehicle.npy"),
            closet_pointclouds_used_for_planning_vehicle)
    np.save(os.path.join(cwd, "data/original_pointclouds.npy"), downsampled_points)
    print(np.array(downsampled_points).shape)
    print("Files were saved")

    rospy.signal_shutdown(rospy.loginfo_once("Node is shutting down"))
