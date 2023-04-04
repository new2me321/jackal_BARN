#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
import numpy as np
import os
import sys
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_vector3
from geometry_msgs.msg import Pose, TransformStamped, Vector3Stamped


cwd = os.getcwd()
args = sys.argv[1:]
world_no = args[0] # world number
if len(args) > 1:
    try:
        get_cylinder_poses = int(args[1])
    except:
        get_cylinder_poses = 1

    if get_cylinder_poses == 1:
        get_cylinder_poses = True
    else:
        get_cylinder_poses = False
else:
    get_cylinder_poses = False
directory = "data/" + str(world_no) + "/"
if not os.path.exists(directory):
    os.makedirs(directory)

fname = os.path.join(cwd, directory + "obstacle_poses.npy")

def cylinder_pose_callback(msg):
    global fname
    # Find the index of the cylinders in the ModelStates message
    cylinder_indices = [msg.name.index(
        'unit_cylinder_{}'.format(i)) for i in range(len(msg.name)-3)]

    # Extract the poses of the cylinders
    cylinder_poses = [msg.pose[index] for index in cylinder_indices]

    np.save(fname, cylinder_poses)


def visualize_obstacles():
    global fname

    cylinder_poses = np.load(fname, allow_pickle=True)

    # Create a ROS Marker message for each cylinder
    marker_msgs = []
    for i in range(len(cylinder_poses)):
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = 'world'
        marker_msg.id = i
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD

        # Do transformations
        r, p, y = euler_from_quaternion([marker_msg.pose.orientation.x, marker_msg.pose.orientation.y,
                                        marker_msg.pose.orientation.z, marker_msg.pose.orientation.w])
        Q = quaternion_from_euler(r, p, y-1.57)

        transform = TransformStamped()
        transform.transform.rotation.x = Q[0]
        transform.transform.rotation.y = Q[1]
        transform.transform.rotation.z = Q[2]
        transform.transform.rotation.w = Q[3]

        # We will use Vector3Stamped object for original pose
        v_orig = Vector3Stamped()
        v_orig.header.stamp = rospy.Time.now()
        v_orig.header.frame_id = 'map'
        v_orig.vector.x = cylinder_poses[i].position.x
        v_orig.vector.y = cylinder_poses[i].position.y
        v_orig.vector.z = 0.25

        # transform Vector3Stamped
        v_transformed = do_transform_vector3(v_orig, transform)

        marker_msg.pose.position = v_transformed.vector

        marker_msg.pose.position.z = 0.25
        marker_msg.pose.orientation.x = Q[0]
        marker_msg.pose.orientation.y = Q[1]
        marker_msg.pose.orientation.z = Q[2]
        marker_msg.pose.orientation.w = Q[3]

        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.5
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 1.0
        marker_msgs.append(marker_msg)

    # Publish the marker messages
    for marker_msg in marker_msgs:
        marker_pub.publish(marker_msg)


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('cylinder_marker_publisher')

    if get_cylinder_poses == True:
        rospy.loginfo_once('saving obstacles poses from Gazebo')
        # Subscribe to the Gazebo model states topic
        rospy.Subscriber('/gazebo/model_states', ModelStates, cylinder_pose_callback)
        rospy.spin()
        print("Saved!")
    else:
        rospy.loginfo_once('visualizing obstacles')
        marker_pub = rospy.Publisher('/cylinder_markers', Marker, queue_size=10)
        loop_rate = 10
        while not rospy.is_shutdown():
            visualize_obstacles()
            # rospy.sleep(1)
            # rospy.spin()
