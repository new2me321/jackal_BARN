#!/usr/bin/env python3

import rospy 
# import sensor_msgs.point_cloud2 as point_cloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection

def scan_callback(scan_msg):
    lp = LaserProjection()

    point_cloud = lp.projectLaser(scan_msg)

    # rospy.loginfo_throttle(1, point_cloud)
    pub.publish(point_cloud)


rospy.init_node("laserscan_to_pointcloud")
rospy.loginfo("laserscan_to_pointcloud initialized")
pub = rospy.Publisher("/pointcloud", PointCloud2, queue_size=1000)
rospy.Subscriber("/scan", LaserScan, scan_callback)

rospy.spin()
rospy.signal_shutdown(rospy.loginfo("Shutting down"))
