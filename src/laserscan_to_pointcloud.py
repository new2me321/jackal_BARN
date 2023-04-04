#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud

rospy.init_node("client_test")
rospy.wait_for_service("assemble_scans")
rospy.loginfo("assemble_scans service started")
assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
pub = rospy.Publisher ("/pointcloud", PointCloud, queue_size=1)
r = rospy.Rate (30)
rospy.loginfo("Publishing pointclouds")
while not rospy.is_shutdown():
    resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
    pub.publish (resp.cloud)
    r.sleep()