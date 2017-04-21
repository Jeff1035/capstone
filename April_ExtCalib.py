#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from visualization_msgs.msg import MarkerArray

pub = rospy.Publisher('/apriltags/extcalib/', Transform)
global res = None

def callback(april_coord, ext_calib):
	pass

def node():
	rospy.init_node('apriltag_calib')
	april_coord = message_filters.Subscriber('/apriltags/detections', MarkerArray)
	ext_calib = message_filters.Subscriber('/kinect2/extrinsicTransform', Transform)
	ts = message_filters.TimeSynchronizer([april_coord, ext_calib], 10)
	tr.registerCallback(callback)
	
	rospy.loginfo(res)
	pub.publish(res)
	rospy.spin()

if __name__=='__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
