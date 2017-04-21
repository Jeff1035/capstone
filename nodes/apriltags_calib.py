#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from visualization_msgs.msg import MarkerArray
import transformations as tf

pub = rospy.Publisher('/apriltags/extcalib/', Transform, queue_size=10)
extcalib = None
res = None

def transform2Matrix(tr):
	global extcalib
	rotation = tr.rotation
	matrix = tf.quaternion_matrix([rotation.w,rotation.x,rotation.y,rotation.z])
	matrix[0,3] = tr.translation.x
	matrix[1,3] = tr.translation.y
	matrix[2,3] = tr.translation.z
	# print "Extrinsic Calibration Matrix: "
	# print matrix
	extcalib = matrix

def getExt():
	while extcalib is None:
		rospy.Subscriber('/kinect2/extrinsicTransform', Transform, transform2Matrix)
	print "calibration finished"

def transform_callback(mkarr):
	for m in mkarr.markers:
		matrix = tf.quaternion_matrix([m.pose.orientation.w,
										m.pose.orientation.x, 
										m.pose.orientation.y,
										m.pose.orientation.z])
		rotation_matrix = matrix[:3,:3]
		translation = np.zeros(3)
		translation[0] = m.pose.position.x
		translation[1] = m.pose.position.y
		translation[2] = m.pose.position.z
		camera_t = extcalib[:3,3]
		#print camera_t
		camera_R = extcalib[:3,:3]
		#print camera_R
		# print rotation_matrix
		# print translation
		t = camera_t + translation
		R = rotation_matrix.dot(camera_R)
		print t
		print R

def april_calib():
	print "searching for apriltags...."
	rospy.Subscriber('/apriltags/marker_array', MarkerArray, transform_callback)
	rospy.spin()

def node():
	rospy.init_node('apriltag_calib')
	getExt()
	april_calib()

if __name__=='__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass
