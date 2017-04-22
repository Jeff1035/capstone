#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from visualization_msgs.msg import MarkerArray, Marker
import transformations as tf

pub = rospy.Publisher('/apriltags/extcalib/', MarkerArray, queue_size=10)
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
	ret_array = MarkerArray()
	print "Detected {0} AprilTags".format(len(mkarr.markers))

	for m in mkarr.markers:
		marker = Marker()
		matrix = tf.quaternion_matrix([m.pose.orientation.w,
										m.pose.orientation.x, 
										m.pose.orientation.y,
										m.pose.orientation.z])
		rotation_matrix = matrix[:3,:3]
		translation = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
		t = extcalib[:3, 3] + translation
		R = extcalib[:3, :3].dot(camera_R)
		marker.pose.position.x = t[0]
		marker.pose.position.y = t[1]
		marker.pose.position.z = t[2]
		#TODO: add the correct orientatoins for each pose
		ret_array.markers.append(marker)

	pub.publish(ret_array)

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
