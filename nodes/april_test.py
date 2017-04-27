#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
import transformations as tf
from openravepy import *
import time

env = Environment()
env.SetViewer('qtcoin')
table = env.ReadKinBodyXMLFile('objects/table.kinbody.xml')
env.Add(table)


mug = env.ReadKinBodyXMLFile('data/box1.kinbody.xml')
env.Add(mug)


#add in robot
module = RaveCreateModule(env, 'urdf')
name = module.SendCommand('load /home/tyler/catkin_ws/src/kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf /home/tyler/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco7dof_standalonev1.srdf')
jaco = env.GetRobot(name)

# Arm and objects Placement
Tz = matrixFromAxisAngle([-np.pi/2,0,0])
pos = np.array([[1,0,0,0],[0,1,0,0.70],[0,0,1,0.37],[0,0,0,1]])
pos = np.dot(pos,Tz)
jaco.SetTransform(pos)

jaco.SetDOFValues(np.array([ -4.13872047e-01 , -5.01264461e-01 ,  4.02553793e-01 , -3.22636312e-01, -6.96080833e-01   ,8.54815566e-01  , 9.35005132e-02 ,  1.14352972e-14,
   1.99840144e-15 ,  3.33066907e-16]
))


def callback(mkarr):
	for m in mkarr.markers:
		print m.pose
		or_w = m.pose.orientation.w
		or_x = m.pose.orientation.x
		or_y = m.pose.orientation.y
		or_z = m.pose.orientation.z
		matrix = tf.quaternion_matrix([or_w, or_x, or_y, or_z])
		matrix[0,3] = m.pose.position.x
		matrix[1,3] = m.pose.position.y
		matrix[2,3] = m.pose.position.z
		# matrix[0:2, 3] = 0
		print matrix

		pos = np.array(matrix)
		mug.SetTransform(pos)

def main():
	# initiate the openrave simulation
	rospy.init_node('april_sim')
	rospy.Subscriber('/apriltags/extcalib/', MarkerArray, callback)
	rospy.spin()


if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
