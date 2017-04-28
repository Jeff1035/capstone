#!/usr/bin/env python
from geometry_msgs.msg import Transform
import transformations as transf
import rospy
import rospkg
import numpy as np
import cv2
import sys
import os

import yaml
def opencv_matrix(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix)

BOARD_DIM_X = 8
BOARD_DIM_Y = 6

def get_Rt(IMAGE_PATH, SERIAL_NUMBER):
	# read in the checkerboard image
	image = cv2.imread(IMAGE_PATH)
	if image is None:
		print "target image at", IMAGE_PATH, "not exist"
		exit()
	gray = cv2.imread(IMAGE_PATH,0)
	# cv2.imshow("calibration image",image)
	# cv2.waitKey()

	# Find the checkerboard corners and refine it
	ret, corners = cv2.findChessboardCorners(gray, (BOARD_DIM_X,BOARD_DIM_Y),None)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

	# If can't find the checkboard pattern, exit
	if not ret:
	    print "cannot find checkerboard with dimention ", BOARD_DIM_X, "x", BOARD_DIM_Y
	    exit()

	# visulization for detected corners
	# for pair in corners:
	#     cv2.circle(gray,(pair[0][0],pair[0][1]), 20, (0,0,255), -1)
	#     cv2.imshow('img', gray)
	#     cv2.waitKey(0)
	# cv2.drawChessboardCorners(image, (8,6), corners, ret)
	# cv2.imshow('img', image)
	# cv2.waitKey(0)

	# create 3D and 2D coordinates
	checker_dim = 0.0322
	objectPoints = np.array([[[i*checker_dim,j*checker_dim,0]] for i in range(BOARD_DIM_Y) for j in range(BOARD_DIM_X)],dtype='float32')
	imagePoints = corners#np.array([[[10*i,10*j]] for i in range(BOARD_DIM_Y) for j in range(BOARD_DIM_X)],dtype='float32')

	# read in K and distortion coefficients
	driver_path = rospkg.RosPack().get_path('kinect2_bridge')
	path = driver_path + "/data/"+ SERIAL_NUMBER + "/calib_color.yaml"
	#matrixA = np.array( cv2.cv.Load(path, cv2.cv.CreateMemStorage(), "cameraMatrix") )
	with open(path,'r') as stream:
		stream.readline()
		camera = yaml.load(stream.read())
	
	cameraMatrix = camera['cameraMatrix']
	distCoeffs = camera['distortionCoefficients']

	# extrinsic calibration
	_, rvec, tvec = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)

	# visulization of projected corners and calculate re-projection error
	error = 0
	for i in range(objectPoints.shape[0]):
		p_obj = objectPoints[i]
		p_img = cv2.projectPoints(p_obj,rvec,tvec,cameraMatrix,distCoeffs)[0][0][0]
		# cv2.circle(image,(p_img[0],p_img[1]),3,(0,0,255),-1)
		# cv2.imshow('img',image)
		# cv2.waitKey(0)
		error += cv2.norm(imagePoints[i][0], p_img, cv2.NORM_L2)
	error /= objectPoints.shape[0]
	print "re-projection error is", error

	# # draw the xyz axis
	# OXYZ = np.array([[[0,0,0]],[[5*checker_dim,0,0]],[[0,5*checker_dim,0]],[[0,0,5*checker_dim]]],dtype='float32')
	# oxyz = cv2.projectPoints(OXYZ,rvec,tvec,cameraMatrix,distCoeffs)[0]
	# cv2.line(image,(oxyz[0][0][0],oxyz[0][0][1]),(oxyz[1][0][0],oxyz[1][0][1]),(255,0,0),5)
	# cv2.line(image,(oxyz[0][0][0],oxyz[0][0][1]),(oxyz[2][0][0],oxyz[2][0][1]),(0,255,0),5)
	# cv2.line(image,(oxyz[0][0][0],oxyz[0][0][1]),(oxyz[3][0][0],oxyz[3][0][1]),(0,0,255),5)
	# cv2.imshow('img',image)
	# cv2.waitKey(0)

	# # draw some test points to visually validate the result
	# testPoints = np.array([[[i,j,k]] for i in range(0,2*BOARD_DIM_Y,2)\
	# 								 for j in range(0,2*BOARD_DIM_X,2)\
	# 							     for k in range(0,20,2)],dtype='float32')
	# for i in range(testPoints.shape[0]):
	# 	p_obj = testPoints[i]
	# 	p_img = cv2.projectPoints(p_obj,rvec,tvec,cameraMatrix,distCoeffs)[0][0][0]
	# 	cv2.circle(image,(p_img[0],p_img[1]),3,(255,0,0),-1)
	# 	cv2.imshow('img',image)
	# 	cv2.waitKey(0)
	R_obj = cv2.Rodrigues(rvec)[0]
	t_obj = tvec
	R = np.linalg.inv(R_obj)
	t = -R.dot(t_obj)
	return R,t

def tf_from_Rt(R, t):
	m = Transform()
	m.translation.x = t[0]
	m.translation.y = t[1]
	m.translation.z = t[2]
	quaternion = transf.quaternion_from_matrix(R)
	m.rotation.w = quaternion[0]
	m.rotation.x = quaternion[1]
	m.rotation.y = quaternion[2]
	m.rotation.z = quaternion[3]
	return m

def talker(m):
	pub = rospy.Publisher('/kinect2/extrinsicTransform', Transform, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		pub.publish(m)
		rate.sleep()


if __name__ == '__main__':
	# print 'Number of arguments:', len(sys.argv), 'arguments.'
	# print 'Argument List:', str(sys.argv)
	if len(sys.argv) is not 3:
		print 'Input argument error\nArg#1: name of calibration image\nArg#2: Kinect Serial number'
		exit()
	IMAGE_PREFIX = os.path.dirname(os.path.realpath(__file__))
	IMAGE_NAME = sys.argv[1]
	SERIAL_NUMBER = sys.argv[2]
	IMAGE_PATH = IMAGE_PREFIX + "/image/" + IMAGE_NAME
	R,t = get_Rt(IMAGE_PATH, SERIAL_NUMBER)
	print R
	print t
	m = tf_from_Rt(R,t)
	try:
		talker(m)
	except rospy.ROSInterruptException:
		pass