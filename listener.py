#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/viki/catkin_ws/src/interactpy/src/interactpy')
import GeometryUtils
import rospy
from geometry_msgs.msg import Transform
import transformations as tf
# import required libraries
import openravepy
import numpy as np
import time

env = openravepy.Environment()
env.SetViewer('qtcoin')
#env.SetViewer('RViz')
#env.SetViewer('InteractiveMarker')

# Importing the Robot
module = openravepy.RaveCreateModule(env, 'urdf')
#name = module.SendCommand('load /home/viki/catkin_ws/src/kinova_description/urdf/jaco7dof.urdf /home/viki/catkin_ws/src/kinova_description/urdf/jaco7dof.srdf')
name = module.SendCommand('load /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/j2s7s300_standalone.urdf /home/viki/catkin_ws/src/kinova-ros/kinova_description/urdf/jaco7dof_standalonev1.srdf')



jaco = env.GetRobot(name)

pi = 3.141592654
joint_names =["j2s7s300_joint_1","j2s7s300_joint_2","j2s7s300_joint_3","j2s7s300_joint_4","j2s7s300_joint_5","j2s7s300_joint_6","j2s7s300_joint_7"]
dofs = [jaco.GetJoint(name).GetDOFIndex() for name in joint_names]
jaco.SetDOFValues([-pi/2,pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2],dofs)

# Importing objects
table = env.ReadKinBodyXMLFile('objects/table.kinbody.xml')

env.Add(table)

#bottle = env.ReadKinBodyXMLFile('objects/fuze_bottle.kinbody.xml')
#env.Add(bottle)
mug = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
env.Add(mug)


# Arm and objects Placement
Tz =openravepy.matrixFromAxisAngle([-np.pi/2,0,0])
pos = np.array([[1,0,0,0.2],[0,1,0,0.73],[0,0,1,0.1],[0,0,0,1]])

#pos = np.array([[1,0,0,0],[0,1,0,0.70],[0,0,1,0.37],[0,0,0,1]])
#pos = np.dot(pos,Tz)
jaco.SetTransform(pos)

#bottle_pos = np.array([[1,0,0,0.2],[0,1,0,0.73],[0,0,1,-0.1],[0,0,0,1]])
#bottle_pos = np.dot(bottle_pos,Tz)
#bottle.SetTransform(bottle_pos)

mug_pos = np.array([[1,0,0,0.4],[0,1,0,0.73],[0,0,1,0.1],[0,0,0,1]])
mug.SetTransform(mug_pos)

#manip = jaco.GetManipulators()[0]



#jaco.SetDOFValues(np.array([ -4.13872047e-01 , -5.01264461e-01 ,  4.02553793e-01 , -3.22636312e-01, -6.96080833e-01   ,8.54815566e-01  , 9.35005132e-02 ,  1.14352972e-14,1.99840144e-15 ,  3.33066907e-16]))
pos1 = np.array([[1,0,0,2],[0,1,0,1],[0,0,1,0],[0,0,0,1]])
pos2 = np.array([[1,0,0,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]])

#for i in range(10):
#        if i%2:
#           jaco.SetTransform(pos1)
#       else:
#            jaco.SetTransform(pos2)
#        time.sleep(1)

def transform2Matrix(tr):
    rotation = tr.rotation
    matrix = tf.quaternion_matrix([rotation.w,rotation.x,rotation.y,rotation.z])
    matrix[0,3] = tr.translation.x
    matrix[1,3] = tr.translation.y
    matrix[2,3] = tr.translation.z
    print matrix ############################
    return matrix


def callback(data): 
    #mug.SetTransform(transform2Matrix(data))
    x = data.translation.x
    y = data.translation.y
    z = data.translation.z
    jaco.SetDOFValues([pi/2,y,pi/2,x,pi/2,z,pi/2],dofs)
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', (data.data)[0])

def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', Transform, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    listener()

