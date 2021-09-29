#!/usr/bin/env python

from __future__ import print_function

import roslib
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Quaternion, Pose
from homework.srv import directKinematics
import rospy
import rospkg 

def handleDirectKinematics(req):
    print("Input joints [%f, %f, %f, %f]"%(req.j1, req.j2, req.j3, req.j4))
    rospack = rospkg.RosPack()
    
    #parse urdf file
    (status, tree) = kdl_parser.treeFromFile(rospack.get_path('homework') + "/urdf/robot.urdf")
    print("\n *** Successfully parsed urdf file and constructed kdl tree *** \n" if status else "Failed to parse urdf file to kdl tree")
    #build a chain from base to end effector
    chain = tree.getChain("base_link", "ee_link")
    #used to solve kinematics
    fk = kdl.ChainFkSolverPos_recursive(chain)
    #create a joint array with inputs
    jointAngles = kdl.JntArray(4)
    jointAngles[0] = req.j1
    jointAngles[1] = req.j2
    jointAngles[2] = req.j3
    jointAngles[3] = req.j4
    finalFrame = kdl.Frame()
    #transform joint values in a frame of end effector
    fk.JntToCart(jointAngles, finalFrame) 
    #build pose message type from the extracted frame
    fk_pose = Pose()
    fk_pose.position.x = finalFrame.p.x()
    fk_pose.position.y = finalFrame.p.y()
    fk_pose.position.z = finalFrame.p.z()
    x,y,z,w = finalFrame.M.GetQuaternion()
    fk_pose.orientation.x = x
    fk_pose.orientation.y = y
    fk_pose.orientation.z = z
    fk_pose.orientation.w = w
    return fk_pose 

def directKinematics_server():
    #init server
    rospy.init_node('directKinematics_server')
    s = rospy.Service('directKinematics', directKinematics, handleDirectKinematics)
    print("Ready to calculate directKinematics.")
    rospy.spin()

if __name__ == "__main__":
    directKinematics_server()