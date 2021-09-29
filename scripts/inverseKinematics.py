#!/usr/bin/env python

from __future__ import print_function

import roslib
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Quaternion, Pose
from homework.srv import inverseKinematics
import rospy
import rospkg 

def handleInverseKinematics(req):
    print("Input frame [%f, %f, %f, %f]"%(req.x, req.y, req.z, req.phi))
    rospack = rospkg.RosPack()

    #parse urdf file
    (status, tree) = kdl_parser.treeFromFile(rospack.get_path('homework') + "/urdf/robot.urdf")
    print("\n *** Successfully parsed urdf file and constructed kdl tree *** \n" if status else "Failed to parse urdf file to kdl tree")
    #build a chain from base to end effector
    chain = tree.getChain("base_link", "ee_link")
    #used to solve kinematics
    ik_vel = kdl.ChainIkSolverVel_pinv(chain) #needed for inverse kinematics
    fk = kdl.ChainFkSolverPos_recursive(chain) #needed for inverse kinematics
    ik = kdl.ChainIkSolverPos_NR(chain, fk, ik_vel)

    #create a frame with input data
    position = kdl.Vector(req.x,req.y,req.z);
    rotation = kdl.Rotation()
    rotation.RotZ(req.phi)
    desired_position = kdl.Frame(rotation, position)
    
    #joint angles are the current position, needed for the solver
    jointAngles = kdl.JntArray(4)
    jointAngles[0] = 0
    jointAngles[1] = 0
    jointAngles[2] = 0
    jointAngles[3] = 0
    #result joint values are in theta_out
    theta_out = kdl.JntArray(4)
    ik.CartToJnt(jointAngles, desired_position, theta_out)
   
    return round(theta_out[0], 2),round(theta_out[1], 2),round(theta_out[2], 2),round(theta_out[3], 2) 

def inverseKinematics_server():
    #init server
    rospy.init_node('inverseKinematics_server')
    s = rospy.Service('inverseKinematics', inverseKinematics, handleInverseKinematics)
    print("Ready to calculate inverse kinematics.")
    rospy.spin()

if __name__ == "__main__":
    inverseKinematics_server()