#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from arm_gazebo.srv import IK, IKResponse
import numpy as np
import math
import TransformationCalculator.TransformationCalculator as TC


def findAngles(req):
    angles = TC.ikTransform(req.links, req.ee)
    return IKResponse(angles)

def ikService():
    rospy.init_node('IkServiceNode', anonymous=True)
    s = rospy.Service('IkService', IK, findAngles)
    print('Waiting for request....')
    rospy.spin()

if __name__ == '__main__':
    ikService()
