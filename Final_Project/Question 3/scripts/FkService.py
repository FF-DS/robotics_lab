#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from arm_gazebo.srv import FK, FKResponse
import numpy as np
import math
import TransformationCalculator.TransformationCalculator as TC


def findEE(req):
    ee = TC.fkTransform(req.links, req.angles)
    return FKResponse(ee)

def fkService():
    rospy.init_node('FkServiceNode', anonymous=True)
    s = rospy.Service('FkService', FK, findEE)
    print('Waiting for request....')
    rospy.spin()

if __name__ == '__main__':
    fkService()
