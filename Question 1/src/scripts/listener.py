#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from Lab1.srv import Transform, TransformResponse
import numpy as np
import math

def transformGivenVector(req):
    rospy.loginfo(rospy.get_caller_id() + 'processing incoming request...')

    # vector
    vector = np.array([
            [req.x],
            [req.y],
            [req.z],
        ])

    # rotations
    alpha, beta, gamma = math.pi * req.alpha/ 180,  math.pi * req.beta/ 180, math.pi * req.gamma/ 180 
    
    x_axis = np.array([
            [1, 0, 0],
            [0, math.cos(alpha), -math.sin(alpha)],
            [0, math.sin(alpha), math.cos(alpha)],
        ])

    y_axis = np.array([
            [math.cos(beta), 0, math.sin(beta)],
            [0, 1, 0],
            [-math.sin(beta), 0, math.cos(beta)],
        ])

    z_axis = np.array([
            [math.cos(gamma), -math.sin(gamma), 0],
            [math.sin(gamma), math.cos(gamma), 0],
            [0, 0, 1],
        ])

    # result
    result = x_axis.dot(y_axis).dot(z_axis).dot(vector)
    print('Result : ', result)
    return TransformResponse(result[0] + req.distance, result[1] + req.distance, result[2] + req.distance)


def listener():
    rospy.init_node('TransformVectorServer', anonymous=True)
    s = rospy.Service('TransformVector', Transform, transformGivenVector)
    rospy.spin()


if __name__ == '__main__':
    listener()
