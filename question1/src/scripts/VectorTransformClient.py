#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from Lab1.srv import *


# data class
class VectorTransformation:
    def __init__(self, x, y, z, d = 0, alpha = 0, beta = 0, gamma = 0):
        self.x = x
        self.y = y
        self.z = z
        self.d = d
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

    def __str__(self):
        return "\n Positions: x = {}, y = {}, z = {} \n Angels: alpha = {}, beta = {}, gamma = {}\n Distance = {}".format( self.x, self.y, self.z, self.alpha, self.beta, self.gamma, self.d)

    def result(self):
        return "\n Positions: x' = {}, y' = {}, z' = {}".format( self.x, self.y, self.z)


# transform server
def transformVectorClient(vectorObj : VectorTransformation ):
    rospy.wait_for_service('TransformVector')
    try:
        transformVector = rospy.ServiceProxy('TransformVector', Transform)
        res = transformVector( 
                        vectorObj.x, vectorObj.y, vectorObj.z,             # positions
                        vectorObj.d,                                       # distance
                        vectorObj.alpha, vectorObj.beta, vectorObj.gamma   # angles
                    )
        return VectorTransformation(res.x_prime, res.y_prime, res.z_prime)

    except rospy.ServiceException as e:
        print(" Service call failed : {}".format(e))



# main method
def main():
    vectorObj = VectorTransformation( 
        1, 2, 3, 
        5,
        30, 40, 70
    )

    print('Input: ', vectorObj)
    result = transformVectorClient(vectorObj)
    print('Result: ', result.result())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
