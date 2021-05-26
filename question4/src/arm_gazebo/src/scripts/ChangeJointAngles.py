#!/usr/bin/env python3

import rospy
from arm_gazebo.msg import *

def changeJointAngle():
    pub = rospy.Publisher('UpdateJointAngles', JointAngles, queue_size=1000)
    rospy.init_node('JointAnglesPublisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        jointAngles = JointAngles(
            float(input("Angle one :")),
            float(input("Angle Trwo :")),
            float(input("Angle Three :")),
            float(input("Angle Four :"))
        )
        print("-------------------------------")
        rospy.loginfo(jointAngles)
        pub.publish(jointAngles)
        rate.sleep()

if __name__ == '__main__':
    try:
        changeJointAngle()
    except rospy.ROSInterruptException:
        pass