import numpy as np
import math
import tinyik as ik


def changeToRad(val, degree):
    return math.pi * val / 180 if degree else val

def changeToDegree(val):
    return 180 * val / math.pi  

def rotationX(rad):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(rad), -np.sin(rad), 0],
        [0, np.sin(rad), np.cos(rad), 0],
        [0, 0, 0, 1],
    ])

def rotationY(rad):
    return np.array([
        [np.cos(rad), 0, np.sin(rad), 0],
        [0, 1, 0, 0],
        [-np.sin(rad), 0, np.cos(rad), 0],
        [0, 0, 0, 1],
    ])

def rotationZ(rad):
    return np.array([
        [np.cos(rad), -np.sin(rad), 0, 0],
        [np.sin(rad), np.cos(rad), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])

def translate(x = 0, y = 0, z = 0):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1],
    ])


# ---------------- main functions -------------- #
def fkTransform(links, joints, degree = False):
    M1 = translate( 0, 0, links[0] ).dot( rotationZ( changeToRad( joints[0], degree ) ) )
    M2 = translate( 0, 0, links[1] ).dot( rotationX( changeToRad( joints[1], degree ) ) )
    M3 = translate( 0, 0, links[2] ).dot( rotationX( changeToRad( joints[2], degree ) ) )
    M4 = translate( 0, 0, links[3] ).dot( rotationX( changeToRad( joints[3], degree ) ) )
    M5 = translate( 0, 0, links[4] ).dot( rotationY( changeToRad( joints[4], degree ) ) )
    M6 = translate( 0, 0, links[5] ).dot( rotationZ( changeToRad( joints[5], degree ) ) )
    M7 = translate( 0, 0, links[6] )

    result = M1.dot(M2).dot(M3).dot(M4).dot(M5).dot(M6).dot(M7)
    return result[0, 3], result[1,3], result[2, 3]


def ikTransform(links, ee):
    arm = ik.Actuator([
        [0.0, 0.0, links[0]],
        "z", [0, 0, links[1]],
        "x", [0, 0, links[2]],
        "x", [0, 0, links[3]],
        "x", [0, 0, links[4]],
        "y", [0, 0, links[5]],
        "z", [0, 0, links[6]],
    ])
    arm.ee = ee
    return arm.angles



## ------------- test code ----------------------- ##

# links = [0.1, 0.05, 2, 1, .5, .2, 0.05]
# # angles = [30, 40, 10, 20, -30, 20]

# # # fkresult = fkTransform( links, angles, True)
# # print( fkresult )
# ikresult = ikTransform(links, [2,2,2] ) 
# print( ikresult, fkTransform(links, ikresult, False)  )