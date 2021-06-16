import numpy as np
import cv2 as cv

kelly = cv.imread("kelly.png")
newKelly = np.zeros_like(kelly)

filter = np.array([
    [-1, 0, 1],
    [-2, 1, 2],
    [-1, 1, 1]
], dtype = np.int)

filter = filter / 9 
k = 3

for i in range(kelly.shape[0] - 2):
    for j in range(kelly.shape[1] - 2):
        patch = kelly[i : i + k, j : j + k]

        Gx = ( filter * patch ).sum()
        Gy = ( filter.T * patch ).sum()

        G = np.sqrt( Gx ** 2 +  Gy ** 2 )
        newKelly[i, j] = G


cv.imshow("Model.Y" , newKelly)
cv.waitKey(0)