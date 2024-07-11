import numpy as np
import cv2


def get_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    lowerL = hsvC[0][0][0] - 10, 100, 100
    upperL = hsvC[0][0][0] + 10, 255, 255

    lowerL = np.array(lowerL, dtype=np.uint8)
    upperL = np.array(upperL, dtype=np.uint8)

    return lowerL, upperL