import numpy as np
import cv2


def get_limits(color):
    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]

    hue_lower = max(0, hue - 10)

    hue_upper = min(179, hue + 10)

    lowerL = np.array([hue_lower, 100, 100], dtype=np.uint8)
    upperL = np.array([hue_upper, 255, 255], dtype=np.uint8)

    return lowerL, upperL