import cv2
from util import get_limits
from PIL import Image # type: ignore


cap = cv2.VideoCapture(0)

yellow = [0, 255, 255]
blue = [156, 0, 0]
red = [0, 0, 255]
green = [0, 128, 0]

colorL = [yellow, blue, red, green]

while True:
    _, img = cap.read()

    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lowerL, upperL = get_limits(colorL)

    mask = cv2.inRange()
    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        img = cv2.rectangle(img, (x1, y1), (x2,y2), (0, 255, 255), 2)

    cv2.imshow('color detection', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()