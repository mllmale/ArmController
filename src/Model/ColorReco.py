from PIL import Image
from ..util.util import get_limits
import numpy as np
import cv2

class ColorReco:
    def __init__(self):
        # Definindo as cores como atributos da classe
        self.yellow = [0, 255, 255]
        self.blue = [156, 0, 0]
        self.red = [0, 0, 255]
        self.green = [0, 128, 0]
        self.colors = [self.yellow, self.blue, self.red, self.green]

    def process_image(self, img):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        centers = []

        for color in self.colors:
            lowerL, upperL = self.get_limits(color)
            mask = cv2.inRange(hsvImg, lowerL, upperL)
            mask_ = Image.fromarray(mask)
            bbox = mask_.getbbox()

            if bbox is not None:
                x1, y1, x2, y2 = bbox
                img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                centers.append((center_x, center_y))

        return img, centers

cap = cv2.VideoCapture(0)
detector = ColorReco()

while True:
    _, img = cap.read()
    processed_img, centers = detector.process_image(img)

    cv2.imshow('color detection', processed_img)
    print("Posições dos centros dos retângulos:", centers)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
