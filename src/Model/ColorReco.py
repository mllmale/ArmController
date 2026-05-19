from utils.limits import get_limits
import numpy as np
import cv2


class ColorReco:
    def __init__(self):
        self.colors_dict = {
            "Amarelo": [0, 255, 255],
            "Azul": [255, 0, 0],
            "Vermelho": [0, 0, 255],
            "Verde": [0, 255, 0]
        }

    def process_image(self, img):
        hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        centers = []

        for name, color_value in self.colors_dict.items():

            if name == "Vermelho":
                mask1 = cv2.inRange(hsvImg, np.array([0, 100, 100]), np.array([10, 255, 255]))
                mask2 = cv2.inRange(hsvImg, np.array([170, 100, 100]), np.array([179, 255, 255]))
                mask = mask1 | mask2
            else:
                lowerL, upperL = get_limits(color_value)
                mask = cv2.inRange(hsvImg, lowerL, upperL)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(img, (x, y), (x + w, y + h), color_value, 2)
                    cv2.putText(img, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_value, 2)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    cv2.circle(img, (center_x, center_y), 5, (255, 255, 255), -1)
                    centers.append((name, center_x, center_y))

        return img, centers


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    detector = ColorReco()

    while True:
        ret, img = cap.read()
        if not ret:
            print("Câmera não encontrada.")
            break

        processed_img, centers = detector.process_image(img)
        cv2.imshow('Deteccao de Cores para o Robo', processed_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()