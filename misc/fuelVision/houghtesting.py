import cv2 as cv
import numpy as np
from google.colab.patches import cv2_imshow
import os
import os.path as path

for img in os.listdir('images'):
    src = cv.imread(path.join('images', img))

    src = cv.resize(src, (1920 // 2, 1080 // 2))

    hsv: np.ndarray = cv.cvtColor(src, cv.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    # Edge detect on the masked areas based on the original image with a lot of sensitivity
    edges = cv.Canny(mask, 50, 150, apertureSize=3)
    
    circles = cv.HoughCircles(
        edges,
        cv.HOUGH_GRADIENT_ALT,
        dp=1,
        minDist=20,
        param1=50,
        param2=0.6,
        minRadius=5,
        maxRadius=500
    )

    print(f"Found {0 if circles is None else len(circles[0])} circles in {img}")


    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv.circle(src, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv.circle(src, (i[0], i[1]), 2, (0, 0, 255), 3)

    cv2_imshow(edges)
