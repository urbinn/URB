# Create a set of filters that detect pixels that mark the top or bottom of a vertical edge in the image.
# These filters are used in Frame.getFramePoints()
# WARNING: OpenCV reversed coordinates (row 0 is the top row) meaning that a point near the top of the image has a lower row number!

import numpy as np
import cv2

SOBELV = np.array([-1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1]).reshape((5,3))
SOBELH = np.array([-1, -1, -1, 0, 0, 0, 1, 1, 1]).reshape((3,3))
# this 3x3 filters identify tge top/bottom of binary edges
TOPV = np.array([0, 0, 0, -1.0, 1.0, -1.0, -1.0, -1.0, -1.0]).reshape((3,3))
BOTTOMV = np.array([-1.0, -1.0, -1.0, -1.0, 1.0, -1.0, 0, 0, 0]).reshape((3,3))

#TOPV = np.array([0, 0.5, 0, -1.0, 0.5, -1.0, -1.0, -1.0, -1.0]).reshape((3,3))
#BOTTOMV = np.array([-1.0, -1.0, -1.0, -1.0, 0.5, -1.0, 0, 0.5, 0]).reshape((3,3))


def filter_image(img, kernel):
    return cv2.filter2D(img, -1, kernel)

# filter image absolute if you do not care about direction
def filter_image_abs(img, kernel, threshold):
    img = filter_image(img, kernel)
    img = np.abs(img)
    if threshold is not None:
        _, img = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    return img

def sobelv(img, threshold = None):
    return filter_image_abs(img, SOBELV, threshold)

def sobelh(img, threshold = None):
    return filter_image_abs(img, SOBELV, threshold)

def top_vertical_edge(img):
    return filter_image(img, TOPV)

def bottom_vertical_edge(img):
    return filter_image(img, BOTTOMV)