import sys
import os
import cv2

from src.urb import *

def init(calibration_file, images_left, images_right, yolo = None):
    if (yolo):
        print("with yolo")
    else:
        # Initialize without yolo output
        urb = Urb(calibration_file, images_left, images_right)
        urb.start()

# Entrypoint
if __name__ == "__main__":
    if (len(sys.argv) >= 4):
        calibration_file, images_left, images_right = sys.argv[1:4]
        yolo = None

        if (len(sys.argv) == 5):
            yolo = sys.argv[4]

        init(calibration_file, images_left, images_right, yolo)
    else:
        print("expected 4 arguments: calibration_file, images_left, images_right and optionally yolo_output")
