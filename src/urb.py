from src.sequence import *
from src.frame import *

import glob

class Urb:
    frames = []
    calibration = None
    sequence = None
    config_mapper = None

    def __init__(self, calibration_file, images_left, images_right, yolo_output = None):
        self.load_calibration_file(calibration_file)
        self.retrieve_frames(images_left, images_right)

    def retrieve_frames(self, images_left, images_right):
        # Load all frames
        for filename in sorted(glob.glob(images_left + '/*')):
            left_frame = Frame(filename, images_right)
            self.frames.append(left_frame)

    def start(self):
        self.sequence = Sequence()
        self.sequence.initialise(self.frames)

    def load_calibration_file(self, file):
        try:
            calibration = json.load(open(file))
        except ValueError as e:
            print("Invalid calibration file given: {}".format(e))
        else:
            self.calibration = calibration
             
                