from src.sequence import *
from src.frame import *
import glob

class Urb:
    def __init__(self, settings, images_left, images_right, yolo_output = None):
        frames = []
        for filename in sorted(glob.glob(images_left + '/*')):
            # bounding_boxes assigning
            left_frame = Frame(filename, images_right)
            frames.append(left_frame)
        
        seq = Sequence()
        for f in ProgressBar()(frames):
            seq.add_frame(f)
             
                