import json
import cv2

# """
# Builds a dictionary from the YOLO output.
# """
# class ClassificationBuilder:
#     def __init__(self, yolo_output):
#         try:
#             yolo = json.load(open(yolo_output))
#         except ValueError, e:
#             print("Invalid YOLO output given to ClassificationBuilder: {}".format(e))
#         else:
#             self.classifications = self.format_output(yolo)

#     def format_output(json):
#         classified_objects = []
#         image_name = json[0]

#         for classification in json:
#             # Filter out image name
#             if (isinstance(classification, list)):
#                 classified_objects.append(BoundingBox(classification))

#         return (image_name, classified_objects)

class BoundingBox:
    """
    Creates a bounding box from a YOLO classification.
    """
    def __init__(self, classification):
        name, confidence, coords = classification
        cx, cy, width, height = coords
        
        self.object_name = name
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height
        self.calculate_bounds()

    def calculate_bounds(self):
        #         min,                            max
        self.y = (int(self.cy - self.height / 2), int(self.cy + self.height / 2))
        self.x = (int(self.cx - self.width / 2), int(self.cx + self.width / 2))
    
    def contains(self, point):
        return (self.x[0] <= point.cx <= self.x[0] + self.width and
            self.y[0] <= point.cy <= self.y[0] + self.height)
    
    def draw(self, img, show_classifier = False, text_color=(0, 255, 0), color=(255, 0, 0)):
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.rectangle(img, (self.x[0], self.y[0]), (self.x[1], self.y[1]), color)   
        
        if show_classifier:
            cv2.putText(img, self.object_name, (self.x[0], self.y[0]), font, 0.4, text_color, 0)
            
    def debug(self):
        print("object: {} cx: {} cy: {} width: {} height: {}")