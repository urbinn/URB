import json

def format_yolo_output(yolo_json_file):
    """ 
    Formats the classifications that YOLO has found in a list within a list
    In case of one frame, input might look like this:

        [
            "image_name",
            [
                "object_name",
                confidence,
                [
                    141.9303436279297,
                    151.1185302734375,
                    24.717220306396484,
                    319.1596984863281
                ]
            ],
            [
                "object_name",
                confidence,
                [
                    418.4037780761719,
                    182.15049743652344,
                    13.399748802185059,
                    67.7059326171875
                ]
            ]
        ]

    Which results in an output of:

        [image_name] = [
            BoundingBox,
            BoundingBox
        ]
    """
    bounding_boxes_per_frame = {}

    with open(yolo_json_file) as yolo_json:
        yolo_data = json.load(yolo_json)

        for index, image in enumerate(yolo_data):
            image_name = image[0]

            if image_name not in bounding_boxes_per_frame:
                bounding_boxes_per_frame[image_name] = []

            for classification in image:
                # Filter out image name
                if (isinstance(classification, list)):
                    bounding_boxes_per_frame[image_name].append(BoundingBox(classification))
    
    return bounding_boxes_per_frame

class BoundingBox:
    def __init__(self, classification):
        name, confidence, coords = classification
        cx, cy, width, height = coords
        
        self.confidence = confidence
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