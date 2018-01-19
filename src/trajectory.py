import numpy as np
import cv2
from src.imageio import zero_image_from_dimension

class Trajectory:
    def __init__(self, poses, dimensions = None):
        self.dimensions = dimensions
        self._poses = self.transform_poses(poses)
        self._coords = self.get_coordinates(self._poses)

    def transform_poses(self, poses):
        points = np.repeat(np.array([[0,0,0,1.0]]), len(poses), axis=0) 

        for i in range(1, len(poses)):
            points[:i] = np.dot(points[:i], poses[i].T)

        return points

    def get_coordinates(self, poses):
        coords = []
        minx = min([p[0] for p in poses])
        maxx = max([p[0] for p in poses])
        miny = min([p[2] for p in poses])
        maxy = max([p[2] for p in poses])
        maxxy = max(maxx - minx, maxy - miny) * 1.01

        for p in poses:
            x = int(self.dimensions[0]) * (p[0] - minx) / (maxxy)
            z = int(self.dimensions[0]) - int(self.dimensions[1]) * (p[2] - miny) / (maxxy)
            coords.append([x, z])

        return coords

    def draw(self):
        x, y = self.dimensions
        img = zero_image_from_dimension(x, y)

        for i in range(1, len(self._coords)):
            cv2.line(img, (int(self._coords[i - 1][0]), int(self._coords[i - 1][1])), 
                (int(self._coords[i][0]), int(self._coords[i][1])), (0,0,0), 2)
                
        return img

