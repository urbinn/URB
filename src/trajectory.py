import numpy as np
import cv2
import random
from src.imageio import zero_image_from_dimension
import matplotlib.pyplot as plt

LEGEND = {
    'Window': (255, 0, 0),
    'Cyclist': (0, 255, 0),
    'Car': (0, 0, 255),
    'Van': (255, 100, 0),
    'Traffic_lights': (255, 255, 0),
    'Lamppost': (255, 255, 100),
    'Pedestrian': (100, 255, 0),
    'Door': (50, 50, 255),
    'Fence': (255, 50, 50),
    'Pole': (50, 255, 50),
    'Forbidden_to_stand_still': (0, 0, 0),
    'Guidance_beacon_right': (0, 100, 0)
}

class Trajectory:
    def __init__(self, poses, dimensions = None, observations_per_frame = None):
        self.dimensions = dimensions
        self._poses = self.transform_poses(poses)
        self._observations_per_frame = observations_per_frame
        self._observations_rwc = []

        for i, observations in enumerate(self._observations_per_frame):
            pose = self._poses[i]            
            for obs in observations:
                self._observations_rwc.append((np.dot(obs.get_affine_coords(), pose), obs.classification))

        self._points = [np.dot(np.array([0,0,0,1.0]), pose) for pose in self._poses]
        self._scaling = self.get_scaling([(x, z) for x, _, z, _ in self._points])

    def transform_poses(self, poses):
        worldposes = np.array([np.eye(4)] * len(poses))

        for i in range(len(poses)-1, 0, -1):
                worldposes[i:] = np.dot(worldposes[i:], np.linalg.pinv(poses[i].T))

        return worldposes

    """Converts 3n vector to fit in given dimensions"""
    def get_scaling(self, coords):
        minx = min([p[0] for p in coords]) -30
        maxx = max([p[0] for p in coords]) +30
        miny = min([p[1] for p in coords]) -30
        maxy = max([p[1] for p in coords]) +30
        maxxy = max(maxx - minx, maxy - miny) * 1.01
        return (minx, maxx, miny, maxy, maxxy)
        
    def scale_coords(self, x, y, scaling):
        minx, maxx, miny, maxxy, maxxy = scaling
        x = int(self.dimensions[0]) * (x - minx) / (maxxy)
        y = int(self.dimensions[0]) - int(self.dimensions[1]) * (y - miny) / (maxxy)
        return x, y

    def draw(self, with_observations = False, with_classifications = False):
        x, y = self.dimensions
        img = zero_image_from_dimension(x, y)

        for i in range(1, len(self._points)):
            coords = self.scale_coords(self._points[i-1][0], self._points[i-1][2], self._scaling)
            coords2 = self.scale_coords(self._points[i][0], self._points[i][2], self._scaling)

            cv2.line(img, (int(coords[0]), int(coords[1])), 
                (int(coords2[0]), int(coords2[1])), (0,0,0), 2)

        """TODO: Extract method"""
        if with_observations and self._observations_per_frame:
            for i, obs in enumerate(self._observations_rwc):
                coord, clas = obs
                x, _, z, _ = coord
                scaled_x, scaled_y = self.scale_coords(x, z, self._scaling)

                cv2.circle(img, (int(scaled_x), int(scaled_y)), 3, LEGEND[clas], -1)

                if with_classifications:            
                    cv2.putText(img, str(clas), (int(scaled_x), int(scaled_y)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1, cv2.LINE_AA)

        return img

