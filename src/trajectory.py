import numpy as np
import cv2
import random
from src.imageio import zero_image_from_dimension

COLORS = {

}

class Trajectory:
    def __init__(self, poses, dimensions = None, observations = None):
        self.dimensions = dimensions
        self._poses = self.transform_poses(poses)
        self._observations = observations
        self._observations_rwc = [obs.get_world_coords() for obs in self._observations]
        self._scaling = self.get_scaling([(x, z) for x, _, z, _ in self._poses])

    def transform_poses(self, poses):
        points = np.repeat(np.array([[0,0,0,1.0]]), len(poses), axis=0) 

        for i in range(1, len(poses)):
            points[i:] = np.dot(points[i:], np.linalg.pinv(poses[i].T))

        return points

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

        for i in range(1, len(self._poses)):
            coords = self.scale_coords(self._poses[i-1][0], self._poses[i-1][2], self._scaling)
            coords2 = self.scale_coords(self._poses[i][0], self._poses[i][2], self._scaling)

            cv2.line(img, (int(coords[0]), int(coords[1])), 
                (int(coords2[0]), int(coords2[1])), (0,0,0), 2)

        """TODO: Extract method"""
        if with_observations and self._observations:
            for i, obs in enumerate(self._observations):
                x, _, z, _ = self._observations_rwc[i]
                clas = self._observations[i].classification
                scaled_x, scaled_y = self.scale_coords(x, z, self._scaling)

                cv2.circle(img, (int(scaled_x), int(scaled_y)), 3, (255, 0, 0), -1)

                if with_classifications:            
                    cv2.putText(img, str(clas), (int(scaled_x), int(scaled_y)), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1, cv2.LINE_AA)

        return img

