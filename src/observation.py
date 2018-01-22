from src.settings.load import *
from src.coords import *
from src.imageio import *
from src.mappoint import *
import sys

class Observation:
    def __init__(self, frame, x, y):
        self.mappoint = None
        self.cx = x
        self.cy = y
        self.frame = frame
        self.leftx = int(x) - HALF_PATCH_SIZE
        self.topy =int(y) - HALF_PATCH_SIZE  
        self.keypoint = cv2.KeyPoint(x, y, 1, 0)
        self.z = None
        self.disparity = None
    
    def get_frame(self):
        return self.frame
    
    def set_mappoint_no_check(self, mappoint):
        self.mappoint = mappoint

    def is_static(self):
        return self.has_mappoint() and self.mappoint.static
        
    # checks if the mappoint projects back to close to similar x,y coordinates on the screen
    # if not, the observation is no longer assigned to the mappoint
    def check_mappoint(self):
        last_observation = self.mappoint.get_last_observation()
        affine_coords = np.dot( self.frame.get_pose(), last_observation.get_affine_coords() )  
        cam_coords = affine_coords_to_cam( affine_coords )
        #print(self.cx, self.cy, cam_coords[0], cam_coords[1])
        if abs(cam_coords[0] - self.cx) > 2 or abs(cam_coords[1] - self.cy) > 2:
            self.mappoint.static = False
            
    def check_mappoint_old(self):
        last_observation = self.mappoint.get_last_observation()
        affine_coords = np.dot( self.frame.get_world_pose(), last_observation.get_world_coords() )
        cam_coords = affine_coords_to_cam( affine_coords )
        #print(self.cx, self.cy, cam_coords[0], cam_coords[1])
        if abs(cam_coords[0] - self.cx) > 2 or abs(cam_coords[1] - self.cy) > 2:
            self.mappoint = None
            
    def register_mappoint(self):
        if self.has_mappoint():
            self.mappoint.add_observation(self)
            self.mappoint.update_world_coords(self)
            
    def get_mappoint(self):
        return self.mappoint
    
    def get_mappoint_id(self):
        try:
            return self.mappoint.id
        except:
            return None
    
    def has_mappoint(self):
        return self.mappoint is not None

    def create_mappoint(self, id):
        self.mappoint = MapPoint(self)
        self.mappoint.id = id
    
    def get_patch(self):
        try:
            return self.patch
        except:
            self.patch = get_patch(self.get_frame().get_smoothed(), self.leftx, self.topy)
            self.latestpatch = self.patch
            return self.patch
        
    def get_patch_distance(self, keypoint, xoff=0):
        if xoff == 0:
            return cv2.norm(self.get_patch(), keypoint.get_patch(), NORM)
        else:
            return cv2.norm(get_patch(self.get_frame().get_smoothed(), self.leftx+xoff, self.topy), keypoint.get_patch(), NORM)
    
    def get_disparity(self, frameRight):
        if self.disparity is None:
            self.confidence, self.distance, self.disparity = patch_disparity(self, frameRight)
        return self.disparity
                                                    
    def get_depth(self):
        if self.z is None and self.disparity is not None:
            self.z = estimated_distance(self.disparity)
        return self.z
    
    def get_patch_mean(self):
        return self.get_patch().flatten().mean()
    
    def get_patch_threshold(self):
        try:
            return self._patch_threshold
        except:
            self._patch_threshold = np.abs(np.array(self.get_patch(), dtype=np.int16) - 
                               np.ones(self.get_patch().shape, dtype=np.int16) * self.get_patch_mean()).std() / 1.5
        return self._patch_threshold
    
    def satisfies_patch_contrast(self, observation):
        contrast = np.abs(np.array(self.get_patch(), dtype=np.int16) - 
                         np.array(observation.get_patch(), dtype=np.int16)).std()
        return contrast < min( self.get_patch_threshold(), observation.get_patch_threshold() )
    
    def get_world_coords(self):
        try:
            return self.world_coords
        except:
            coords = self.get_affine_coords()
            self.world_coords = np.dot( self.frame.get_inv_world_pose(), coords)
            return self.world_coords
    
    def get_affine_coords(self):
        try:
            return self._affine_coords
        except:
            self._affine_coords = cam_to_affine_coords(self.cx, self.cy, self.get_depth())
            return self._affine_coords
    
    def get_keypoint(self):
        return self.keypoint
        
# OpenCV reverses coordinates, so the observation on top of an edge has a smaller y coordinate than the bottom of the same vertical edge
class ObservationTopRight(Observation):
    def __init__(self, frame, x, y):
        Observation.__init__(self, frame, x, y)
        self.topy = self.cy - 1
        self.leftx = self.cx - 1

class ObservationTopLeft(Observation):
    def __init__(self, frame, x, y):
        Observation.__init__(self, frame, x, y)
        self.topy = self.cy - 1
        self.leftx = self.cx - PATCH_SIZE + 1
        
class ObservationBottomLeft(Observation):
    def __init__(self, frame, x, y):
        Observation.__init__(self, frame, x, y)
        self.topy = self.cy - PATCH_SIZE + 1
        self.leftx = self.cx - PATCH_SIZE + 1
  
class ObservationBottomRight(Observation):
    def __init__(self, frame, x, y):
        Observation.__init__(self, frame, x, y)
        self.topy = self.cy - PATCH_SIZE + 1
        self.leftx = self.cx - 1
