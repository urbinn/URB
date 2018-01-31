from src.settings.load import *
from src.coords import *
from src.imageio import *

class MapPoint:
    def __init__(self, obs):
        self.id = None
        self.world_coords = obs.get_world_coords()
        self.observations = { obs }
        self.last_observation = obs
        self.static = True
        self.cumulative_world_pose = None
        
    def get_world_coords(self):
        return self.world_coords
    
    def get_cumulative_world_pose(self):
        if self.cumulative_world_pose is None: 
            self.cumulative_world_pose = np.dot(self.last_observation.frame.get_cumulative_world_pose(), self.last_observation.get_affine_coords() ) 
        return self.cumulative_world_pose
    
    def update_world_coords(self, obs):
        if obs.get_depth() is not None:
            self.world_coords = obs.get_world_coords()
            self.last_observation = obs
    
    def update_world_coords_ba(self, coords):
        self.world_coords = coords
        
    def get_last_observation(self):
        return self.last_observation
    
    def add_observation(self, observation):
        self.observations.add(observation)
        
    def remove_observation(self, observation):
        self.observations.remove(observation)
        
    def get_observations(self):
        return self.observations
    