from src.settings.load import *
from src.coords import *
from src.imageio import *

class MapPoint:
    def __init__(self, obs):
        self.id = None
        self.z = obs.get_depth()
        self._affine_coords = obs.get_affine_coords()
        self.world_coords = obs.get_world_coords()
        self.observations = { obs }
    
    def get_affine_coords(self):
        return self._affine_coords
    
    def get_world_coords(self):
        return self.world_coords
    
    def update_world_coords(self, obs):
        if obs.get_depth() is not None and obs.z < self.z:
            self.z = obs.z
            self.world_coords = obs.get_world_coords()
    
    def add_observation(self, observation):
        self.observations.add(observation)
        
    def remove_observation(self, observation):
        self.observations.remove(observation)
        
    def get_observations(self):
        return self.observations
    