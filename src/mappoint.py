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
        
    def get_world_coords(self):
        return self.world_coords
    
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
    
