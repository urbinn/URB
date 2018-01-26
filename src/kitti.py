import builtins
s = 0 if builtins.SEQUENCE < 3 else 4 if builtins.SEQUENCE > 3 else 3
builtins.SETTINGS = 'src.settings.settings_kitti' + str(s)

from src.settings.load import *
from src.sequence import *

LEFTDIR = '/data/urbinn/datasets/kitti/sequences/%02d/image_0'%builtins.SEQUENCE
RIGHTDIR = '/data/urbinn/datasets/kitti/sequences/%02d/image_1'%builtins.SEQUENCE
