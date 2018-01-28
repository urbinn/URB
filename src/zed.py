import builtins
from src.settings.load import *
from src.sequence import *

set_sequence(0, 'ZED')

# TODO
def get_leftdir():
    return '/data/urbinn/datasets/zed/left/'%str(get_sequence())

def get_rightdir():
    return '/data/urbinn/datasets/zed/right/'%str(get_sequence())

