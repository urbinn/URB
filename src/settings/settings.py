import cv2
import os
import importlib

# Set constants
# de breedte en hoogte van de patches die gematched worden
# een lagere patchSize resulteert in meer punten

def env_int(variable, default):
    return int(os.environ[variable]) if variable in os.environ else default

def env_float(variable, default):
    return float(os.environ[variable]) if variable in os.environ else default

globals().update( {
'SEQUENCE':0,
'NORM':cv2.NORM_L1,
'PATCH_SIZE':17,
'HALF_PATCH_SIZE':17 // 2,  
'STEREO_CONFIDENCE':1.6,
'SEQUENCE_CONFIDENCE':1.6,
'SEQUENCE':0,
'DATASET':'KITTI',
'SETTINGS_LOADED':False })

def set_sequence(sequence, dataset='KITTI'):
    global SEQUENCE, DATASET
    SEQUENCE = sequence
    DATASET = dataset
    load_settings(True)
    
def load_settings(force=False):
    global SEQUENCE, DATASET, SETTINGS_LOADED
    if not SETTINGS_LOADED or force:
        if DATASET == 'KITTI':
            if SEQUENCE < 3:
                file = 'settings_kitti0'
            elif SEQUENCE > 3:
                file = 'settings_kitti4'
            else:
                file = 'settings_kitti3'
        elif DATASET == 'ZED':
            file = 'settings_zed'
        else:
            raise ValueError('unknown dataset ' + DATASET)
        module = importlib.import_module('src.settings.' + file)
        settings = { k: v for (k, v) in module.__dict__.items() if not k.startswith('_') }
        globals().update( settings )
        SETTINGS_LOADED = True

def set_norm(norm):
    global NORM
    NORM = norm
    
def set_patch_size(patch_size):
    global PATCH_SIZE
    PATCH_SIZE = patch_size
    
def set_half_patch_size(half_patch_size):
    global HALF_PATCH_SIZE
    HALF_PATCH_SIZE = half_patch_size
    
def set_stereo_confidence(stereo_confidence):
    global STEREO_CONFIDENCE
    STEREO_CONFIDENCE = stereo_confidence
    
def set_sequence_confidence(sequence_confidence):
    global SEQUENCE_CONFIDENCE
    SEQUENCE_CONFIDENCE = sequence_confidence
    
def get_sequence_confidence():
    return SEQUENCE_CONFIDENCE

def get_stereo_confidence():
    return STEREO_CONFIDENCE

def get_patch_size():
    return PATCH_SIZE

def get_half_patch_size():
    return HALF_PATCH_SIZE

def get_norm():
    return NORM

def get_sequence():
    return SEQUENCE

def get_camera_bf():
    return CAMERA_BF

def get_camera_fx():
    return CAMERA_FX

def get_camera_fy():
    return CAMERA_FY

def get_camera_cx():
    return CAMERA_CX

def get_camera_cy():
    return CAMERA_CY

def get_camera_fx_inv():
    return 1.0 / get_camera_fx()

def get_camera_fy_inv():
    return 1.0 / get_camera_fy()

def set_leftdir_base(leftdir_base):
    global LEFTDIR_BASE
    LEFTDIR_BASE = leftdir_base

def set_rightdir_base(rightdir_base):
    global RIGHTDIR_BASE
    RIGHTDIR_BASE = rightdir_base

def get_leftdir():
    return LEFTDIR_BASE%get_sequence()

def get_rightdir():
    return RIGHTDIR_BASE%get_sequence()

