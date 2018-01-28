from src.settings.settings import *

# bf is baseline * focal distance om de diepte te berekenen met formule focal distance * baseline / disparity
###ZED
CAMERA_BF=0.12 * 700.726
CAMERA_FX =  700.726
CAMERA_FY =  700.726
CAMERA_CX = 679.831
CAMERA_CY = 370.07

CAMERA_FX_INV = 1.0 / CAMERA_FX;
CAMERA_FY_INV = 1.0 / CAMERA_FY;

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