import sys
from src.settings.settings import *
import numpy as np

def cam_to_affine_coords(u, v, z):
    return np.array([(u-get_camera_cx()) * z * get_camera_fx_inv(), (v-get_camera_cy()) * z * get_camera_fy_inv(), z, 1.0])

def affine_coords_to_cam(coords):
    z = coords[2]
    x = coords[0] * get_camera_fx() / z + get_camera_cx()
    y = coords[1] * get_camera_fy() / z + get_camera_cy()
    return np.array([x,y,z, 1.0], dtype=np.float64)

def estimated_distance(disparity):
    return - get_camera_bf() / disparity

# Estimates the subpixel disparity based on a parabola fitting of the three points around the minimum.
def subpixel_disparity(disparity , coords):
    try:
        subdisparity =  (coords[0] - coords[2]) / (2.0 * (coords[0] + coords[2] - 2.0 * coords[1]))
        return -max(disparity + subdisparity, 0.001)
    except:
        return -disparity - 1
    
#Compute the distance for a patch in the left hand image by computing the disparity of the patch 
# in the right hand image. A low confidence (near 1) indicates there is another location where the 
# patch also fits, and therefore the depth estimate may be wrong. 
# The bestDistance is returned along with its nearest neighbors to facilitate subpixel disparity estimation.
def patch_disparity(obs, frame_right):
    frame_left = obs.get_frame()
    if obs.cy < get_patch_size() or obs.cy > frame_left.get_height() - get_patch_size() or \
        obs.cx < get_patch_size() or obs.cx > frame_left.get_width() - get_patch_size():
            return None, None, None
    best_disparity = 0
    best_distance = sys.maxsize
    next_best_distance = sys.maxsize
    last_distance = sys.maxsize
    descending = True
    distances = []
    patchL = obs.get_patch()
    for disparity in range(0, obs.leftx):
        patchR = frame_right.get_image()[obs.topy:obs.topy+get_patch_size(), 
                                                               obs.leftx-disparity:obs.leftx+get_patch_size()-disparity]
        #print(patchL.shape, patchR.shape,leftxstart,patchSize, disparity)
        distance = cv2.norm(patchL, patchR, get_norm())
        distances.append(distance)
        if descending:
            if len(distances) > 0 and distance > last_distance:
                if last_distance < best_distance:
                    next_best_distance = best_distance    
                    best_distance = last_distance
                    best_disparity = disparity - 1
                elif last_distance < next_best_distance:
                    next_best_distance = last_distance 
                descending = False
        elif distance < last_distance:
            descending = True      
        last_distance = distance
        
    # bepaal minimale distance op disparities meer dan 1 pixel van lokale optimum 
    #minrest = sys.maxsize
    #if best_disparity > 1:
    #    minrest = min(distances[0:best_disparity-1])
    #if best_disparity < obs.leftx - HALF_PATCH_SIZE - 2:
    #    minrest = min([minrest, min(distances[best_disparity+2:])])
        
    # de disparity schatting is onbetrouwbaar als die dicht bij 1 komt
    # gebruik hier als threshold 1.4 om punten eruit te filteren waarvoor we
    # geen betrouwbare disparity estimates kunnen maken
    confidence = next_best_distance / (best_distance + 0.001)
    
    # Geef de beste disparity op pixel niveau terug, met de twee neighbors om subpixel disparity uit te rekenen
    if best_disparity == 0:
        disparity = subpixel_disparity(best_disparity, [best_distance, distances[1], distances[2]])
    elif best_disparity == obs.leftx -1:
        return 1.0, best_distance, disparity
        #disparity = subpixel_disparity(best_disparity, [distances[best_disparity-2], distances[best_disparity-1], best_distance])
    else:
        disparity = subpixel_disparity(best_disparity, [distances[best_disparity-1], best_distance, distances[best_disparity+1]])
    return confidence, best_distance, disparity
