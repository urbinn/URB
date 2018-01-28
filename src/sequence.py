from progressbar import ProgressBar
from src.settings.settings import *
from src.frame import *
from src.json import *
import sys
import os
import shutil
import urbg2o
import numpy as np

# returns the two best matching points in the list of keyPoints for the given query keyPoint
def matching_observation(o, observations):
    if len(observations) < 2:
         return (0, None)
    best_distance = sys.maxsize
    next_best_distance = sys.maxsize
    best_observation = None
    best_frame_point = None
    for kp in observations:
        if type(kp) is type(o):
            distance = kp.get_patch_distance(o)
            if distance < o.distance * o.confidence:
                if distance < best_distance:
                    next_best_distance = best_distance
                    best_distance = distance
                    best_frame_point = kp
                elif distance < next_best_distance:
                    next_best_distance = distance
    if best_frame_point is not None:
        #if kp.get_patch_distance(best_frame_point, xoff=-1) <= best_distance:
        #    return (1.0, None)
        #elif kp.get_patch_distance(best_frame_point, xoff=1) <= best_distance:
        #    return (1.0, None)
        if not o.satisfies_patch_contrast(best_frame_point):
            return (1.0, None)
    confidence = next_best_distance / (best_distance + 0.01)
    return (confidence, best_frame_point)

# returns the matching keyPoints in a new frame to keyPoints in a keyFrame that exceed a confidence score
def match_frame(frame, keyframeobservations, sequence_confidence = None):
    if sequence_confidence is None:
        sequence_confidence = get_sequence_confidence()
    newframematches = dict()
    for i, keyframeobs in enumerate(keyframeobservations):
        confidence, newframeobs = matching_observation(keyframeobs, frame.get_observations())
        if confidence > sequence_confidence:
            try:
                if newframematches[newframeobs][0] < confidence:
                    newframematches[newframeobs] = (confidence, keyframeobs)
                    newframeobs.set_mappoint_no_check(keyframeobs.get_mappoint())
            except:
                newframematches[newframeobs] = (confidence, keyframeobs)
                newframeobs.set_mappoint_no_check(keyframeobs.get_mappoint())
    return [ (n, k[1]) for n,k in newframematches.items() ]

def responsible_observations(matches):
    responsible = []
    non_responsible = []
    for m in matches:
        if m[0].check_mappoint():
            responsible.append(m)
        else:
            non_responsible.append(m)
    return responsible, non_responsible

def diff2(pose1, pose2):
    p = pose1 - pose2
    return sum(sum(p * p))
            
def pose_frame(frame, keyframe, sequence_confidence = None):
    keyframeobservations = keyframe.get_static_observations()
    matches = match_frame(frame, keyframeobservations, sequence_confidence = sequence_confidence)
    previous_frame = keyframe.frames[-1] if len(keyframe.frames) > 0 else keyframe
    diff = sys.maxsize
    while True:
        pose, points_left = get_pose(matches)
        frame.set_pose(pose)
        diffn = diff2(pose, previous_frame.get_pose())
        if diffn < diff:
            diff = diffn
            bestpose = pose
            bestmatches, matches = responsible_observations(matches)
            if previous_frame == keyframe and keyframe.get_previous_keyframe() is None:
                break
        else:
            break
    frame.set_pose(bestpose)
    count = len(bestmatches)
    for obs, kfobs in bestmatches:
        obs.register_mappoint()
            
    last_z = keyframe.frames[-1].get_pose()[2, 3] if len(keyframe.frames) > 0 else keyframe.get_pose()[2,3]
    last_rotation = keyframe.frames[-1].get_pose()[0, 2] if len(keyframe.frames) > 0 else keyframe.get_pose()[0,2]
    rotation = pose[0,2]
    rotation = abs(rotation - last_rotation)
    invalid_rotation = rotation > 0.2
    speed = (pose[2,3] - last_z)
    invalid_speed = speed < -4 or speed > 0
    #if invalid_rotation:
    #    print('invalid rotation', rotation, '\n', pose)
    #if invalid_speed:
    #    print('invalid speed keyframe {} frame {} speed {}\n'.format(keyframe.frameid, frame.frameid, speed), pose)
    #print(frame.frameid, '\n', bestpose)
    return invalid_speed, invalid_rotation, count
    
def create_sequence(frames, sequence_confidence=None):
    s = Sequence()
    for i, f in enumerate(ProgressBar()(frames)):
        #print('add frame ' + str(i))
        s.add_frame(f, sequence_confidence = sequence_confidence );
    return s

class Sequence:
    def __init__(self):
        self.mappointcount = 0
        self.keyframecount = 0
        self.keyframes = []
        self.rotation = 0
        self.speed = 0
       
    def add_keyframe(self, frame, sequence_confidence = None, run_ba=True):
        if len(self.keyframes) == 0:
            frame.set_pose( np.eye(4, dtype=np.float64) )
            self._add_keyframe(frame, run_ba=run_ba)
        else:
            pose_frame(frame, self.keyframes[-1], sequence_confidence = sequence_confidence)
            #keyframe = self.keyframes[-1]
            #matches = match_frame(frame, keyframe.get_observations(), sequence_confidence = sequence_confidence)
            #pose, points_left = get_pose(matches)
            #print(points_left, len(matches), pose)
            #frame.set_pose(pose)
            self._add_keyframe( frame, run_ba=run_ba )                     

    def add_frame(self, frame, sequence_confidence = None, clean=False, run_ba=True):
        #print('add_frame')
        if len(self.keyframes) == 0:
            frame.set_pose( np.eye(4, dtype=np.float64) )
            self._add_keyframe(frame, run_ba=run_ba)
        else:
            keyframe = self.keyframes[-1]
            invalid_speed, invalid_rotation, points_left = pose_frame(frame, keyframe, sequence_confidence=sequence_confidence)

            #print(len(matches), points_left, frame.get_pose())
            # make the former frame into a keyframe
            if invalid_rotation or points_left < 30 :         
                keyframe = keyframe.frames.pop()
                self._add_keyframe( keyframe, run_ba=run_ba )
                #keyframeobservations = list(keyframeobservations)
                #obsnotvisiblelastkeyframe = [o for o in self.keyframes[-2].get_observations() if o.mappoint.last_observation == o]
                #keyframeobservations.extend( obsnotvisiblelastkeyframe )
                
                for obs in frame.get_observations():
                    obs.set_mappoint_no_check(None)
                invalid_speed, invalid_rotation, points_left = pose_frame(frame, keyframe, sequence_confidence=sequence_confidence)
                print('noot', invalid_speed, invalid_rotation, points_left)

            frame.keyframe = keyframe
            keyframe.frames.append(frame)
                
    def _run_local_ba(self, frame):
        cv_keyframes = get_covisible_keyframes(frame)
        mappoints = get_mappoints(cv_keyframes)
        f_keyframes = [] #get_fixed_keyframes(mappoints, cv_keyframes)[:0]

        cv_keyframes_np = keyframes_to_np(cv_keyframes)
        f_keyframes_np = keyframes_to_np(f_keyframes)
        mappoints_np = mappoints_to_np(mappoints)
        links_np = links_to_np(mappoints)

        f_keyframes = np.asfortranarray(f_keyframes_np)
        cv_keyframes_BA = np.asfortranarray(cv_keyframes_np)
        mappoints = np.asfortranarray(mappoints_np)
        links = np.array(links_np, order='f')

        results =  urbg2o.localBundleAdjustment(cv_keyframes_BA, f_keyframes, mappoints, links)

        for i in range(2, len(cv_keyframes_BA)):
            cv_keyframes[i].set_world_pose( cv_keyframes_BA[i, 2:].reshape((4,4)) )
             
    def _add_keyframe(self, frame, run_ba=True):
        frame.set_previous_keyframe(None if len(self.keyframes) == 0 else self.keyframes[-1])
        frame.compute_depth()
        frame.filter_has_depth()
        #frame.filter_most_confident()
        count = 0
        for obs in frame.get_observations():
            # obs matches previous keyframe
            if obs.has_mappoint():
                obs.check_mappoint()
                obs.register_mappoint()
                if obs.has_mappoint():
                    count += 1
            if not obs.has_mappoint() and obs.get_depth() is not None:
                obs.create_mappoint(self.mappointcount)
                self.mappointcount += 1
        print('connected mappoints ', count)
        frame.keyframeid = self.keyframecount
        self.keyframecount += 1
        self.keyframes.append(frame)
        frame.frames = []
        if len(self.keyframes) > 2 and run_ba:
            pass
            #self._run_local_ba(frame)
            
    def dump(self, folder):
        if os.path.exists(folder):
            shutil.rmtree(folder)
        os.mkdir(folder)
        #save_keyframepoints(folder + '/0.txt', self.get_keyframepoints())
        for i, frame in enumerate(self.frames):
            save_framepoints(folder + '/' + str(i+1) + '.txt', frame.get_observations())

# load a file with stored keyframeposes, returns keyframeid, frameid, poses
# if the frameid is None, its an old file that does not contain the frameid
# the poses are flattened 4x4 matrices
def load_keyframes(file):
    keyframes = np.load(file)
    keyframeids = keyframes[:, 0]
    if keyframes.shape[1] == 17: # old version, only has keyframe_ids
        poses = keyframes[:, 1:]
        return keyframeids, None, poses
    else: # new version, also has frame_ids
        frameids = keyframes[:, 1]
        poses = keyframes[:, 2:]
        return keyframeids, frameids, poses
            
# return a set of frames that share mappoints with the given keyframe
def get_covisible_keyframes(keyframe):
    mappoints = [ o.get_mappoint() for o in keyframe.get_observations() ]
    frames = { o.get_frame() for m in mappoints for o in m.get_observations() }
    return sorted(frames, key=lambda o: o.keyframeid)

# return a set of all mappoint that are visible in the given set of keyframes
def get_mappoints(keyframes):
    return { o.get_mappoint() for kf in keyframes for o in kf.get_observations() 
            if o.get_mappoint() is not None and len(o.get_mappoint().observations) > 1}

# return a set of keyframes that share mappoints with one or more of the covisible_keyframes
# that are not in the set of covisible keyframes
def get_fixed_keyframes(mappoints, covisible_keyframes):
    frames = { o.get_frame() for m in mappoints for o in m.get_observations() }
    return sorted(frames - set(covisible_keyframes), key=lambda o: o.keyframeid)

# save the keyframe poses to file
# (keyframe_id, frame_id, 4x4 pose)
def keyframes_to_np(keyframes):
    return np.hstack([ [ [ kf.keyframeid] for kf in keyframes ], [ [kf.frameid] for kf in keyframes ], [ kf.get_cumulative_world_pose().flatten() for kf in keyframes ] ])

def keyframes_pose_to_np(keyframes):
    return np.hstack([ [ [ kf.keyframeid] for kf in keyframes ], [ [kf.frameid] for kf in keyframes ], [ kf.get_pose().flatten() for kf in keyframes ] ])

# save the world coordinates of mappoints to file
# (mappoint_id, x, y, z, 1)
def mappoints_to_np(mappoints):
    kfp = np.hstack([ [ [m.id] for m in mappoints ], [ m.get_world_coords() for m in mappoints ] ])
    return kfp

# save the edges between keyframes and mappoints
# (mappoint_id, keyframe_id, pixel_x, pixel_y)
# where pixel_x and pixel_y are the screen coordinates of the mappoint in the keyframe
def links_to_np(mappoints):
    links = [ (m.id, o.get_frame().keyframeid, o.cx, o.cy) for m in mappoints for o in m.get_observations()]
    return np.array(links, dtype=np.float64, order='f')

