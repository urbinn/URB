import unittest
import numpy as np
import urbg2o

class OutlierDetection(unittest.TestCase):
  def test_local_ba(self):
    cv_keyframes = np.load('tests/fixtures/local-ba/cv_keyframes.npy')
    f_keyframes = np.load('tests/fixtures/local-ba/f_keyframes.npy')
    mappoints = np.load('tests/fixtures/local-ba/mappoints.npy')
    links = np.load('tests/fixtures/local-ba/links.npy')

    cv_keyframes = np.asfortranarray(cv_keyframes)
    f_keyframes = np.asfortranarray(f_keyframes)
    mappoints = np.asfortranarray(mappoints)
    links = np.array(links, order='f')
    
    print('keyframes', len(cv_keyframes), '\n')
    print('f_keyframes', len(f_keyframes), '\n')
    print('mappoints', len(mappoints), '\n')
     
    self.assertIsNotNone(cv_keyframes)
    self.assertIsNotNone(f_keyframes)
    self.assertIsNotNone(mappoints)
    self.assertIsNotNone(links)
    
    print('keyframes', cv_keyframes, '\n')
    
    # TODO: Check if keyframes are modified
    result =  urbg2o.outliersForLocalBundleAdjustment(cv_keyframes, f_keyframes, mappoints, links)
    print('result', result)
    self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()


