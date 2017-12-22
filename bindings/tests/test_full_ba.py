import unittest
import numpy as np
import urbg2o

class FullBA(unittest.TestCase):
  def test_full_ba(self):
    cv_keyframes = np.load('tests/fixtures/local-ba/cv_keyframes.npy')
    mappoints = np.load('tests/fixtures/local-ba/mappoints.npy')
    links = np.load('tests/fixtures/local-ba/links.npy')

    cv_keyframes = np.asfortranarray(cv_keyframes)
    mappoints = np.asfortranarray(mappoints)
    links = np.array(links, order='f')
    
    print('keyframes', len(cv_keyframes), '\n')

    print('mappoints', len(mappoints), '\n')
     
    self.assertIsNotNone(cv_keyframes)

    self.assertIsNotNone(mappoints)
    self.assertIsNotNone(links)
    
    # TODO: Check if keyframes are modified
    result =  urbg2o.fulllBundleAdjustment(cv_keyframes, mappoints, links)
    self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()

