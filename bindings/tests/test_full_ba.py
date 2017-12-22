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
    print('start full ba')
    result =  urbg2o.fullBundleAdjustment(cv_keyframes, mappoints, links)
    print('end full ba')
    self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()

