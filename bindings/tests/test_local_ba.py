import unittest
import numpy as np
import urbg2o

class LocalBA(unittest.TestCase):
  def test_local_ba(self):
    cv_keyframes = np.load('tests/fixtures/local-ba/cv_keyframes.npy')
    f_keyframes = np.load('tests/fixtures/local-ba/f_keyframes.npy')
    mappoints = np.load('tests/fixtures/local-ba/mappoints.npy')
    links = np.load('tests/fixtures/local-ba/links.npy')

    cv_keyframes = np.asfortranarray(cv_keyframes)
    mappoints = np.asfortranarray(mappoints)
    links = np.array(links, order='f')
    
    self.assertIsNotNone(cv_keyframes)
    self.assertIsNotNone(f_keyframes)
    self.assertIsNotNone(mappoints)
    self.assertIsNotNone(links)
    
    # TODO: Check if keyframes are modified
    result =  urbg2o.localBundleAdjustment(cv_keyframes, f_keyframes, mappoints, links)
    self.assertIsNotNone(result)

if __name__ == '__main__':
    unittest.main()


