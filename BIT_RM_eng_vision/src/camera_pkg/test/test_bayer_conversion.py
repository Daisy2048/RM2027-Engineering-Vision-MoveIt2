import unittest

import cv2
import numpy as np

from camera_pkg.daheng_camera import BAYER_MODES


class BayerConversionTest(unittest.TestCase):
    def test_rggb_converts_to_bgr(self):
        raw = np.empty((8, 8), dtype=np.uint8)
        raw[0::2, 0::2] = 200  # R
        raw[0::2, 1::2] = 100  # G
        raw[1::2, 0::2] = 100  # G
        raw[1::2, 1::2] = 10   # B

        converted = cv2.cvtColor(raw, BAYER_MODES[0][1])
        self.assertEqual(converted[3, 3].tolist(), [10, 100, 200])


if __name__ == "__main__":
    unittest.main()
