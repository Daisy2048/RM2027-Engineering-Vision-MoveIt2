import unittest
import uuid

import numpy as np

from shared_image_pkg.shared_frame import HEADER_SIZE, SharedFrameReader, SharedFrameWriter


class SharedFrameTest(unittest.TestCase):
    def setUp(self):
        self.name = "roi_test_" + uuid.uuid4().hex
        self.writer = SharedFrameWriter(64 * 48 * 3, self.name)
        # 测试读写器在同一进程，不能注销进程级 resource_tracker。
        self.reader = SharedFrameReader(self.name, unregister_tracker=False)

    def tearDown(self):
        self.reader.close()
        self.writer.close()

    def test_round_trip(self):
        image = np.arange(64 * 48 * 3, dtype=np.uint8).reshape(48, 64, 3)
        sequence = self.writer.write(image, 123456789, "camera", "bgr8")
        frame = self.reader.read_latest()
        self.assertEqual(sequence, 2)
        self.assertEqual(frame.sequence, sequence)
        self.assertEqual(frame.timestamp_ns, 123456789)
        self.assertEqual(frame.frame_id, "camera")
        self.assertEqual(frame.encoding, "bgr8")
        np.testing.assert_array_equal(frame.image, image)

    def test_reader_snapshot_is_not_overwritten(self):
        first = np.zeros((48, 64, 3), dtype=np.uint8)
        second = np.full((48, 64, 3), 255, dtype=np.uint8)
        self.writer.write(first, 1)
        snapshot = self.reader.read_latest().image
        self.writer.write(second, 2)
        self.assertTrue(np.all(snapshot == 0))

    def test_rejects_oversized_frame(self):
        with self.assertRaises(ValueError):
            self.writer.write(np.zeros((49, 64, 3), dtype=np.uint8), 1)

    def test_reader_reconnects_after_writer_restart(self):
        self.writer.write(np.zeros((48, 64, 3), dtype=np.uint8), 1)
        self.assertEqual(self.reader.read_latest().sequence, 2)
        self.writer.close()
        self.writer = SharedFrameWriter(64 * 48 * 3, self.name)
        self.writer.write(np.ones((48, 64, 3), dtype=np.uint8), 2)
        self.assertTrue(self.reader.reconnect())
        frame = self.reader.read_latest()
        self.assertEqual(frame.sequence, 2)
        self.assertTrue(np.all(frame.image == 1))

    def test_header_is_cache_line_multiple(self):
        self.assertEqual(HEADER_SIZE % 32, 0)


if __name__ == "__main__":
    unittest.main()
