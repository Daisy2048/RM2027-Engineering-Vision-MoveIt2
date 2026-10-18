import unittest

from pnp_solver_pkg.keypoint_mapping import (
    POINT_VISIBILITY_ORDER,
    build_visibility_mask,
)


def detection(class_name, visible_names):
    names = {
        name
        for expected_class, name in POINT_VISIBILITY_ORDER
        if expected_class == class_name
    }
    return {
        "class_name": class_name,
        "pose": {
            name: {"x": 1.0, "y": 2.0, "score": 1.0 if name in visible_names else 0.0}
            for name in names
        },
    }


class VisibilityMaskTest(unittest.TestCase):
    def test_empty_side_is_zero(self):
        self.assertEqual(build_visibility_mask({}), 0)

    def test_pillar_only_uses_bits_zero_to_three(self):
        group = {"pillar": [detection("pillar", {"TL", "BR"})], "exchange": []}
        self.assertEqual(build_visibility_mask(group), (1 << 0) | (1 << 3))

    def test_exchange_only_uses_bits_four_to_ten(self):
        group = {
            "pillar": [],
            "exchange": [detection("exchange", {"shell_M", "light_BR"})],
        }
        self.assertEqual(build_visibility_mask(group), (1 << 5) | (1 << 10))

    def test_complete_target_has_all_eleven_bits(self):
        pillar_names = {
            name for class_name, name in POINT_VISIBILITY_ORDER if class_name == "pillar"
        }
        exchange_names = {
            name
            for class_name, name in POINT_VISIBILITY_ORDER
            if class_name == "exchange"
        }
        group = {
            "pillar": [detection("pillar", pillar_names)],
            "exchange": [detection("exchange", exchange_names)],
        }
        self.assertEqual(build_visibility_mask(group), 0x07FF)


if __name__ == "__main__":
    unittest.main()
