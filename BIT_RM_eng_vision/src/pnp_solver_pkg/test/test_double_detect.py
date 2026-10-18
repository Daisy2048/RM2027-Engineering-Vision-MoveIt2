import unittest

from pnp_solver_pkg.double_detect import DoubleDetectCheck, classify_right_left


def point(x, score=1.0):
    return {"x": float(x), "y": 100.0, "score": float(score)}


def pillar(center, score=1.0):
    return {
        "class_name": "pillar",
        "bbox": [center - 12, 80, center + 12, 120],
        "pose": {
            "TL": point(center - 10, score),
            "TR": point(center + 10, score),
            "BL": point(center - 10, score),
            "BR": point(center + 10, score),
        },
    }


def exchange(center, score=1.0):
    return {
        "class_name": "exchange",
        "bbox": [center - 25, 70, center + 25, 130],
        "pose": {
            "light_TL": point(center - 20, score),
            "light_TR": point(center + 20, score),
            "light_BL": point(center - 20, score),
            "light_BR": point(center + 20, score),
            "shell_L": point(center - 20, score),
            "shell_M": point(center, score),
            "shell_R": point(center + 20, score),
        },
    }


class DoubleDetectCheckTest(unittest.TestCase):
    def classify(self, pillars=(), exchanges=()):
        checker = DoubleDetectCheck(iou_threshold=0.3)
        checker._update({"pillar": list(pillars), "exchange": list(exchanges)})
        return checker, classify_right_left(checker)

    def test_empty(self):
        checker, slots = self.classify()
        self.assertEqual(slots, [{}, {}])

    def test_single_pillar_defaults_to_right(self):
        item = pillar(100)
        _, (right, left) = self.classify(pillars=[item])
        self.assertIs(right["pillar"], item)
        self.assertEqual(left, {})

    def test_single_exchange_defaults_to_right(self):
        item = exchange(100)
        _, (right, left) = self.classify(exchanges=[item])
        self.assertIs(right["exchange"], item)
        self.assertEqual(left, {})

    def test_single_pair_defaults_to_right(self):
        p = pillar(100)
        e = exchange(100)
        _, (right, left) = self.classify([p], [e])
        self.assertIs(right["pillar"], p)
        self.assertIs(right["exchange"], e)
        self.assertEqual(left, {})

    def test_unpaired_pillar_and_exchange_are_sorted(self):
        p = pillar(100)
        e = exchange(300)
        _, (right, left) = self.classify([p], [e])
        self.assertIs(right["exchange"], e)
        self.assertIs(left["pillar"], p)

    def test_pair_and_single_pillar_are_sorted(self):
        left_pillar = pillar(100)
        right_pillar = pillar(300)
        right_exchange = exchange(300)
        _, (right, left) = self.classify(
            [right_pillar, left_pillar], [right_exchange]
        )
        self.assertIs(right["pillar"], right_pillar)
        self.assertIs(right["exchange"], right_exchange)
        self.assertIs(left["pillar"], left_pillar)

    def test_pair_and_single_exchange_are_sorted(self):
        left_exchange = exchange(100)
        right_pillar = pillar(300)
        right_exchange = exchange(300)
        _, (right, left) = self.classify(
            [right_pillar], [right_exchange, left_exchange]
        )
        self.assertIs(right["pillar"], right_pillar)
        self.assertIs(right["exchange"], right_exchange)
        self.assertIs(left["exchange"], left_exchange)

    def test_two_pairs_are_sorted_independent_of_input_order(self):
        left_pillar, right_pillar = pillar(100), pillar(300)
        left_exchange, right_exchange = exchange(100), exchange(300)
        _, (right, left) = self.classify(
            [right_pillar, left_pillar], [left_exchange, right_exchange]
        )
        self.assertIs(right["pillar"], right_pillar)
        self.assertIs(right["exchange"], right_exchange)
        self.assertIs(left["pillar"], left_pillar)
        self.assertIs(left["exchange"], left_exchange)

    def test_pairing_preserves_legacy_behavior_without_score_filter(self):
        p = pillar(100, score=0.0)
        e = exchange(100)
        _, (right, left) = self.classify([p], [e])
        self.assertIs(right["pillar"], p)
        self.assertIs(right["exchange"], e)
        self.assertEqual(left, {})

    def test_more_than_two_unpaired_groups_is_ambiguous(self):
        checker, slots = self.classify(
            [pillar(100), pillar(300)], [exchange(500)]
        )
        self.assertEqual(slots, [{}, {}])


if __name__ == "__main__":
    unittest.main()
