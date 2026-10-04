import math
import unittest

from hmmd_radar.view_model import RangeDopplerViewModel


class RangeDopplerViewModelTests(unittest.TestCase):
    def test_matrix_orientation_and_raw_values_are_preserved(self):
        values = [0] * 320
        values[3 * 16 + 11] = 0xFFFFFFFF
        model = RangeDopplerViewModel()
        model.update(20, 16, values, now=10)

        self.assertEqual(model.matrix[3][11], 0xFFFFFFFF)
        self.assertEqual(model.matrix[0][0], 0)
        self.assertEqual(model.display_matrix()[3][11], 0xFFFFFFFF)

    def test_log1p_changes_display_copy_only_and_freshness_uses_monotonic_receipt(self):
        values = [0] * 320
        values[35] = 99
        model = RangeDopplerViewModel(stale_after=2.0)
        model.update(20, 16, values, now=5.0)

        self.assertEqual(model.display_matrix(True)[2][3], math.log1p(99))
        self.assertEqual(model.matrix[2][3], 99)
        self.assertFalse(model.is_stale(now=7.0))
        self.assertTrue(model.is_stale(now=7.01))

    def test_rejects_dimensions_count_and_non_uint32_values(self):
        model = RangeDopplerViewModel()
        with self.assertRaises(ValueError):
            model.update(16, 20, [0] * 320)
        with self.assertRaises(ValueError):
            model.update(20, 16, [0] * 319)
        for bad in (True, -1, 1 << 32, 1.5):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                model.update(20, 16, [bad] + [0] * 319)

    def test_stale_before_first_receipt(self):
        self.assertTrue(RangeDopplerViewModel().is_stale(now=0.0))


if __name__ == "__main__":
    unittest.main()
