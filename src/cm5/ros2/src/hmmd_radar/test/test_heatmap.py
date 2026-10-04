import math
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from hmmd_radar.heatmap import RangeDopplerHeatmap
from hmmd_radar.view_model import RangeDopplerViewModel


class HeatmapTests(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_updates_orientation_scaling_and_stale_indicator(self):
        values = list(range(320))
        model = RangeDopplerViewModel(stale_after=1.0)
        model.update(20, 16, values, now=1.0)
        plot = RangeDopplerHeatmap(model)
        plot.render(stale=False)

        image = np.asarray(plot.image.get_array())
        self.assertEqual(image[4, 9], values[4 * 16 + 9])
        self.assertEqual(plot.image.get_clim(), (0.0, 319.0))
        self.assertFalse(plot.status_label.get_visible())

        plot.toggle_scale()
        plot.render(stale=True)
        self.assertTrue(plot.status_label.get_visible())
        self.assertEqual(plot.status_label.get_text(), "STALE DATA")
        self.assertEqual(plot.image.get_clim(), (0.0, math.log1p(319)))
        self.assertEqual(plot.colorbar.ax.get_ylabel(), "log1p(amplitude squared)")


if __name__ == "__main__":
    unittest.main()
