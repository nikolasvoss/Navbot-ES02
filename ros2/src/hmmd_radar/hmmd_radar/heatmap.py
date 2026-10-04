import numpy as np
import matplotlib.pyplot as plt


class RangeDopplerHeatmap:
    def __init__(self, model):
        self.model = model
        self.figure, self.axes = plt.subplots()
        self.image = self.axes.imshow(
            np.zeros((20, 16)), origin="lower", aspect="auto", interpolation="nearest", vmin=0, vmax=1
        )
        self.colorbar = self.figure.colorbar(self.image, ax=self.axes)
        self.axes.set_xlabel("Range gate index")
        self.axes.set_ylabel("Doppler bin index")
        self.axes.set_xticks(range(16))
        self.axes.set_yticks(range(20))
        self.status_label = self.axes.text(
            0.5,
            0.5,
            "WAITING FOR DATA",
            transform=self.axes.transAxes,
            ha="center",
            va="center",
            color="white",
            fontsize=14,
            fontweight="bold",
            bbox={"facecolor": "black", "alpha": 0.75},
        )
        self.logarithmic = False

    def render(self, stale: bool):
        matrix = np.asarray(self.model.display_matrix(self.logarithmic), dtype=float)
        low = float(np.min(matrix))
        high = float(np.max(matrix))
        if high <= low:
            high = low + 1.0
        self.image.set_data(matrix)
        self.image.set_clim(low, high)
        self.colorbar.set_label("log1p(amplitude squared)" if self.logarithmic else "Amplitude squared")
        if stale:
            self.status_label.set_text("STALE DATA" if self.model.received_at is not None else "WAITING FOR DATA")
            self.status_label.set_visible(True)
        else:
            self.status_label.set_visible(False)
        self.figure.canvas.draw_idle()

    def toggle_scale(self):
        self.logarithmic = not self.logarithmic
