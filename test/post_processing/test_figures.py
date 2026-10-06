"""Tests for reporting figures.

Contents:
    EncodePngDataUriTests: PNG encoder round-trip.
    EmptyStateCardHtmlTests: message escaping.
    DecimatedIndicesTests: display decimation.
    WheelSmallMultiplesDecimationTests: public API.
    ImageSliderFigureTests: slider construction.

Frames encode as compressed PNG data URIs rather than
raw pixel arrays; decimation affects display only.
"""

from __future__ import annotations

import base64
import struct
import sys
import unittest
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "post_processing")
)

from python_tools.data.models import ImageFrame
from python_tools.reporting import figures


def _decode_png_data_uri(data_uri: str) -> np.ndarray:
    """Decode a PNG data URI back to RGB."""
    png_bytes = base64.b64decode(data_uri.split(",", 1)[1])
    assert png_bytes[:8] == b"\x89PNG\r\n\x1a\n"
    offset = 8
    chunks: dict[bytes, bytes] = {}
    while offset < len(png_bytes):
        (length,) = struct.unpack(
            ">I", png_bytes[offset : offset + 4]
        )
        chunk_type = png_bytes[offset + 4 : offset + 8]
        chunks[chunk_type] = png_bytes[
            offset + 8 : offset + 8 + length
        ]
        offset += 8 + length + 4
    width, height = struct.unpack(">II", chunks[b"IHDR"][:8])
    raw = zlib.decompress(chunks[b"IDAT"])
    stride = 1 + width * 3
    decoded = np.zeros((height, width, 3), dtype=np.uint8)
    for row in range(height):
        row_bytes = raw[row * stride : (row + 1) * stride]
        decoded[row] = np.frombuffer(
            row_bytes[1:], dtype=np.uint8
        ).reshape(width, 3)
    return decoded


class EncodePngDataUriTests(unittest.TestCase):
    """Tests for PNG data-URI encoding."""

    def test_round_trips_exactly(self) -> None:
        """Decoded PNG matches the original pixels."""
        rng = np.random.default_rng(7)
        image = rng.integers(0, 256, size=(12, 20, 3), dtype=np.uint8)
        decoded = _decode_png_data_uri(
            figures._encode_png_data_uri(image)
        )
        np.testing.assert_array_equal(decoded, image)

    def test_much_smaller_than_json_pixel_array_would_be(
        self,
    ) -> None:
        """Smooth image encodes well under half raw size."""
        # A smooth gradient stands in for a real frame.
        x = np.linspace(0, 255, 64, dtype=np.uint8)
        smooth_image = np.tile(x[np.newaxis, :, np.newaxis], (64, 1, 3))
        data_uri = figures._encode_png_data_uri(smooth_image)
        encoded_bytes = base64.b64decode(data_uri.split(",", 1)[1])
        self.assertLess(len(encoded_bytes), smooth_image.nbytes // 2)


class EmptyStateCardHtmlTests(unittest.TestCase):
    """Tests for empty-state card escaping."""

    def test_escapes_html_significant_characters(self) -> None:
        """Markup in the message is escaped."""
        message = "<script>alert(1)</script> & 'quoted' \"double\" < >"
        card_html = figures.empty_state_card_html(message)
        self.assertNotIn("<script>alert(1)</script>", card_html)
        self.assertIn("&lt;script&gt;", card_html)
        self.assertIn("&amp;", card_html)
        self.assertIn("&#x27;quoted&#x27;", card_html)
        self.assertIn("&quot;double&quot;", card_html)


class DecimatedIndicesTests(unittest.TestCase):
    """Tests for display decimation."""

    def test_inputs_already_below_the_limit_are_untouched(
        self,
    ) -> None:
        """Short series returns every index."""
        times_s = np.linspace(0, 1, 50)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=2000
        )
        np.testing.assert_array_equal(indices, np.arange(50))

    def test_preserves_first_and_last_sample(self) -> None:
        """First and last samples always survive."""
        times_s = np.linspace(0, 100, 100_000)
        magnitude = np.sin(times_s)
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=200
        )
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 99_999)

    def test_result_respects_the_bound_for_monotonic_data(
        self,
    ) -> None:
        """Monotonic series decimates near the bound."""
        times_s = np.linspace(0, 100, 100_000)
        magnitude = times_s.copy()
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=200
        )
        self.assertLessEqual(len(indices), 210)
        self.assertGreater(len(indices), 50)

    def test_preserves_a_sharp_spike_a_naive_every_nth_sample_would_miss(
        self,
    ) -> None:
        """Single-sample spike is still selected."""
        sample_count = 10_000
        times_s = np.linspace(0, 100, sample_count)
        magnitude = np.zeros(sample_count)
        spike_index = 3333
        magnitude[spike_index] = 1000.0
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=100
        )
        self.assertIn(spike_index, indices)
        naive_every_nth = set(
            range(0, sample_count, sample_count // 100)
        )
        self.assertNotIn(spike_index, naive_every_nth)

    def test_segments_are_decimated_independently_across_a_gap(
        self,
    ) -> None:
        """Segments keep their own endpoints."""
        first_segment = np.linspace(0, 1, 5_000)
        second_segment = np.linspace(1000, 1001, 5_000)
        times_s = np.concatenate([first_segment, second_segment])
        magnitude = np.abs(np.sin(times_s))
        indices = set(
            figures._decimated_indices(
                times_s, magnitude, maximum_points=100
            ).tolist()
        )
        self.assertIn(0, indices)
        self.assertIn(4_999, indices)
        self.assertIn(5_000, indices)
        self.assertIn(9_999, indices)

    def _fragmented_series(
        self, burst_count: int, burst_length: int = 5
    ):
        """Build many short gapped bursts."""
        times = []
        t = 0.0
        for _ in range(burst_count):
            for _ in range(burst_length):
                times.append(t)
                t += 0.01
            t += 5.0
        return np.array(times)

    def test_many_short_segments_never_exceed_the_global_bound(
        self,
    ) -> None:
        """Fragmented series respects the global cap."""
        for burst_count in (1_000, 3_000):
            times_s = self._fragmented_series(burst_count)
            magnitude = np.abs(np.sin(np.arange(times_s.size)))
            indices = figures._decimated_indices(
                times_s, magnitude, maximum_points=2_000
            )
            self.assertLessEqual(
                len(indices),
                2_000,
                f"burst_count={burst_count} got {len(indices)}",
            )

    def test_fragmented_series_result_is_ordered_and_unique(
        self,
    ) -> None:
        """Fragmented result is sorted and unique."""
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=2_000
        ).tolist()
        self.assertEqual(indices, sorted(set(indices)))

    def test_fragmented_series_covers_beginning_middle_and_end(
        self,
    ) -> None:
        """Selected segments span the whole run."""
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=2_000
        )
        total_span = times_s[-1] - times_s[0]
        selected_times = times_s[indices]
        # One sample falls in each third of the run.
        for low, high in ((0.0, 1 / 3), (1 / 3, 2 / 3),
                          (2 / 3, 1.0)):
            lower_bound = times_s[0] + low * total_span
            upper_bound = times_s[0] + high * total_span
            in_range = np.count_nonzero(
                (selected_times >= lower_bound)
                & (selected_times <= upper_bound)
            )
            self.assertGreater(
                in_range,
                0,
                f"none in [{lower_bound}, {upper_bound}]",
            )

    def test_fragmented_series_is_deterministic(self) -> None:
        """Repeated decimation gives identical results."""
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        first = figures._decimated_indices(
            times_s, magnitude, maximum_points=2_000
        )
        second = figures._decimated_indices(
            times_s, magnitude, maximum_points=2_000
        )
        np.testing.assert_array_equal(first, second)

    def test_backward_time_jump_is_treated_as_a_segment_boundary(
        self,
    ) -> None:
        """Backward jump splits the series."""
        forward_segment = np.linspace(0, 10, 5_000)
        backward_segment = np.linspace(3, 13, 5_000)
        times_s = np.concatenate(
            [forward_segment, backward_segment]
        )
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = set(
            figures._decimated_indices(
                times_s, magnitude, maximum_points=200
            ).tolist()
        )
        self.assertIn(0, indices)
        self.assertIn(4_999, indices)
        self.assertIn(5_000, indices)
        self.assertIn(9_999, indices)

    def test_large_forward_gap_is_treated_as_a_segment_boundary(
        self,
    ) -> None:
        """Large forward gap splits the series."""
        first_segment = np.linspace(0, 1, 2_000)
        second_segment = np.linspace(500, 501, 2_000)
        times_s = np.concatenate([first_segment, second_segment])
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = set(
            figures._decimated_indices(
                times_s, magnitude, maximum_points=100
            ).tolist()
        )
        self.assertIn(1_999, indices)
        self.assertIn(2_000, indices)

    def test_repeated_timestamps_do_not_crash_or_create_spurious(
        self,
    ) -> None:
        """Repeated timestamps do not split or crash."""
        times_s = np.sort(
            np.concatenate([np.arange(2_500, dtype=float)] * 2)
        )
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=100
        )
        self.assertLessEqual(len(indices), 100)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], times_s.size - 1)

    def test_maximum_points_zero_returns_nothing(self) -> None:
        """Zero budget returns no indices."""
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=0
        )
        self.assertEqual(len(indices), 0)

    def test_maximum_points_one_returns_only_the_final_sample(
        self,
    ) -> None:
        """Budget of one keeps the final sample."""
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=1
        )
        self.assertEqual(list(indices), [9_999])

    def test_maximum_points_two_returns_first_and_last(
        self,
    ) -> None:
        """Budget of two keeps first and last."""
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=2
        )
        self.assertEqual(list(indices), [0, 9_999])

    def test_aligned_channels_stay_consistent_under_fragmentation(
        self,
    ) -> None:
        """Same indices apply to every channel."""
        times_s = self._fragmented_series(3_000)
        sample_count = times_s.size
        values = np.stack(
            [np.sin(np.arange(sample_count) + offset)
             for offset in range(6)],
            axis=-1,
        )
        figure = figures.six_wheel_small_multiples(
            times_s, values, "Speed (m/s)"
        )
        for trace in figure.data:
            self.assertLessEqual(
                len(trace.x), figures.DEFAULT_MAXIMUM_DISPLAY_POINTS
            )
            self.assertEqual(len(trace.x), len(trace.y))

    def test_deterministic_across_repeated_calls(self) -> None:
        """Repeated calls give identical results."""
        rng = np.random.default_rng(3)
        times_s = np.sort(rng.uniform(0, 100, 5_000))
        magnitude = rng.normal(size=5_000)
        first = figures._decimated_indices(
            times_s, magnitude, maximum_points=50
        )
        second = figures._decimated_indices(
            times_s, magnitude, maximum_points=50
        )
        np.testing.assert_array_equal(first, second)

    def test_nan_magnitude_does_not_crash_or_dominate_selection(
        self,
    ) -> None:
        """Mostly-NaN signal still selects the value."""
        sample_count = 2_000
        times_s = np.linspace(0, 10, sample_count)
        magnitude = np.full(sample_count, np.nan)
        magnitude[500] = 5.0
        indices = figures._decimated_indices(
            times_s, magnitude, maximum_points=50
        )
        self.assertLessEqual(len(indices), 60)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], sample_count - 1)


class WheelSmallMultiplesDecimationTests(unittest.TestCase):
    """Tests for six-wheel decimation end to end."""

    def test_high_rate_series_is_decimated_in_the_rendered_figure(
        self,
    ) -> None:
        """High-rate series renders with few points."""
        sample_count = 100_000
        times_s = np.linspace(0, 100, sample_count)
        values = np.tile(np.sin(times_s)[:, np.newaxis], (1, 6))
        figure = figures.six_wheel_small_multiples(
            times_s, values, "rad/s"
        )
        for trace in figure.data:
            self.assertLess(len(trace.x), 5_000)


class ImageSliderFigureTests(unittest.TestCase):
    """Tests for image slider construction."""

    def _make_frame(self, time_s: float) -> ImageFrame:
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        return ImageFrame(
            time_s=time_s, rgb=rgb, source_encoding="rgb8"
        )

    def test_empty_frames_produces_an_empty_figure(self) -> None:
        """No frames produces no images or sliders."""
        figure = figures.image_slider_figure([])
        self.assertEqual(figure.layout.images, ())

    def test_single_frame_has_no_slider(self) -> None:
        """Single frame renders with no slider."""
        figure = figures.image_slider_figure([self._make_frame(1.0)])
        self.assertEqual(len(figure.layout.images), 1)
        self.assertEqual(figure.layout.sliders, ())

    def test_multiple_frames_produce_one_slider_step_each(
        self,
    ) -> None:
        """N frames produce N slider steps."""
        frames = [self._make_frame(t) for t in (1.0, 2.0, 3.0)]
        figure = figures.image_slider_figure(frames)
        self.assertEqual(len(figure.layout.sliders[0].steps), 3)
        for step in figure.layout.sliders[0].steps:
            self.assertEqual(step.method, "relayout")


if __name__ == "__main__":
    unittest.main()
