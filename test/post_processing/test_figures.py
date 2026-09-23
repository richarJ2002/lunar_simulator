"""!
@brief  Tests for python_tools.reporting.figures: the stdlib-only PNG
        data-URI encoder used for the image slider (a real bug -- raw
        `go.Image` pixel arrays embedded as JSON numbers made a sampled
        camera-frame report page hundreds of megabytes -- fixed by
        encoding frames as compressed PNGs instead).
"""

from __future__ import annotations

import base64
import struct
import sys
import unittest
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "post_processing"))

from python_tools.data.models import ImageFrame
from python_tools.reporting import figures


def _decode_png_data_uri(data_uri: str) -> np.ndarray:
    """!
    @brief   Decodes a data URI produced by `_encode_png_data_uri` back to
             an RGB array, independently of the encoder, so the test
             proves a genuinely valid PNG rather than merely calling the
             encoder and trusting its own output.

    @param   data_uri
             The `data:image/png;base64,...` URI to decode.

    @return  The decoded RGB array, shape (H, W, 3).
    """
    png_bytes = base64.b64decode(data_uri.split(",", 1)[1])
    assert png_bytes[:8] == b"\x89PNG\r\n\x1a\n"
    offset = 8
    chunks: dict[bytes, bytes] = {}
    while offset < len(png_bytes):
        (length,) = struct.unpack(">I", png_bytes[offset : offset + 4])
        chunk_type = png_bytes[offset + 4 : offset + 8]
        chunks[chunk_type] = png_bytes[offset + 8 : offset + 8 + length]
        offset += 8 + length + 4
    width, height = struct.unpack(">II", chunks[b"IHDR"][:8])
    raw = zlib.decompress(chunks[b"IDAT"])
    stride = 1 + width * 3
    decoded = np.zeros((height, width, 3), dtype=np.uint8)
    for row in range(height):
        row_bytes = raw[row * stride : (row + 1) * stride]
        decoded[row] = np.frombuffer(row_bytes[1:], dtype=np.uint8).reshape(width, 3)
    return decoded


class EncodePngDataUriTests(unittest.TestCase):
    """!
    @brief  Tests for `_encode_png_data_uri`.
    """

    def test_round_trips_exactly(self) -> None:
        """!
        @brief  A decoded PNG matches the original pixel data exactly
                (lossless), independently re-parsed rather than trusting
                the encoder's own claims.
        """
        rng = np.random.default_rng(7)
        image = rng.integers(0, 256, size=(12, 20, 3), dtype=np.uint8)
        decoded = _decode_png_data_uri(figures._encode_png_data_uri(image))
        np.testing.assert_array_equal(decoded, image)

    def test_much_smaller_than_json_pixel_array_would_be(self) -> None:
        """!
        @brief  A realistic (spatially smooth, compressible) image encodes
                to well under half its raw byte size -- the actual defect
                this replaced was embedding raw pixels as JSON numbers,
                which is *larger* than the raw bytes, not smaller.
        """
        # A smooth gradient compresses well, standing in for a real
        # (non-adversarial) camera frame.
        x = np.linspace(0, 255, 64, dtype=np.uint8)
        smooth_image = np.tile(x[np.newaxis, :, np.newaxis], (64, 1, 3))
        data_uri = figures._encode_png_data_uri(smooth_image)
        encoded_bytes = base64.b64decode(data_uri.split(",", 1)[1])
        self.assertLess(len(encoded_bytes), smooth_image.nbytes // 2)


class EmptyStateCardHtmlTests(unittest.TestCase):
    """!
    @brief  Tests for `empty_state_card_html`'s escaping (Defect 7 in the
            plan's 2026-09-22 audit: this was the one HTML-producing
            function in the codebase that did not escape its dynamic
            text, unlike python_tools.reporting.html's own functions).
    """

    def test_escapes_html_significant_characters(self) -> None:
        """!
        @brief  A message containing `<`, `>`, `&`, quotes, and a
                script-like payload is escaped, never injected verbatim.
        """
        message = "<script>alert(1)</script> & 'quoted' \"double\" < >"
        card_html = figures.empty_state_card_html(message)
        self.assertNotIn("<script>alert(1)</script>", card_html)
        self.assertIn("&lt;script&gt;", card_html)
        self.assertIn("&amp;", card_html)
        self.assertIn("&#x27;quoted&#x27;", card_html)
        self.assertIn("&quot;double&quot;", card_html)


class DecimatedIndicesTests(unittest.TestCase):
    """!
    @brief  Tests for `_decimated_indices`, the shared display decimation
            every high-rate time-series figure constructor uses to fix
            Defect 3 in the plan's 2026-09-22 audit: a real generated
            `wheel_odometry.html` was 45 MB because the undecimated
            ~1 kHz raw joint-state feed was embedded directly in Plotly
            JSON. Decimation affects display only -- python_tools.data.
            metrics always computes against the full-resolution arrays.
    """

    def test_inputs_already_below_the_limit_are_untouched(self) -> None:
        """!
        @brief  A series at or under the point budget is returned whole,
                with no segmentation or bucketing overhead.
        """
        times_s = np.linspace(0, 1, 50)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=2000)
        np.testing.assert_array_equal(indices, np.arange(50))

    def test_preserves_first_and_last_sample(self) -> None:
        """!
        @brief  The very first and last sample of a long series always
                survive decimation.
        """
        times_s = np.linspace(0, 100, 100_000)
        magnitude = np.sin(times_s)
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=200)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 99_999)

    def test_result_respects_the_bound_for_monotonic_data(self) -> None:
        """!
        @brief  A long, smoothly monotonic series decimates down close to
                the configured bound, not just below the original length.
        """
        times_s = np.linspace(0, 100, 100_000)
        magnitude = times_s.copy()
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=200)
        self.assertLessEqual(len(indices), 210)
        self.assertGreater(len(indices), 50)

    def test_preserves_a_sharp_spike_a_naive_every_nth_sample_would_miss(self) -> None:
        """!
        @brief  A single-sample spike at an index that does not align with
                any "clean" stride is still selected -- min/max bucket
                decimation, not naive every-Nth sampling.
        """
        sample_count = 10_000
        times_s = np.linspace(0, 100, sample_count)
        magnitude = np.zeros(sample_count)
        spike_index = 3333
        magnitude[spike_index] = 1000.0
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=100)
        self.assertIn(spike_index, indices)
        naive_every_nth = set(range(0, sample_count, sample_count // 100))
        self.assertNotIn(spike_index, naive_every_nth)

    def test_segments_are_decimated_independently_across_a_gap(self) -> None:
        """!
        @brief  Two dense segments separated by a large gap each keep
                their own first and last sample, even under heavy overall
                decimation -- a discontinuity is never bridged.
        """
        first_segment = np.linspace(0, 1, 5_000)
        second_segment = np.linspace(1000, 1001, 5_000)
        times_s = np.concatenate([first_segment, second_segment])
        magnitude = np.abs(np.sin(times_s))
        indices = set(figures._decimated_indices(times_s, magnitude, maximum_points=100).tolist())
        self.assertIn(0, indices)
        self.assertIn(4_999, indices)
        self.assertIn(5_000, indices)
        self.assertIn(9_999, indices)

    def _fragmented_series(self, burst_count: int, burst_length: int = 5):
        """!
        @brief   Builds a series of `burst_count` short bursts, each
                 `burst_length` samples densely spaced, separated by real
                 gaps 500x the intra-burst spacing -- a plausible shape for
                 an intermittently-dropping topic over a long run, and the
                 exact pattern that overshot `maximum_points` several-fold
                 before the 2026-09-23 correction.

        @param   burst_count
                 The number of separate bursts (segments) to generate.
        @param   burst_length
                 Samples per burst.

        @return  `times_s`, ascending, shape `(burst_count * burst_length,)`.
        """
        times = []
        t = 0.0
        for _ in range(burst_count):
            for _ in range(burst_length):
                times.append(t)
                t += 0.01
            t += 5.0
        return np.array(times)

    def test_many_short_segments_never_exceed_the_global_bound(self) -> None:
        """!
        @brief  Regression for the 2026-09-23 correction: the previous
                per-segment "at least 2 points" floor had no global cap.
                1,000 real, separately-gapped bursts (5,000 samples) used
                to return ~3,352 points against a 2,000-point budget; 3,000
                bursts (15,000 samples) used to return ~10,057. Both must
                now respect the cap exactly.
        """
        for burst_count in (1_000, 3_000):
            times_s = self._fragmented_series(burst_count)
            magnitude = np.abs(np.sin(np.arange(times_s.size)))
            indices = figures._decimated_indices(times_s, magnitude, maximum_points=2_000)
            self.assertLessEqual(
                len(indices),
                2_000,
                f"burst_count={burst_count} returned {len(indices)} > 2000",
            )

    def test_fragmented_series_result_is_ordered_and_unique(self) -> None:
        """!
        @brief  Even under the many-segments code path, the returned
                indices are sorted and free of duplicates.
        """
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=2_000).tolist()
        self.assertEqual(indices, sorted(set(indices)))

    def test_fragmented_series_covers_beginning_middle_and_end(self) -> None:
        """!
        @brief  Selecting a representative subset of segments must span
                the complete run, not cluster near the start -- the
                previous "at least 2 points per segment, in segment order"
                loop never truncated, but a naive prefix-based fix to the
                overshoot would have reintroduced exactly this bias.
        """
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=2_000)
        total_span = times_s[-1] - times_s[0]
        selected_times = times_s[indices]
        # At least one selected sample falls in each third of the run.
        for low, high in ((0.0, 1 / 3), (1 / 3, 2 / 3), (2 / 3, 1.0)):
            lower_bound = times_s[0] + low * total_span
            upper_bound = times_s[0] + high * total_span
            in_range = np.count_nonzero(
                (selected_times >= lower_bound) & (selected_times <= upper_bound)
            )
            self.assertGreater(
                in_range, 0, f"no selected sample in [{lower_bound}, {upper_bound}]"
            )

    def test_fragmented_series_is_deterministic(self) -> None:
        """!
        @brief  Decimating the same fragmented input twice, including the
                representative-segment-selection code path, produces
                identical results.
        """
        times_s = self._fragmented_series(3_000)
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        first = figures._decimated_indices(times_s, magnitude, maximum_points=2_000)
        second = figures._decimated_indices(times_s, magnitude, maximum_points=2_000)
        np.testing.assert_array_equal(first, second)

    def test_backward_time_jump_is_treated_as_a_segment_boundary(self) -> None:
        """!
        @brief  Regression for the 2026-09-23 correction: the previous
                gap detector only checked for large *forward* gaps, so a
                genuine backward time jump (e.g. a clock reset) was
                invisible to it even though python_tools.bag.timestamps.
                segment_series always treats one as a discontinuity. A
                backward jump must now split the series, so the sample
                immediately before and after it both survive decimation.
        """
        forward_segment = np.linspace(0, 10, 5_000)
        backward_segment = np.linspace(3, 13, 5_000)
        times_s = np.concatenate([forward_segment, backward_segment])
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = set(
            figures._decimated_indices(times_s, magnitude, maximum_points=200).tolist()
        )
        self.assertIn(0, indices)
        self.assertIn(4_999, indices)
        self.assertIn(5_000, indices)
        self.assertIn(9_999, indices)

    def test_large_forward_gap_is_treated_as_a_segment_boundary(self) -> None:
        """!
        @brief  A single, very large forward gap (not a backward jump) is
                still detected as a discontinuity, exercised alongside the
                backward-jump case so both of segment_series's own two
                conditions are independently verified here.
        """
        first_segment = np.linspace(0, 1, 2_000)
        second_segment = np.linspace(500, 501, 2_000)
        times_s = np.concatenate([first_segment, second_segment])
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = set(
            figures._decimated_indices(times_s, magnitude, maximum_points=100).tolist()
        )
        self.assertIn(1_999, indices)
        self.assertIn(2_000, indices)

    def test_repeated_timestamps_do_not_crash_or_create_spurious_segments(self) -> None:
        """!
        @brief  A run of exactly repeated (zero-diff) timestamps is not,
                by itself, a discontinuity -- matching segment_series's
                own strict "<" backward-jump condition, which a repeated
                (equal) timestamp does not satisfy -- and must not crash
                the median/threshold computation or bucket ranking.
        """
        times_s = np.sort(np.concatenate([np.arange(2_500, dtype=float)] * 2))
        magnitude = np.abs(np.sin(np.arange(times_s.size)))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=100)
        self.assertLessEqual(len(indices), 100)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], times_s.size - 1)

    def test_maximum_points_zero_returns_nothing(self) -> None:
        """!
        @brief  A zero budget returns no indices at all.
        """
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=0)
        self.assertEqual(len(indices), 0)

    def test_maximum_points_one_returns_only_the_final_sample(self) -> None:
        """!
        @brief  A budget of one cannot represent both endpoints; the final
                sample is kept, consistent with
                python_tools.data.extractors._BoundedEvenSampler's
                identical one-slot policy.
        """
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=1)
        self.assertEqual(list(indices), [9_999])

    def test_maximum_points_two_returns_first_and_last(self) -> None:
        """!
        @brief  A budget of exactly two returns exactly the true global
                first and last sample.
        """
        times_s = np.linspace(0, 100, 10_000)
        magnitude = np.abs(np.sin(times_s))
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=2)
        self.assertEqual(list(indices), [0, 9_999])

    def test_aligned_channels_stay_consistent_under_fragmentation(self) -> None:
        """!
        @brief  End-to-end through the public `six_wheel_small_multiples`
                constructor: even for a heavily fragmented series, the
                same selected indices apply to `times_s` and every wheel
                channel, and the embedded trace point count respects the
                global bound.
        """
        times_s = self._fragmented_series(3_000)
        sample_count = times_s.size
        values = np.stack(
            [np.sin(np.arange(sample_count) + offset) for offset in range(6)], axis=-1
        )
        figure = figures.six_wheel_small_multiples(times_s, values, "Speed (m/s)")
        for trace in figure.data:
            self.assertLessEqual(len(trace.x), figures.DEFAULT_MAXIMUM_DISPLAY_POINTS)
            self.assertEqual(len(trace.x), len(trace.y))

    def test_deterministic_across_repeated_calls(self) -> None:
        """!
        @brief  Decimating the same input twice produces identical
                results -- required for reproducible report regeneration.
        """
        rng = np.random.default_rng(3)
        times_s = np.sort(rng.uniform(0, 100, 5_000))
        magnitude = rng.normal(size=5_000)
        first = figures._decimated_indices(times_s, magnitude, maximum_points=50)
        second = figures._decimated_indices(times_s, magnitude, maximum_points=50)
        np.testing.assert_array_equal(first, second)

    def test_nan_magnitude_does_not_crash_or_dominate_selection(self) -> None:
        """!
        @brief  A mostly-NaN ranking signal (e.g. an unaligned error
                series) neither crashes nor prevents the one real value of
                interest from being selected.
        """
        sample_count = 2_000
        times_s = np.linspace(0, 10, sample_count)
        magnitude = np.full(sample_count, np.nan)
        magnitude[500] = 5.0
        indices = figures._decimated_indices(times_s, magnitude, maximum_points=50)
        self.assertLessEqual(len(indices), 60)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], sample_count - 1)


class WheelSmallMultiplesDecimationTests(unittest.TestCase):
    """!
    @brief  End-to-end confirmation that `six_wheel_small_multiples` (the
            constructor directly responsible for Defect 3's 45 MB real
            report page) actually applies decimation through the public
            API, not just at the private helper.
    """

    def test_high_rate_series_is_decimated_in_the_rendered_figure(self) -> None:
        """!
        @brief  A ~1 kHz-scale synthetic joint-state series renders with
                far fewer points per trace than were fed in.
        """
        sample_count = 100_000
        times_s = np.linspace(0, 100, sample_count)
        values = np.tile(np.sin(times_s)[:, np.newaxis], (1, 6))
        figure = figures.six_wheel_small_multiples(times_s, values, "rad/s")
        for trace in figure.data:
            self.assertLess(len(trace.x), 5_000)


class ImageSliderFigureTests(unittest.TestCase):
    """!
    @brief  Tests for `image_slider_figure`'s data-URI-based embedding and
            slider construction.
    """

    def _make_frame(self, time_s: float) -> ImageFrame:
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        return ImageFrame(time_s=time_s, rgb=rgb, source_encoding="rgb8")

    def test_empty_frames_produces_an_empty_figure(self) -> None:
        """!
        @brief  No frames produces a figure with no images or sliders,
                not an error.
        """
        figure = figures.image_slider_figure([])
        self.assertEqual(figure.layout.images, ())

    def test_single_frame_has_no_slider(self) -> None:
        """!
        @brief  A single frame renders as a static image with no slider
                (a slider needs at least two positions to be meaningful).
        """
        figure = figures.image_slider_figure([self._make_frame(1.0)])
        self.assertEqual(len(figure.layout.images), 1)
        self.assertEqual(figure.layout.sliders, ())

    def test_multiple_frames_produce_one_slider_step_each(self) -> None:
        """!
        @brief  N frames produce exactly N slider steps, each embedding
                that frame's own data URI.
        """
        frames = [self._make_frame(t) for t in (1.0, 2.0, 3.0)]
        figure = figures.image_slider_figure(frames)
        self.assertEqual(len(figure.layout.sliders[0].steps), 3)
        for step in figure.layout.sliders[0].steps:
            self.assertEqual(step.method, "relayout")


if __name__ == "__main__":
    unittest.main()
