"""Tests for Kalman filter displayed thresholds.

Contents:
    EffectiveNisThresholdTests: automatic NIS values.

The page must show the value the filter applies; the
visual pose gate is the 6-DoF value, not the 7-DoF one.
"""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "post_processing"))

import post_processing_kalman_filter  # noqa: E402

# The C++ source of truth for automatic thresholds.
SELECT_NIS_THRESHOLD_SOURCE = (
    REPOSITORY_ROOT
    / "src/systems/alpha/alpha_localisation/alpha_kalman_filter"
    / "methods/selectNisThreshold.cc"
)


def _cpp_chi_square_table() -> list[float]:
    """Parse the C++ chi-square table."""
    source = SELECT_NIS_THRESHOLD_SOURCE.read_text(encoding="utf-8")
    body = re.search(
        r"CHI_SQUARE_99_PERCENT\s*\{([^}]*)\}", source
    ).group(1)
    return [float(value) for value in re.findall(r"\d+\.\d+", body)]


class EffectiveNisThresholdTests(unittest.TestCase):
    """Tests for report automatic NIS thresholds."""

    def test_table_matches_the_cpp_source_for_every_listed_dof(
        self,
    ) -> None:
        """Every listed DoF matches the C++ table."""
        cpp_table = _cpp_chi_square_table()
        for degrees_of_freedom, value in (
            post_processing_kalman_filter._CHI_SQUARE_99_PERCENT.items()
        ):
            with self.subTest(degrees_of_freedom=degrees_of_freedom):
                self.assertAlmostEqual(
                    value,
                    cpp_table[degrees_of_freedom - 1],
                    places=6,
                )

    def test_visual_pose_and_wheel_twist_automatic_values(
        self,
    ) -> None:
        """6-DoF pose and 2-DoF twist gates are correct."""
        self.assertAlmostEqual(
            post_processing_kalman_filter._effective_nis_threshold(
                0.0, 6
            ),
            16.812,
        )
        self.assertAlmostEqual(
            post_processing_kalman_filter._effective_nis_threshold(
                0.0, 2
            ),
            9.210,
        )

    def test_positive_configured_value_is_used_verbatim(
        self,
    ) -> None:
        """Positive configured threshold overrides the table."""
        self.assertEqual(
            post_processing_kalman_filter._effective_nis_threshold(
                12.5, 6
            ),
            12.5,
        )


if __name__ == "__main__":
    unittest.main()
