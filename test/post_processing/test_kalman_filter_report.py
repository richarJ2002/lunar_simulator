"""!
@brief  Tests for post_processing_kalman_filter's displayed configuration:
        the effective NIS threshold shown on the Kalman page must equal the
        value the ESKF actually applies (2026-09-23 correction: the page's
        table listed the 7-DoF chi-square value against 6 DoF, overstating
        the visual pose gate as 18.475 instead of 16.812).
"""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "post_processing"))

import post_processing_kalman_filter  # noqa: E402

# The C++ source of truth for the automatic thresholds.
SELECT_NIS_THRESHOLD_SOURCE = (
    REPOSITORY_ROOT
    / "src/systems/alpha/alpha_localisation/alpha_kalman_filter/methods/selectNisThreshold.cc"
)


def _cpp_chi_square_table() -> list[float]:
    """!
    @brief   Parses selectNisThreshold.cc's `CHI_SQUARE_99_PERCENT` array.

    @return  The table's values; index `k` is the threshold for `k + 1` DoF.
    """
    source = SELECT_NIS_THRESHOLD_SOURCE.read_text(encoding="utf-8")
    body = re.search(r"CHI_SQUARE_99_PERCENT\s*\{([^}]*)\}", source).group(1)
    return [float(value) for value in re.findall(r"\d+\.\d+", body)]


class EffectiveNisThresholdTests(unittest.TestCase):
    """!
    @brief  The report's automatic NIS thresholds match the ESKF's own.
    """

    def test_table_matches_the_cpp_source_for_every_listed_dof(self) -> None:
        """!
        @brief  Every degree of freedom the report lists maps to the same
                value selectNisThreshold.cc applies for that dimension.
        """
        cpp_table = _cpp_chi_square_table()
        for degrees_of_freedom, value in post_processing_kalman_filter._CHI_SQUARE_99_PERCENT.items():
            with self.subTest(degrees_of_freedom=degrees_of_freedom):
                self.assertAlmostEqual(value, cpp_table[degrees_of_freedom - 1], places=6)

    def test_visual_pose_and_wheel_twist_automatic_values(self) -> None:
        """!
        @brief  The two gates the page displays: 6-DoF visual pose and
                2-DoF wheel twist, with a zero (automatic) configuration.
        """
        self.assertAlmostEqual(post_processing_kalman_filter._effective_nis_threshold(0.0, 6), 16.812)
        self.assertAlmostEqual(post_processing_kalman_filter._effective_nis_threshold(0.0, 2), 9.210)

    def test_positive_configured_value_is_used_verbatim(self) -> None:
        """!
        @brief  A positive configured threshold overrides the table.
        """
        self.assertEqual(post_processing_kalman_filter._effective_nis_threshold(12.5, 6), 12.5)


if __name__ == "__main__":
    unittest.main()
