/*!
 * @File:         selectByMinimumDistance.cc
 *
 * @Brief:        Implements ranked greedy minimum-distance feature
 *                selection.
 *
 * @Date:         16/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "feature_tracking/objects/ShiTomasiCornerDetector.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>
#include <array>
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

std::size_t ShiTomasiCornerDetector::selectByMinimumDistance(
    std::size_t                                      candidateCount_in,
    std::array<Point2D, MAXIMUM_SUPPORTED_FEATURES> &features_out) noexcept
{
    /*!
     * Sort by descending response, breaking exact ties by a fixed
     * (row, column) order. The tie-break is required for determinism:
     * without it, candidates with identical response (a real
     * possibility on synthetic or saturated imagery) would be ordered
     * however `std::sort` happens to leave them, which is not guaranteed
     * stable and could differ across standard-library implementations.
     */
    std::sort(candidatePool_.begin(),
              candidatePool_.begin() +
                  static_cast<std::ptrdiff_t>(candidateCount_in),
              [](const Candidate &first_in,
                 const Candidate &second_in) noexcept -> bool
              {
                  if (first_in.response != second_in.response)
                  {
                      return first_in.response > second_in.response;
                  }
                  if (first_in.row != second_in.row)
                  {
                      return first_in.row < second_in.row;
                  }
                  return first_in.column < second_in.column;
              });

    /* Squared comparison avoids a square root per candidate pair below. */
    const float minimumDistanceSquared =
        minimumDistancePx_ * minimumDistancePx_;

    /* Number of features accepted so far. */
    std::size_t acceptedCount = 0U;

    /*!
     * Greedy selection in ranked order: accept the strongest remaining
     * candidate unless it lies within the configured minimum distance of
     * an already-accepted feature. This is an all-pairs check against
     * every already-accepted feature, but both `candidateCount_in`
     * (bounded by `MAXIMUM_CANDIDATE_POOL`) and `acceptedCount` (bounded
     * by `maximumFeatures_ &lt;= MAXIMUM_SUPPORTED_FEATURES`) are already
     * small, fixed, compile-/configuration-time constants -- the worst
     * case is a small, bounded number of comparisons regardless of image
     * content, so no spatial partitioning scheme is needed for either
     * correctness or performance here.
     */
    for (std::size_t candidateIndex = 0U;
         candidateIndex < candidateCount_in && acceptedCount < maximumFeatures_;
         ++candidateIndex)
    {
        /* This candidate's position, tested against every already-
         * accepted feature below. */
        const Point2D &candidatePosition =
            candidatePool_[candidateIndex].position;

        bool isFarEnoughFromEveryAccepted = true;
        for (std::size_t acceptedIndex = 0U; acceptedIndex < acceptedCount;
             ++acceptedIndex)
        {
            /* Displacement to this already-accepted feature. */
            const float deltaX =
                candidatePosition.x - features_out[acceptedIndex].x;
            const float deltaY =
                candidatePosition.y - features_out[acceptedIndex].y;

            /* Squared Euclidean distance to this already-accepted
             * feature. */
            const float distanceSquared = (deltaX * deltaX) + (deltaY * deltaY);

            /* Too close to this already-accepted feature; reject the
             * candidate outright, no need to check the rest. */
            if (distanceSquared < minimumDistanceSquared)
            {
                isFarEnoughFromEveryAccepted = false;
                break;
            }
        }

        if (isFarEnoughFromEveryAccepted)
        {
            /* Accept this candidate. */
            features_out[acceptedCount] = candidatePosition;
            ++acceptedCount;
        }
    }

    return acceptedCount;
}

} /* namespace localisation::visual_odometry::feature_tracking */
