/*!
 * @File:         collectCandidates.cc
 *
 * @Brief:        Implements bounded thresholded local-maxima extraction.
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
#include <cstddef>

namespace localisation::visual_odometry::feature_tracking
{

/* This function's cognitive-complexity finding is accepted: it is one
 * cohesive algorithmic step (bounded local-maxima candidate collection
 * with non-maximum suppression) matching the reference algorithm's own
 * structure; splitting it further would reduce traceability without
 * reducing the underlying mathematical complexity -- see
 * DEVIATION_LOG.md. */
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::size_t ShiTomasiCornerDetector::collectCandidates(
    float responseThreshold_in) noexcept
{
    /* Number of valid entries currently held in candidatePool_. */
    std::size_t count = 0U;

    /*!
     * Orders two candidates so `std::push_heap`/`std::pop_heap` maintain
     * a MIN-heap on response. Those algorithms keep the element that is
     * "greatest" under the supplied ordering at the front; ordering by
     * "response is greater" therefore keeps the SMALLEST-response
     * candidate at the front, which is exactly the one evicted in
     * `O(log n)` below once the fixed-capacity pool is full and a
     * stronger candidate arrives. Declared as a local lambda (rather
     * than a free function) specifically so it can name the private
     * nested `Candidate` type.
     */
    const auto isLowerPriorityForEviction =
        [](const Candidate &first_in,
           const Candidate &second_in) noexcept -> bool
    { return first_in.response > second_in.response; };

    /*!
     * Candidacy is restricted to strictly interior pixels (excluding the
     * outermost row/column on every side) so the 3x3 local-maxima
     * comparison below always reads real, non-mirrored neighbor
     * responses rather than values fabricated by computeGradients()'s
     * border reflection -- a corner at the extreme image edge is
     * therefore never reported, a deliberate margin rather than an
     * oversight.
     */
    for (int row = 1; row <= height_ - 2; ++row)
    {
        for (int column = 1; column <= width_ - 2; ++column)
        {
            /* Row-major offset of this pixel's response. */
            const std::size_t offset = (static_cast<std::size_t>(row) *
                                        static_cast<std::size_t>(width_)) +
                                       static_cast<std::size_t>(column);

            /* This pixel's response. */
            const float centerResponse = response_[offset];

            /* Reject anything at or below the acceptance threshold
             * before spending time on the neighbor comparison below. */
            if (!(centerResponse > responseThreshold_in))
            {
                continue;
            }

            /*!
             * Compare against all eight real neighbors; a pixel accepts
             * ties ("greater than", not "greater than or equal to", is
             * the REJECT condition) so a flat plateau's pixels are all
             * independently retained, matching OpenCV's dilate-then-
             * equal-compare non-max suppression rather than arbitrarily
             * picking one plateau pixel.
             */
            bool isLocalMaximum = true;
            for (int windowRow = -1; windowRow <= 1 && isLocalMaximum;
                 ++windowRow)
            {
                for (int windowColumn = -1; windowColumn <= 1; ++windowColumn)
                {
                    /* The center pixel is not its own neighbor. */
                    if (windowRow == 0 && windowColumn == 0)
                    {
                        continue;
                    }

                    /* Row-major offset of this real (non-mirrored)
                     * neighbor. */
                    const std::size_t neighborOffset =
                        (static_cast<std::size_t>(row + windowRow) *
                         static_cast<std::size_t>(width_)) +
                        static_cast<std::size_t>(column + windowColumn);

                    /* A strictly stronger neighbor disqualifies this
                     * pixel as a local maximum. */
                    if (response_[neighborOffset] > centerResponse)
                    {
                        isLocalMaximum = false;
                        break;
                    }
                }
            }

            if (!isLocalMaximum)
            {
                continue;
            }

            /* This pixel is an accepted candidate; assemble its record. */
            const Candidate candidate{
                Point2D{static_cast<float>(column), static_cast<float>(row)},
                centerResponse,
                row,
                column};

            if (count < MAXIMUM_CANDIDATE_POOL)
            {
                /* Pool not yet full: append and restore the heap
                 * property. */
                candidatePool_[count] = candidate;
                ++count;
                std::push_heap(candidatePool_.begin(),
                               candidatePool_.begin() +
                                   static_cast<std::ptrdiff_t>(count),
                               isLowerPriorityForEviction);
            }
            else if (candidate.response > candidatePool_.front().response)
            {
                /*!
                 * Pool full but this candidate outranks the weakest
                 * currently held one: evict it (pop_heap moves it to the
                 * back), overwrite that slot with the new candidate, and
                 * restore the heap property. This keeps the pool's
                 * worst-case size fixed at MAXIMUM_CANDIDATE_POOL
                 * regardless of how many interior local maxima the image
                 * actually has.
                 */
                std::pop_heap(candidatePool_.begin(),
                              candidatePool_.begin() +
                                  static_cast<std::ptrdiff_t>(count),
                              isLowerPriorityForEviction);
                candidatePool_[count - 1U] = candidate;
                std::push_heap(candidatePool_.begin(),
                               candidatePool_.begin() +
                                   static_cast<std::ptrdiff_t>(count),
                               isLowerPriorityForEviction);
            }
            /* Otherwise: pool full and this candidate is no stronger
             * than the weakest kept one; discard it. */
        }
    }

    return count;
}

} /* namespace localisation::visual_odometry::feature_tracking */
