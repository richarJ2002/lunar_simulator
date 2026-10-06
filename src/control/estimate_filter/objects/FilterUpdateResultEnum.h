/*!
 * @file            FilterUpdateResultEnum.h
 *
 * @brief           Declares what one low-pass filter update did.
 *
 * @date            25/09/2026
 */

#ifndef SRS_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_ENUM_H
#define SRS_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_ENUM_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace control::estimate_filter
{

/*!
 * @brief           Outcome of EstimateLowPassFilter::update().
 */
enum class FilterUpdateResult : std::uint8_t
{
    /*!
     * @brief       The sample was filtered into a new output.
     */
    FILTER_UPDATE_RESULT_FILTERED = 0U,

    /*!
     * @brief       Filter restarted from the sample (first sample, a gap longer
     *              than the continuity limit, or a requested reset).
     */
    FILTER_UPDATE_RESULT_RESET = 1U,

    /*!
     * @brief       The sample was ignored: its stamp did not advance, or it was
     *              not finite. The output is unchanged.
     */
    FILTER_UPDATE_RESULT_SKIPPED = 2U
};

} /* namespace control::estimate_filter */

#endif /* SRS_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_ENUM_H */
