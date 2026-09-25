/*!
 * @file            FilterUpdateResult.h
 *
 * @brief           Declares what one low-pass filter update did.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_H
#define LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace control::estimate_filter
{

/*!
 * @brief           Outcome of EstimateLowPassFilter::update().
 */
enum class FilterUpdateResult : std::uint8_t
{
    /*! The sample was filtered into a new output. */
    FILTER_UPDATE_RESULT_FILTERED = 0U,

    /*! The filter restarted from the sample (first sample, a gap longer
     *  than the continuity limit, or a requested reset). */
    FILTER_UPDATE_RESULT_RESET = 1U,

    /*! The sample was ignored: its stamp did not advance, or it was not
     *  finite. The output is unchanged. */
    FILTER_UPDATE_RESULT_SKIPPED = 2U
};

} /* namespace control::estimate_filter */

#endif /* LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_FILTER_UPDATE_RESULT_H */
