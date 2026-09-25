/*!
 * @file            LowPassConfiguration.h
 *
 * @brief           Declares the estimate low-pass filter's tuning.
 *
 * @date            25/09/2026
 */

#ifndef LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_LOW_PASS_CONFIGURATION_H
#define LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_LOW_PASS_CONFIGURATION_H

namespace control::estimate_filter
{

/*!
 * @brief           Corner frequencies and continuity limit of the estimate
 *                  low-pass filter. Check with isConfigurationValid()
 *                  before use.
 */
struct LowPassConfiguration
{
  public:
    /*!
     * @brief           Corner frequency of each of the two identical
     *                  first-order stages filtering the body twist (the
     *                  cascade is critically damped; its -3 dB point is
     *                  about 0.64 of this).
     *
     * @frame           N/A
     * @units           hertz
     */
    double velocityCutoffHz{1.0};

    /*!
     * @brief           Corner frequency of the position pull toward the
     *                  input position.
     *
     * @frame           N/A
     * @units           hertz
     */
    double positionCutoffHz{0.5};

    /*!
     * @brief           Corner frequency of the attitude SLERP toward the
     *                  input attitude.
     *
     * @frame           N/A
     * @units           hertz
     */
    double attitudeCutoffHz{1.0};

    /*!
     * @brief           Largest stamp gap still filtered continuously; a
     *                  longer gap resets the filter to the new sample.
     *
     * @frame           N/A
     * @units           seconds
     */
    double maximumGapS{0.5};
};

} /* namespace control::estimate_filter */

#endif /* LUNAR_SIMULATOR_CONTROL_ESTIMATE_FILTER_LOW_PASS_CONFIGURATION_H */
