/*!
 * @file            EstimateLowPassFilterClass.h
 *
 * @brief           Declares the ROS-free low-pass filter that smooths the
 *                  fused estimate for controllers.
 *
 * @date            25/09/2026
 */

#ifndef SRS_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_CLASS_H
#define SRS_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_CLASS_H

/* Object Includes */
#include "estimate_filter/objects/EstimateSampleStruct.h"
#include "estimate_filter/objects/FilterUpdateResultEnum.h"
#include "estimate_filter/objects/LowPassConfigurationStruct.h"

namespace control::estimate_filter
{

/*!
 * @brief           Smooths a pose-and-twist estimate stream.
 *
 * Every stage uses the exact discrete first-order gain
 * alpha = 1 - exp(-dt / tau), tau = 1 / (2 pi f_c), so irregular sample
 * spacing is handled consistently.
 *
 * - Body twist: two identical first-order stages in cascade, a critically
 *   damped second-order low-pass with no step overshoot.
 * - Position: predicted with the filtered body velocity rotated by the
 *   filtered attitude, then pulled toward the input position. On a
 *   constant-velocity ramp the error decays by (1 - alpha) per sample, so
 *   there is no steady-state lag.
 * - Attitude: spherical interpolation toward the input quaternion; no
 *   Euler angles, so yaw crossing +/-pi needs no wrapping.
 *
 * A sample whose stamp does not advance, or that is not finite, is
 * skipped. A gap longer than maximumGap_s, or a requested reset (a frame
 * change), restarts the filter from the sample. The filter only smooths:
 * it cannot remove the estimate's slow drift. Not thread-safe; one owner.
 */
class EstimateLowPassFilter
{
  public:
    /*!
     * @brief           Constructs a filter that restarts from its first
     *                  sample.
     *
     * @param[in]       configuration_in
     *                  Tuning; must satisfy isConfigurationValid().
     */
    explicit EstimateLowPassFilter(
        const LowPassConfiguration &configuration_in) noexcept :
        configuration(configuration_in)
    {
    }

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Filters one input sample.
     *
     * @param[in]       sample_in
     *                  Input estimate.
     *
     * @param[in]       shouldReset_in
     *                  True to restart from this sample regardless of the
     *                  gap, e.g. when the input's frames changed.
     *
     * @return          Whether the sample was filtered, reset the filter or
     *                  was skipped.
     */
    FilterUpdateResult update(const EstimateSample &sample_in,
                              bool                  shouldReset_in);

    /*!
     * @brief           Returns the latest filtered estimate.
     *
     * @return          The output, stamped with the latest accepted input
     *                  stamp; meaningful once update() has not skipped at
     *                  least once.
     */
    const EstimateSample &getOutput() const;

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Restarts every filter stage from one sample.
     *
     * @param[in]       sample_in
     *                  Sample the output and all stages are set to.
     */
    void reset(const EstimateSample &sample_in);

    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Filter tuning.
     *
     * @frame           N/A
     * @units           Mixed; see LowPassConfiguration
     */
    LowPassConfiguration configuration;

    /*!
     * @brief           Whether the filter holds a state to continue from.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasState{false};

    /*!
     * @brief           Latest filtered estimate.
     *
     * @frame           Mixed; see EstimateSample
     * @units           Mixed; see EstimateSample
     */
    EstimateSample output;

    /*!
     * @brief           First-stage linear velocity of the twist cascade.
     *
     * @frame           body
     * @units           metres per second
     */
    Eigen::Vector3d firstStageLinearVelocity_body_mPerS{
        Eigen::Vector3d::Zero()};

    /*!
     * @brief           First-stage angular velocity of the twist cascade.
     *
     * @frame           body
     * @units           radians per second
     */
    Eigen::Vector3d firstStageAngularVelocity_body_radPs{
        Eigen::Vector3d::Zero()};
};

} /* namespace control::estimate_filter */

#endif /* SRS_CONTROL_ESTIMATE_FILTER_ESTIMATE_LOW_PASS_FILTER_CLASS_H   \
        */
