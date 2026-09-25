/*!
 * @file            holdImuShock.cc
 *
 * @brief           Implements the single-sample IMU contact-shock gate.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

bool AlphaKalmanFilterNode::holdImuShock(const ImuSample &previousSample_in,
                                         double           thresholdMps2_in,
                                         ImuSample       &sample_inout)
{
    /* A held previous sample means the change persisted into this one, so
     * it is real and must be accepted. */
    if (!(thresholdMps2_in > 0.0) ||
        !previousSample_in.discardedDeltaVelocity_body_mPerS.isZero(0.0))
    {
        return false;
    }
    const double interval_s =
        sample_inout.timestamp_s - previousSample_in.timestamp_s;
    if (!(interval_s > 0.0))
    {
        return false;
    }

    const Eigen::Vector3d specificForceChange_body_mPerS2 =
        sample_inout.linearAcceleration_body_mPerS2 -
        previousSample_in.linearAcceleration_body_mPerS2;
    if (!(specificForceChange_body_mPerS2.norm() > thresholdMps2_in))
    {
        return false;
    }

    /* Prediction holds a sample's specific force over the interval that
     * ends at its stamp, so this is the velocity change it would have
     * integrated. */
    sample_inout.discardedDeltaVelocity_body_mPerS =
        specificForceChange_body_mPerS2 * interval_s;
    sample_inout.linearAcceleration_body_mPerS2 =
        previousSample_in.linearAcceleration_body_mPerS2;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
