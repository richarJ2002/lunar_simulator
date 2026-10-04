/*!
 * @file            holdImuShock.cc
 *
 * @brief           Implements the single-sample IMU contact-shock gate.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNodeClass.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

bool AlphaKalmanFilterNode::holdImuShock(const ImuSample &previousSample_in,
                                         double           threshold_mPs2_in,
                                         ImuSample       &sample_inout)
{
    /* A held previous sample means the change persisted into this one, so
     * it is real and must be accepted. */
    if (!(threshold_mPs2_in > 0.0) ||
        !previousSample_in.discardedDeltaVelocity_body_mPs.isZero(0.0))
    {
        return false;
    }
    const double interval_s =
        sample_inout.timestamp_s - previousSample_in.timestamp_s;
    if (!(interval_s > 0.0))
    {
        return false;
    }

    const Eigen::Vector3d specificForceChange_body_mPs2 =
        sample_inout.linearAcceleration_body_mPs2 -
        previousSample_in.linearAcceleration_body_mPs2;
    if (!(specificForceChange_body_mPs2.norm() > threshold_mPs2_in))
    {
        return false;
    }

    /* Prediction holds a sample's specific force over the interval that
     * ends at its stamp, so this is the velocity change it would have
     * integrated. */
    sample_inout.discardedDeltaVelocity_body_mPs =
        specificForceChange_body_mPs2 * interval_s;
    sample_inout.linearAcceleration_body_mPs2 =
        previousSample_in.linearAcceleration_body_mPs2;
    return true;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
