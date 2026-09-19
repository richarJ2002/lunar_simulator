/*!
 * @File:         compensateMeasurementAge.cc
 *
 * @Brief:        Implements constant-rate extrapolation of a stale
 *                measurement's state estimate.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
#include <algorithm>
#include <cmath>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::compensateMeasurementAge(
    StateVector &state_inout, double ageS_in)
{
    /*!
     * Position is advanced with the current linear-velocity estimate under
     * a constant-velocity assumption over the (short) measurement age.
     */
    state_inout.segment<3>(0) += state_inout.segment<3>(6) * ageS_in;

    /*!
     * Re-applies the same Euler-rate kinematics as computeProcessModel()
     * so that extrapolating attitude here stays consistent with how the
     * EKF's own process model would have predicted forward:
     * rollDot  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
     * pitchDot = cos(roll) q - sin(roll) r
     * yawDot   = sin(roll) sec(pitch) q + cos(roll) sec(pitch) r
     * where [p q r] is body angular rate (rad/s) and pitch is clamped away
     * from +/-pi/2 to avoid the tan/sec singularity there.
     */

    /* Current roll state, used by every rate below. */
    const double rollRad = state_inout(3);

    /* Current pitch state, clamped away from the tan/sec singularity. */
    const double pitchRad =
        std::clamp(state_inout(4), -1.5533, 1.5533);

    /* sin(roll), reused by both the roll-rate and yaw-rate equations. */
    const double sineRoll = std::sin(rollRad);

    /* cos(roll), reused by both the roll-rate and pitch-rate equations. */
    const double cosineRoll = std::cos(rollRad);

    /* tan(pitch), used only by the roll-rate equation. */
    const double tangentPitch = std::tan(pitchRad);

    /* sec(pitch) = 1 / cos(pitch), used only by the yaw-rate equation. */
    const double secantPitch = 1.0 / std::cos(pitchRad);

    /* Evaluate the roll-rate equation above. */
    const double rollRateRadps =
        state_inout(9) + sineRoll * tangentPitch * state_inout(10) +
        cosineRoll * tangentPitch * state_inout(11);

    /* Evaluate the pitch-rate equation above. */
    const double pitchRateRadps =
        cosineRoll * state_inout(10) - sineRoll * state_inout(11);

    /* Evaluate the yaw-rate equation above. */
    const double yawRateRadps =
        sineRoll * secantPitch * state_inout(10) +
        cosineRoll * secantPitch * state_inout(11);

    /* Advance roll by its rate over the measurement age, then wrap it
     * back into the canonical [-pi, pi] range. */
    state_inout(3) = wrapAngle(state_inout(3) + rollRateRadps * ageS_in);

    /* Advance and wrap pitch the same way. */
    state_inout(4) =
        wrapAngle(state_inout(4) + pitchRateRadps * ageS_in);

    /* Advance and wrap yaw the same way. */
    state_inout(5) = wrapAngle(state_inout(5) + yawRateRadps * ageS_in);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
