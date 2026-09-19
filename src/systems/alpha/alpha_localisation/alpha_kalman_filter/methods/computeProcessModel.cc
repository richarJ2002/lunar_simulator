/*!
 * @File:         computeProcessModel.cc
 *
 * @Brief:        Implements Alpha's Euler-rate process model and its
 *                Jacobian, evaluated at one state.
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

void AlphaKalmanFilterNode::computeProcessModel(
    const StateVector &state_in, StateVector &stateDerivative_out,
    StateMatrix &processJacobian_out) const
{
    /* Current roll state, used throughout this evaluation. */
    const double rollRad = state_in(3);

    /* Current pitch state, clamped away from the tan/sec singularity at
     * +/-pi/2. */
    const double pitchRad = std::clamp(state_in(4), -1.5533, 1.5533);

    /* sin(roll), reused by the roll-rate/yaw-rate equations and their
     * Jacobian entries below. */
    const double sineRoll = std::sin(rollRad);

    /* cos(roll), reused the same way. */
    const double cosineRoll = std::cos(rollRad);

    /* tan(pitch), used by the roll-rate equation and its Jacobian. */
    const double tangentPitch = std::tan(pitchRad);

    /* cos(pitch), used only to derive secantPitch below. */
    const double cosinePitch = std::cos(pitchRad);

    /* sec(pitch) = 1 / cos(pitch), used by the yaw-rate equation and its
     * Jacobian. */
    const double secantPitch = 1.0 / cosinePitch;

    /* Body-frame roll rate state. */
    const double angularRateXRadps = state_in(9);

    /* Body-frame pitch rate state. */
    const double angularRateYRadps = state_in(10);

    /* Body-frame yaw rate state. */
    const double angularRateZRadps = state_in(11);

    /*!
     * Euler-rate process model, with body angular rate omega = [p q r]^T:
     * rollDot  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
     * pitchDot = cos(roll) q - sin(roll) r
     * yawDot   = sin(roll) sec(pitch) q + cos(roll) sec(pitch) r
     * Angles are radians, angular rates radians/second, and multiplication is
     * scalar left-to-right exactly as represented below.
     */
    stateDerivative_out = StateVector::Zero();

    /* Position's derivative is simply the linear-velocity state. */
    stateDerivative_out.segment<3>(0) = state_in.segment<3>(6);

    /* Evaluate the rollDot equation above. */
    stateDerivative_out(3) = angularRateXRadps +
                             sineRoll * tangentPitch * angularRateYRadps +
                             cosineRoll * tangentPitch * angularRateZRadps;

    /* Evaluate the pitchDot equation above. */
    stateDerivative_out(4) =
        cosineRoll * angularRateYRadps - sineRoll * angularRateZRadps;

    /* Evaluate the yawDot equation above. */
    stateDerivative_out(5) = sineRoll * secantPitch * angularRateYRadps +
                             cosineRoll * secantPitch * angularRateZRadps;

    /*!
     * The six per-wheel slip states (12-17) do not follow the pose/twist
     * kinematics above: each mean-reverts toward zero at slipDecayRateHz
     * absent fresh evidence for that wheel (slipDot_w = -slipDecayRateHz *
     * slip_w), so a stale or spuriously-fused slip observation relaxes
     * back toward "no slip assumed" for that wheel rather than persisting
     * indefinitely between handleSlipObservationCallBack() corrections.
     * One shared decay rate is used for all six wheels; there is no
     * physical reason to expect it to differ per wheel.
     */
    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        const Eigen::Index index = SLIP_STATE_START_INDEX + wheel;

        stateDerivative_out(index) = -slipDecayRateHz * state_in(index);
    }

    /* Jacobian of the process model above, needed by the engine to
     * propagate covariance through the continuous covariance equation. */
    processJacobian_out = StateMatrix::Zero();

    /* Position's derivative depends linearly on linear velocity with
     * unit coefficient, so this 3x3 block is simply the identity. */
    processJacobian_out.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();

    /* Partial derivative of each wheel's slip decay above with respect to
     * its own slip state; each wheel's slip state is independent of every
     * other wheel's, so this stays diagonal. */
    for (Eigen::Index wheel = 0; wheel < WHEEL_COUNT; ++wheel)
    {
        const Eigen::Index index = SLIP_STATE_START_INDEX + wheel;

        processJacobian_out(index, index) = -slipDecayRateHz;
    }

    /* Partial derivative of rollDot with respect to roll. */
    processJacobian_out(3, 3) =
        cosineRoll * tangentPitch * angularRateYRadps -
        sineRoll * tangentPitch * angularRateZRadps;

    /* Partial derivative of rollDot with respect to pitch; uses
     * d(tan)/d(pitch) = sec^2(pitch). */
    processJacobian_out(3, 4) =
        sineRoll * secantPitch * secantPitch * angularRateYRadps +
        cosineRoll * secantPitch * secantPitch * angularRateZRadps;

    /* Partial derivative of rollDot with respect to the roll-rate
     * state. */
    processJacobian_out(3, 9) = 1.0;

    /* Partial derivative of rollDot with respect to the pitch-rate
     * state. */
    processJacobian_out(3, 10) = sineRoll * tangentPitch;

    /* Partial derivative of rollDot with respect to the yaw-rate
     * state. */
    processJacobian_out(3, 11) = cosineRoll * tangentPitch;

    /* Partial derivative of pitchDot with respect to roll. */
    processJacobian_out(4, 3) =
        -sineRoll * angularRateYRadps - cosineRoll * angularRateZRadps;

    /* Partial derivative of pitchDot with respect to the pitch-rate
     * state. */
    processJacobian_out(4, 10) = cosineRoll;

    /* Partial derivative of pitchDot with respect to the yaw-rate
     * state. */
    processJacobian_out(4, 11) = -sineRoll;

    /* Partial derivative of yawDot with respect to roll. */
    processJacobian_out(5, 3) =
        cosineRoll * secantPitch * angularRateYRadps -
        sineRoll * secantPitch * angularRateZRadps;

    /* Partial derivative of yawDot with respect to pitch; uses
     * d(sec)/d(pitch) = sec(pitch) tan(pitch). */
    processJacobian_out(5, 4) =
        secantPitch * tangentPitch *
        (sineRoll * angularRateYRadps + cosineRoll * angularRateZRadps);

    /* Partial derivative of yawDot with respect to the pitch-rate
     * state. */
    processJacobian_out(5, 10) = sineRoll * secantPitch;

    /* Partial derivative of yawDot with respect to the yaw-rate state. */
    processJacobian_out(5, 11) = cosineRoll * secantPitch;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
