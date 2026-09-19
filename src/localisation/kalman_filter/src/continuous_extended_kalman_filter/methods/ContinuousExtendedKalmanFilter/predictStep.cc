/*!
 * @File:         predictStep.cc
 *
 * @Brief:        Applies one continuous EKF process-model step.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "kalman_filter/objects/ContinuousExtendedKalmanFilter.h"

/* Data include */
/* None */

/* Generic Libraries */
#include <algorithm>
#include <cmath>

namespace lunar_simulator::localisation::kalman_filter
{

FilterStatus
ContinuousExtendedKalmanFilter::predictStep(double timeStepS_in) noexcept
{
    const double rollRad = state(3);
    const double pitchRad = std::clamp(state(4), -1.5533, 1.5533);
    const double sineRoll = std::sin(rollRad);
    const double cosineRoll = std::cos(rollRad);
    const double tangentPitch = std::tan(pitchRad);
    const double cosinePitch = std::cos(pitchRad);
    const double secantPitch = 1.0 / cosinePitch;
    const double angularRateXRadps = state(9);
    const double angularRateYRadps = state(10);
    const double angularRateZRadps = state(11);

    /*
     * Euler-rate process model, with body angular rate omega = [p q r]^T:
     * rollDot  = p + sin(roll) tan(pitch) q + cos(roll) tan(pitch) r
     * pitchDot = cos(roll) q - sin(roll) r
     * yawDot   = sin(roll) sec(pitch) q + cos(roll) sec(pitch) r
     * Angles are radians, angular rates radians/second, and multiplication is
     * scalar left-to-right exactly as represented below.
     */
    StateVector derivative = StateVector::Zero();
    derivative.segment<3>(0) = state.segment<3>(6);
    derivative(3) = angularRateXRadps +
                    sineRoll * tangentPitch * angularRateYRadps +
                    cosineRoll * tangentPitch * angularRateZRadps;
    derivative(4) =
        cosineRoll * angularRateYRadps - sineRoll * angularRateZRadps;
    derivative(5) = sineRoll * secantPitch * angularRateYRadps +
                    cosineRoll * secantPitch * angularRateZRadps;

    StateMatrix processJacobian = StateMatrix::Zero();
    processJacobian.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    processJacobian(3, 3) = cosineRoll * tangentPitch * angularRateYRadps -
                            sineRoll * tangentPitch * angularRateZRadps;
    processJacobian(3, 4) =
        sineRoll * secantPitch * secantPitch * angularRateYRadps +
        cosineRoll * secantPitch * secantPitch * angularRateZRadps;
    processJacobian(3, 9) = 1.0;
    processJacobian(3, 10) = sineRoll * tangentPitch;
    processJacobian(3, 11) = cosineRoll * tangentPitch;
    processJacobian(4, 3) =
        -sineRoll * angularRateYRadps - cosineRoll * angularRateZRadps;
    processJacobian(4, 10) = cosineRoll;
    processJacobian(4, 11) = -sineRoll;
    processJacobian(5, 3) = cosineRoll * secantPitch * angularRateYRadps -
                            sineRoll * secantPitch * angularRateZRadps;
    processJacobian(5, 4) =
        secantPitch * tangentPitch *
        (sineRoll * angularRateYRadps + cosineRoll * angularRateZRadps);
    processJacobian(5, 10) = sineRoll * secantPitch;
    processJacobian(5, 11) = cosineRoll * secantPitch;

    state += derivative * timeStepS_in;
    state(3) = wrapAngle(state(3));
    state(4) = wrapAngle(state(4));
    state(5) = wrapAngle(state(5));

    /* Continuous covariance equation: Pdot = F P + P F^T + Q. */
    covariance += (processJacobian * covariance +
                   covariance * processJacobian.transpose() + processNoise) *
                  timeStepS_in;
    covariance = 0.5 * (covariance + covariance.transpose());

    if (!state.allFinite() || !covariance.allFinite())
    {
        return FilterStatus::FILTER_STATUS_NUMERICAL_FAILURE;
    }
    return FilterStatus::FILTER_STATUS_SUCCESS;
}

} /* namespace lunar_simulator::localisation::kalman_filter */
