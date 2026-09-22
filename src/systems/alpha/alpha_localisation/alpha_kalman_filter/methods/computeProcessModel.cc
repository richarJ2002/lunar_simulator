/**
 * @file            computeProcessModel.cc
 *
 * @brief           Applies the configured gravity vector to process modeling.
 *
 * @date            22/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::computeProcessModel(
    const NominalStateVector &state_in,
    const Eigen::Vector3d    &specificForceBodyMps2_in,
    const Eigen::Vector3d    &angularVelocityBodyRadPerS_in,
    double                    timeStepS_in,
    NominalStateVector       &predictedState_out,
    ErrorStateMatrix         &processJacobian_out) const
{
    calculateProcessModel(state_in,
                          specificForceBodyMps2_in,
                          angularVelocityBodyRadPerS_in,
                          gravityAcceleration_fixed_mPerS2,
                          timeStepS_in,
                          predictedState_out,
                          processJacobian_out);
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
