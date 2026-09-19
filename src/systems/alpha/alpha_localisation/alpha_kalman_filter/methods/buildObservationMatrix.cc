/*!
 * @File:         buildObservationMatrix.cc
 *
 * @Brief:        Implements one-hot observation-matrix construction from a
 *                list of directly-observed state indices.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
/* None */

/* Object Include */
#include "objects/AlphaKalmanFilterNode.h"

/* Generic Libraries */
/* None */

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

Eigen::MatrixXd AlphaKalmanFilterNode::buildObservationMatrix(
    const std::vector<Eigen::Index> &observedIndices_in,
    Eigen::Index stateSize_in)
{
    /* One row per observed index, one column per state; row i observes
     * state observedIndices_in[i] with unit coefficient. */
    Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(observedIndices_in.size()), stateSize_in);

    for (std::size_t row = 0; row < observedIndices_in.size(); ++row)
    {
        observation(static_cast<Eigen::Index>(row), observedIndices_in[row]) =
            1.0;
    }

    return observation;
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
