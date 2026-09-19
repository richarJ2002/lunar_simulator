/*!
 * @File:         setNoiseTriplet.cc
 *
 * @Brief:        Implements one three-axis process-noise diagonal write.
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

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::setNoiseTriplet(StateMatrix &noise_inout,
                                            Eigen::Index firstIndex_in,
                                            double variance_in)
{
    /*!
     * The EKF state groups position, orientation, linear velocity and
     * angular velocity into consecutive triplets of axes; each configured
     * process-noise parameter applies uniformly across one triplet's three
     * diagonal entries. A small positive floor keeps the process-noise
     * matrix strictly positive definite even if a variance parameter is
     * configured to zero.
     */
    for (Eigen::Index index = firstIndex_in; index < firstIndex_in + 3;
         ++index)
    {
        /* Write the floored variance onto this axis's diagonal entry. */
        noise_inout(index, index) = std::max(1.0e-9, variance_in);
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
