/*!
 * @file            solveBodyTwist.cc
 *
 * @brief           Implements the twelve-row weighted least-squares wheel
 *                  kinematic solve.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "public_functions.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <cstddef>

/* External Library Includes */
#include <Eigen/Eigenvalues>

namespace localisation::wheel_odometry
{

BodyTwistSolution
    solveBodyTwist(const std::array<double, 6> &steeringAngle_rad_in,
                   const std::array<double, 6> &rollingSpeed_mps_in,
                   const std::array<double, 6> &wheelX_m_in,
                   const std::array<double, 6> &wheelY_m_in,
                   double                       rollingSpeedStddevMps_in,
                   double                       steeringStddevRad_in,
                   double                       lateralSlipStddevMps_in)
{
    BodyTwistSolution solution;
    if (!(rollingSpeedStddevMps_in > 0.0) || !(lateralSlipStddevMps_in > 0.0) ||
        !(steeringStddevRad_in >= 0.0))
    {
        return solution;
    }

    /* Accumulate the weighted normal equations row by row; the 12x3
     * design matrix itself is never needed. */
    Eigen::Matrix3d normalMatrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d normalVector = Eigen::Vector3d::Zero();
    const double    rollingWeight =
        1.0 / (rollingSpeedStddevMps_in * rollingSpeedStddevMps_in);
    for (std::size_t wheel = 0U; wheel < steeringAngle_rad_in.size(); ++wheel)
    {
        const double cosine          = std::cos(steeringAngle_rad_in[wheel]);
        const double sine            = std::sin(steeringAngle_rad_in[wheel]);
        const double x_m             = wheelX_m_in[wheel];
        const double y_m             = wheelY_m_in[wheel];
        const double rollingSpeedMps = rollingSpeed_mps_in[wheel];

        /* Rolling row: the wheel's speed along its heading. */
        const Eigen::Vector3d rollingRow(cosine,
                                         sine,
                                         x_m * sine - y_m * cosine);
        normalMatrix += rollingWeight * rollingRow * rollingRow.transpose();
        normalVector += rollingWeight * rollingRow * rollingSpeedMps;

        /* No-side-slip row: zero speed across the heading, with steering
         * noise projected through d(row)/d(steering) = -rolling speed. */
        const Eigen::Vector3d lateralRow(-sine,
                                         cosine,
                                         x_m * cosine + y_m * sine);
        const double          steeringInducedStddevMps =
            steeringStddevRad_in * rollingSpeedMps;
        const double lateralVariance =
            lateralSlipStddevMps_in * lateralSlipStddevMps_in +
            steeringInducedStddevMps * steeringInducedStddevMps;
        normalMatrix += lateralRow * lateralRow.transpose() / lateralVariance;
    }
    if (!normalMatrix.allFinite() || !normalVector.allFinite())
    {
        return solution;
    }

    /* A symmetric positive-definite normal matrix is required for both
     * the solve and the covariance. */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        normalMatrix,
        Eigen::EigenvaluesOnly);
    if (eigenSolver.info() != Eigen::Success ||
        !(eigenSolver.eigenvalues().minCoeff() > 0.0))
    {
        return solution;
    }
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(normalMatrix);
    if (decomposition.info() != Eigen::Success)
    {
        return solution;
    }
    solution.twist_body      = decomposition.solve(normalVector);
    solution.covariance_body = decomposition.solve(Eigen::Matrix3d::Identity());
    solution.covariance_body =
        0.5 * (solution.covariance_body + solution.covariance_body.transpose());
    solution.normalCondition = eigenSolver.eigenvalues().maxCoeff() /
                               eigenSolver.eigenvalues().minCoeff();
    solution.isValid =
        solution.twist_body.allFinite() && solution.covariance_body.allFinite();
    return solution;
}

} /* namespace localisation::wheel_odometry */
