/*!
 * @file            solveBodyTwist.cc
 *
 * @brief           Implements the twelve-row weighted least-squares wheel
 *                  kinematic solve.
 *
 * @date            25/09/2026
 */

/* Matching Declaration Include */
#include "public_functions/public_functions.h"

/* C++ Standard Library Includes */
#include <cmath>
#include <cstddef>

/* External Library Includes */
#include <Eigen/Eigenvalues>

namespace localisation::wheel_odometry
{

BodyTwistSolution
    solveBodyTwist(const std::array<double, 6> &steeringAngle_rad_in,
                   const std::array<double, 6> &rollingSpeed_mPs_in,
                   const std::array<double, 6> &wheelX_m_in,
                   const std::array<double, 6> &wheelY_m_in,
                   double                       rollingSpeedStddev_mPs_in,
                   double                       steeringStddev_rad_in,
                   double                       lateralSlipStddev_mPs_in)
{
    /* Start invalid; every early return below keeps it so. */
    BodyTwistSolution solution;

    /* Reject non-physical noise scales before building the system. */
    if (!(rollingSpeedStddev_mPs_in > 0.0) || !(lateralSlipStddev_mPs_in > 0.0) ||
        !(steeringStddev_rad_in >= 0.0))
    {
        return solution;
    }

    /* Seed the 3x3 normal equations and the rolling-row weight. */
    Eigen::Matrix3d normalMatrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d normalVector = Eigen::Vector3d::Zero();
    const double    rollingWeight =
        1.0 / (rollingSpeedStddev_mPs_in * rollingSpeedStddev_mPs_in);

    /*!
     * Accumulate the weighted normal equations row by row; the 12x3 design
     * matrix itself is never needed.
     */
    for (std::size_t wheel = 0U; wheel < steeringAngle_rad_in.size(); wheel++)
    {
        /* Cache this wheel's steering trigonometry and inputs. */
        const double cosine          = std::cos(steeringAngle_rad_in[wheel]);
        const double sine            = std::sin(steeringAngle_rad_in[wheel]);
        const double x_m             = wheelX_m_in[wheel];
        const double y_m             = wheelY_m_in[wheel];
        const double rollingSpeed_mPs = rollingSpeed_mPs_in[wheel];

        /* Rolling row: the wheel's speed along its heading. */
        const Eigen::Vector3d rollingRow(cosine,
                                         sine,
                                         x_m * sine - y_m * cosine);
        normalMatrix += rollingWeight * rollingRow * rollingRow.transpose();
        normalVector += rollingWeight * rollingRow * rollingSpeed_mPs;

        /*!
         * No-side-slip row: zero speed across the heading, with steering
         * noise projected through d(row)/d(steering) = -rolling speed.
         */
        const Eigen::Vector3d lateralRow(-sine,
                                         cosine,
                                         x_m * cosine + y_m * sine);

        /* Project steering noise into a rolling-speed deviation. */
        const double steeringInducedStddev_mPs =
            steeringStddev_rad_in * rollingSpeed_mPs;

        /* Combine slip and steering-induced lateral variance. */
        const double lateralVariance =
            lateralSlipStddev_mPs_in * lateralSlipStddev_mPs_in +
            steeringInducedStddev_mPs * steeringInducedStddev_mPs;

        /* Fold this wheel's no-side-slip row into the equations. */
        normalMatrix += lateralRow * lateralRow.transpose() / lateralVariance;
    }

    /* Bail out if accumulation produced non-finite equations. */
    if (!normalMatrix.allFinite() || !normalVector.allFinite())
    {
        return solution;
    }

    /*!
     * A symmetric positive-definite normal matrix is required for both
     * the solve and the covariance.
     */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        normalMatrix,
        Eigen::EigenvaluesOnly);

    /* Require success with an all-positive eigenvalue spectrum. */
    if (eigenSolver.info() != Eigen::Success ||
        !(eigenSolver.eigenvalues().minCoeff() > 0.0))
    {
        return solution;
    }

    /* Factor the normal matrix; bail out if it fails. */
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(normalMatrix);
    if (decomposition.info() != Eigen::Success)
    {
        return solution;
    }

    /* Solve for twist and covariance, then symmetrize and validate. */
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
