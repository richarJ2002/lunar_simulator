/*!
 * @File:         test_kalman_math.cpp
 *
 * @Brief:        Tests the reusable, model-agnostic continuous extended
 *                Kalman filter engine.
 *
 * @Date:         17/09/2026
 *
 */

/* Function Includes */
#include <gtest/gtest.h>

/* Object Include */
#include "objects/ContinuousExtendedKalmanFilter.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
#include <array>
#include <cmath>

namespace
{

using Filter = localisation::kalman_filter::ekf_continuous_kalman_filter::
    ContinuousExtendedKalmanFilter;
using FilterStatus =
    localisation::kalman_filter::ekf_continuous_kalman_filter::FilterStatus;

TEST(KalmanMath, PredictAndUpdateRequireInitializeFirst)
{
    Filter filter;
    const Eigen::VectorXd derivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, 2);
    const Eigen::MatrixXd noise = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_EQ(filter.predict(0.1, derivative, jacobian, noise),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);

    const Eigen::VectorXd innovation = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_EQ(filter.update(innovation, observation, noise),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);
}

TEST(KalmanMath, TerminateReturnsToUninitializedState)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(2, Eigen::VectorXd::Zero(2),
                                Eigen::MatrixXd::Identity(2, 2)),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(filter.terminate(), FilterStatus::FILTER_STATUS_SUCCESS);

    const Eigen::VectorXd derivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(2, 2);
    const Eigen::MatrixXd noise = Eigen::MatrixXd::Identity(2, 2);
    EXPECT_EQ(filter.predict(0.1, derivative, jacobian, noise),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);
}

TEST(KalmanMath, PredictsConstantLinearVelocity)
{
    Filter filter;

    /* A two-state [position, velocity] constant-velocity model: the
     * caller (not the engine) evaluates f(x) = [velocity, 0] and its
     * Jacobian F = [[0, 1], [0, 0]] at the current state. */
    Eigen::VectorXd initialState(2);
    initialState << 1.0, 2.0;
    ASSERT_EQ(filter.initialize(2, initialState,
                                Eigen::MatrixXd::Identity(2, 2)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Eigen::VectorXd stateDerivative(2);
    stateDerivative << 2.0, 0.0;
    Eigen::MatrixXd processJacobian = Eigen::MatrixXd::Zero(2, 2);
    processJacobian(0, 1) = 1.0;
    const Eigen::MatrixXd processNoise = Eigen::MatrixXd::Zero(2, 2);

    ASSERT_EQ(filter.predict(0.5, stateDerivative, processJacobian,
                             processNoise),
              FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_NEAR(filter.getState()(0), 2.0, 1.0e-10);
    EXPECT_NEAR(filter.getState()(1), 2.0, 1.0e-10);
    EXPECT_TRUE(
        filter.getCovariance().isApprox(filter.getCovariance().transpose(),
                                        1.0e-12));
}

TEST(KalmanMath, CorrectsOnlySelectedMeasurementStates)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(3, Eigen::VectorXd::Zero(3),
                                Eigen::MatrixXd::Identity(3, 3)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    /* Observation matrix selecting only states 0 and 1, leaving state 2
     * entirely unobserved -- matching the old MeasurementMask{0,1}
     * behaviour without needing a dummy zero-effect measurement channel
     * for the unobserved state. */
    Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(2, 3);
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;

    Eigen::VectorXd innovation(2);
    innovation << 10.0, -4.0;
    const Eigen::MatrixXd measurementNoise = Eigen::MatrixXd::Identity(2, 2);

    ASSERT_EQ(filter.update(innovation, observation, measurementNoise),
              FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_NEAR(filter.getState()(0), 5.0, 1.0e-12);
    EXPECT_NEAR(filter.getState()(1), -2.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(filter.getState()(2), 0.0);
    EXPECT_LT(filter.getCovariance()(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(filter.getCovariance()(2, 2), 1.0);
}

TEST(KalmanMath, UsesIndependentMeasurementVariances)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(2, Eigen::VectorXd::Zero(2),
                                Eigen::MatrixXd::Identity(2, 2)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd innovation(2);
    innovation << 10.0, 10.0;
    Eigen::MatrixXd measurementNoise = Eigen::MatrixXd::Zero(2, 2);
    measurementNoise(0, 0) = 0.01;
    measurementNoise(1, 1) = 100.0;

    ASSERT_EQ(filter.update(innovation, observation, measurementNoise),
              FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_GT(filter.getState()(0), 9.0);
    EXPECT_LT(filter.getState()(1), 0.2);
}

TEST(KalmanMath, RejectsMismatchedMatrixDimensions)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(3, Eigen::VectorXd::Zero(3),
                                Eigen::MatrixXd::Identity(3, 3)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    /*!
     * With fixed-size Eigen types this class of error used to be caught
     * at compile time; dynamic sizing makes it a real runtime failure
     * mode the engine must now detect itself.
     */
    const Eigen::VectorXd wrongSizeDerivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd rightSizeJacobian = Eigen::MatrixXd::Zero(3, 3);
    const Eigen::MatrixXd rightSizeNoise = Eigen::MatrixXd::Identity(3, 3);
    EXPECT_EQ(filter.predict(0.1, wrongSizeDerivative, rightSizeJacobian,
                             rightSizeNoise),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);

    const Eigen::VectorXd innovation = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd wrongColumnsObservation =
        Eigen::MatrixXd::Zero(2, 4);
    const Eigen::MatrixXd rightSizeMeasurementNoise =
        Eigen::MatrixXd::Identity(2, 2);
    EXPECT_EQ(filter.update(innovation, wrongColumnsObservation,
                            rightSizeMeasurementNoise),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);
}

TEST(KalmanMath, SixWheelRollingSystemRecoversPlanarTwist)
{
    const std::array<double, 6> wheelXM{0.64, 0.64, 0.0, 0.0, -0.72, -0.72};
    const std::array<double, 6> wheelYM{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};
    const Eigen::Vector3d expectedTwist(0.4, 0.05, 0.2);
    Eigen::Matrix<double, 6, 3> rollingMatrix;
    for (std::size_t index = 0U; index < wheelXM.size(); ++index)
    {
        const double steeringRad =
            std::atan2(expectedTwist.y() + wheelXM[index] * expectedTwist.z(),
                       expectedTwist.x() - wheelYM[index] * expectedTwist.z());
        rollingMatrix(static_cast<Eigen::Index>(index), 0) =
            std::cos(steeringRad);
        rollingMatrix(static_cast<Eigen::Index>(index), 1) =
            std::sin(steeringRad);
        rollingMatrix(static_cast<Eigen::Index>(index), 2) =
            -wheelYM[index] * std::cos(steeringRad) +
            wheelXM[index] * std::sin(steeringRad);
    }
    const Eigen::Matrix<double, 6, 1> wheelSpeeds =
        rollingMatrix * expectedTwist;
    const Eigen::Vector3d recoveredTwist =
        rollingMatrix.colPivHouseholderQr().solve(wheelSpeeds);

    EXPECT_TRUE(recoveredTwist.isApprox(expectedTwist, 1.0e-10));
}

TEST(KalmanMath, ParallelWheelsDoNotCreateUnobservableLateralVelocity)
{
    const std::array<double, 6> wheelYM{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};
    Eigen::Matrix<double, 6, 3> rollingMatrix;
    for (std::size_t index = 0U; index < wheelYM.size(); ++index)
    {
        rollingMatrix(static_cast<Eigen::Index>(index), 0) = 1.0;
        rollingMatrix(static_cast<Eigen::Index>(index), 1) = 0.0;
        rollingMatrix(static_cast<Eigen::Index>(index), 2) = -wheelYM[index];
    }
    const Eigen::Matrix<double, 6, 1> wheelSpeeds =
        Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Vector3d recoveredTwist =
        rollingMatrix.completeOrthogonalDecomposition().solve(wheelSpeeds);

    EXPECT_TRUE(recoveredTwist.isZero(1.0e-12));
}

TEST(KalmanMath, NumericalSteeringNoiseDoesNotCreateLateralVelocity)
{
    const std::array<double, 6> wheelYM{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};
    Eigen::Matrix<double, 6, 3> rollingMatrix;
    Eigen::Matrix<double, 6, 1> wheelSpeeds;
    for (std::size_t index = 0U; index < wheelYM.size(); ++index)
    {
        const double steeringNoiseRad =
            4.0e-12 + static_cast<double>(index) * 1.0e-15;
        rollingMatrix(static_cast<Eigen::Index>(index), 0) =
            std::cos(steeringNoiseRad);
        rollingMatrix(static_cast<Eigen::Index>(index), 1) =
            std::sin(steeringNoiseRad);
        rollingMatrix(static_cast<Eigen::Index>(index), 2) =
            -wheelYM[index] * std::cos(steeringNoiseRad);
        wheelSpeeds(static_cast<Eigen::Index>(index)) =
            (static_cast<double>(index) - 2.5) * 1.0e-14;
    }
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 3>>
        rollingDecomposition(rollingMatrix);
    rollingDecomposition.setThreshold(1.0e-6);
    const Eigen::Vector3d recoveredTwist =
        rollingDecomposition.solve(wheelSpeeds);

    EXPECT_NEAR(recoveredTwist.y(), 0.0, 1.0e-12);
}

TEST(KalmanMath, VisualRollingSpeedRecoversWheelSlipRatio)
{
    constexpr double expectedSlipRatio = 0.20;
    constexpr double wheelXM = 0.64;
    constexpr double wheelYM = 0.60;
    constexpr double steeringRad = 0.31;
    const Eigen::Vector3d visualTwistBody(0.18, 0.03, 0.07);
    const double expectedRollingSpeedMps =
        std::cos(steeringRad) *
            (visualTwistBody.x() - wheelYM * visualTwistBody.z()) +
        std::sin(steeringRad) *
            (visualTwistBody.y() + wheelXM * visualTwistBody.z());
    const double rawWheelSpeedMps =
        expectedRollingSpeedMps / (1.0 - expectedSlipRatio);
    const double observedSlipRatio =
        1.0 - expectedRollingSpeedMps / rawWheelSpeedMps;

    EXPECT_NEAR(observedSlipRatio, expectedSlipRatio, 1.0e-12);
    EXPECT_NEAR(rawWheelSpeedMps * (1.0 - observedSlipRatio),
                expectedRollingSpeedMps, 1.0e-12);
}

} /* namespace */
