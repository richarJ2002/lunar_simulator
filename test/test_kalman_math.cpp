/*!
 * @File:         test_kalman_math.cpp
 *
 * @Brief:        Tests the reusable continuous extended Kalman filter.
 *
 * @Date:         15/09/2026
 *
 */

/* Function Includes */
#include <gtest/gtest.h>

/* Object Include */
#include "kalman_filter/objects/ContinuousExtendedKalmanFilter.h"

/* Data include */
#include <Eigen/Dense>

/* Generic Libraries */
#include <array>
#include <cmath>

namespace
{

using Filter = lunar_simulator::localisation::kalman_filter::
    ContinuousExtendedKalmanFilter;
using FilterStatus = lunar_simulator::localisation::kalman_filter::FilterStatus;

Filter::StateVector uniformVariance(double variance_in)
{
    return Filter::StateVector::Constant(variance_in);
}

TEST(KalmanMath, RequiresInitBeforeStep)
{
    Filter filter;
    Filter::StateVector state;
    Filter::StateMatrix covariance;
    const Filter::MeasurementMask mask{};

    EXPECT_EQ(filter.step(0.0, nullptr, mask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);
}

TEST(KalmanMath, PredictsConstantLinearVelocity)
{
    Filter filter;
    Filter::StateMatrix processNoise = Filter::StateMatrix::Zero();
    ASSERT_EQ(filter.init(processNoise, Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector measurement = Filter::StateVector::Zero();
    measurement(0) = 1.0;
    measurement(6) = 2.0;
    Filter::MeasurementMask fullMask{};
    fullMask.fill(true);
    Filter::StateVector state;
    Filter::StateMatrix covariance;
    ASSERT_EQ(filter.step(0.0, &measurement, fullMask, uniformVariance(0.1),
                          state, covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    const Filter::MeasurementMask emptyMask{};
    ASSERT_EQ(filter.step(0.5, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_NEAR(state(0), 2.0, 1.0e-10);
    EXPECT_NEAR(state(6), 2.0, 1.0e-10);
    EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1.0e-12));
}

TEST(KalmanMath, TerminateReturnsToUninitializedState)
{
    Filter filter;
    ASSERT_EQ(filter.init(Filter::StateMatrix::Zero(),
                          Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(filter.terminate(), FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector state;
    Filter::StateMatrix covariance;
    const Filter::MeasurementMask mask{};
    EXPECT_EQ(filter.step(0.0, nullptr, mask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);
}

TEST(KalmanMath, CorrectsOnlySelectedMeasurementStates)
{
    Filter filter;
    ASSERT_EQ(filter.init(Filter::StateMatrix::Zero(),
                          Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector state;
    Filter::StateMatrix covariance;
    const Filter::MeasurementMask emptyMask{};
    ASSERT_EQ(filter.step(0.0, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector measurement = Filter::StateVector::Zero();
    measurement(0) = 10.0;
    measurement(1) = -4.0;
    measurement(2) = 99.0;
    Filter::MeasurementMask planarMask{};
    planarMask[0] = true;
    planarMask[1] = true;
    ASSERT_EQ(
        filter.step(0.0, &measurement, planarMask, uniformVariance(1.0), state,
                    covariance),
        FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_NEAR(state(0), 5.0, 1.0e-12);
    EXPECT_NEAR(state(1), -2.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(state(2), 0.0);
    EXPECT_LT(covariance(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(covariance(2, 2), 1.0);
}

TEST(KalmanMath, RejectsNonMonotonicTimestamp)
{
    Filter filter;
    ASSERT_EQ(filter.init(Filter::StateMatrix::Zero(),
                          Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector state;
    Filter::StateMatrix covariance;
    const Filter::MeasurementMask emptyMask{};
    ASSERT_EQ(filter.step(2.0, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_EQ(filter.step(1.0, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);
}

TEST(KalmanMath, UsesIndependentMeasurementVariances)
{
    Filter filter;
    ASSERT_EQ(filter.init(Filter::StateMatrix::Zero(),
                          Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Filter::StateVector measurement = Filter::StateVector::Zero();
    measurement(0) = 10.0;
    measurement(1) = 10.0;
    Filter::MeasurementMask mask{};
    mask[0] = true;
    mask[1] = true;
    Filter::StateVector variances = Filter::StateVector::Ones();
    variances(0) = 0.01;
    variances(1) = 100.0;
    Filter::StateVector state;
    Filter::StateMatrix covariance;
    const Filter::MeasurementMask emptyMask{};
    ASSERT_EQ(filter.step(0.0, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    ASSERT_EQ(filter.step(0.0, &measurement, mask, variances, state, covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_GT(state(0), 9.0);
    EXPECT_LT(state(1), 0.2);
}

TEST(KalmanMath, PredictsAcrossLongTimestampGap)
{
    Filter filter;
    ASSERT_EQ(filter.init(Filter::StateMatrix::Zero(),
                          Filter::StateMatrix::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);
    Filter::StateVector measurement = Filter::StateVector::Zero();
    measurement(6) = 1.0;
    Filter::MeasurementMask fullMask{};
    fullMask.fill(true);
    Filter::StateVector state;
    Filter::StateMatrix covariance;
    ASSERT_EQ(filter.step(0.0, &measurement, fullMask, uniformVariance(0.01),
                          state, covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    const Filter::MeasurementMask emptyMask{};
    ASSERT_EQ(filter.step(1.5, nullptr, emptyMask, uniformVariance(1.0), state,
                          covariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_NEAR(state(0), 1.5, 1.0e-10);
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
