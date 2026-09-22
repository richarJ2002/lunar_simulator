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
#include "objects/AlphaKalmanFilterNode.h"
#include "objects/ContinuousExtendedKalmanFilter.h"
#include "objects/ErrorStateIndex.h"
#include "objects/StateIndex.h"

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
using AlphaFilter = systems::alpha::alpha_localisation::alpha_kalman_filter::
    AlphaKalmanFilterNode;
using StateIndex =
    systems::alpha::alpha_localisation::alpha_kalman_filter::StateIndex;
using ErrorStateIndex =
    systems::alpha::alpha_localisation::alpha_kalman_filter::ErrorStateIndex;

Eigen::Quaterniond
    rotationVectorToQuaternion(const Eigen::Vector3d &rotationVector_in)
{
    const double magnitude = rotationVector_in.norm();
    if (magnitude < 1.0e-15)
    {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(magnitude, rotationVector_in / magnitude));
}

Eigen::Vector3d
    quaternionToRotationVector(const Eigen::Quaterniond &quaternion_in)
{
    const Eigen::AngleAxisd angleAxis(quaternion_in.normalized());
    return angleAxis.axis() * angleAxis.angle();
}

AlphaFilter::NominalStateVector makeEskfState()
{
    AlphaFilter::NominalStateVector state =
        AlphaFilter::NominalStateVector::Zero();
    const Eigen::Index quaternionWIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_W);
    state(quaternionWIndex) = 1.0;
    return state;
}

AlphaFilter::NominalStateVector
    injectEskfError(const AlphaFilter::NominalStateVector &state_in,
                    const AlphaFilter::ErrorStateVector   &error_in)
{
    AlphaFilter::NominalStateVector state = state_in;
    const Eigen::Index              positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAccelerometerBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    state.segment<3>(positionIndex) += error_in.segment<3>(errorPositionIndex);
    state.segment<3>(velocityIndex) += error_in.segment<3>(errorVelocityIndex);
    state.segment<3>(accelerometerBiasIndex) +=
        error_in.segment<3>(errorAccelerometerBiasIndex);
    state.segment<3>(gyroscopeBiasIndex) +=
        error_in.segment<3>(errorGyroscopeBiasIndex);
    Eigen::Quaterniond quaternion(state(quaternionIndex + 3),
                                  state(quaternionIndex),
                                  state(quaternionIndex + 1),
                                  state(quaternionIndex + 2));
    quaternion = quaternion * rotationVectorToQuaternion(
                                  error_in.segment<3>(errorAttitudeIndex));
    quaternion.normalize();
    state(quaternionIndex)     = quaternion.x();
    state(quaternionIndex + 1) = quaternion.y();
    state(quaternionIndex + 2) = quaternion.z();
    state(quaternionIndex + 3) = quaternion.w();
    return state;
}

AlphaFilter::ErrorStateVector
    calculateEskfError(const AlphaFilter::NominalStateVector &reference_in,
                       const AlphaFilter::NominalStateVector &perturbed_in)
{
    AlphaFilter::ErrorStateVector error = AlphaFilter::ErrorStateVector::Zero();
    const Eigen::Index            positionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_POSITION_X);
    const Eigen::Index quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    const Eigen::Index errorPositionIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_POSITION_X);
    const Eigen::Index errorAttitudeIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ATTITUDE_X);
    const Eigen::Index errorVelocityIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index errorAccelerometerBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index errorGyroscopeBiasIndex = static_cast<Eigen::Index>(
        ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_X);

    error.segment<3>(errorPositionIndex) =
        perturbed_in.segment<3>(positionIndex) -
        reference_in.segment<3>(positionIndex);
    error.segment<3>(errorVelocityIndex) =
        perturbed_in.segment<3>(velocityIndex) -
        reference_in.segment<3>(velocityIndex);
    error.segment<3>(errorAccelerometerBiasIndex) =
        perturbed_in.segment<3>(accelerometerBiasIndex) -
        reference_in.segment<3>(accelerometerBiasIndex);
    error.segment<3>(errorGyroscopeBiasIndex) =
        perturbed_in.segment<3>(gyroscopeBiasIndex) -
        reference_in.segment<3>(gyroscopeBiasIndex);
    const Eigen::Quaterniond referenceQuaternion(
        reference_in(quaternionIndex + 3),
        reference_in(quaternionIndex),
        reference_in(quaternionIndex + 1),
        reference_in(quaternionIndex + 2));
    const Eigen::Quaterniond perturbedQuaternion(
        perturbed_in(quaternionIndex + 3),
        perturbed_in(quaternionIndex),
        perturbed_in(quaternionIndex + 1),
        perturbed_in(quaternionIndex + 2));
    error.segment<3>(errorAttitudeIndex) = quaternionToRotationVector(
        referenceQuaternion.conjugate() * perturbedQuaternion);
    return error;
}

TEST(KalmanMath, BiasAwareEskfStateSizesMatchContract)
{
    EXPECT_EQ(AlphaFilter::NOMINAL_STATE_SIZE, 16);
    EXPECT_EQ(AlphaFilter::ERROR_STATE_SIZE, 15);
    EXPECT_EQ(
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_Z),
        15);
    EXPECT_EQ(static_cast<Eigen::Index>(
                  ErrorStateIndex::ERROR_STATE_INDEX_GYROSCOPE_BIAS_Z),
              14);
}

TEST(KalmanMath, BiasCorrectedStationaryImuDoesNotMoveState)
{
    AlphaFilter::NominalStateVector state = makeEskfState();
    const Eigen::Index              accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    const Eigen::Vector3d accelerometerBias(0.01, -0.02, 0.03);
    const Eigen::Vector3d gyroscopeBias(0.001, -0.002, 0.003);
    const Eigen::Vector3d gravity_fixed_mPerS2(0.0, 0.0, -1.62);
    state.segment<3>(accelerometerBiasIndex) = accelerometerBias;
    state.segment<3>(gyroscopeBiasIndex)     = gyroscopeBias;

    AlphaFilter::NominalStateVector predictedState;
    AlphaFilter::ErrorStateMatrix   processJacobian;
    AlphaFilter::calculateProcessModel(state,
                                       Eigen::Vector3d(0.0, 0.0, 1.62) +
                                           accelerometerBias,
                                       gyroscopeBias,
                                       gravity_fixed_mPerS2,
                                       0.02,
                                       predictedState,
                                       processJacobian);

    EXPECT_TRUE(predictedState.isApprox(state, 1.0e-12));
    EXPECT_TRUE(processJacobian.allFinite());
}

TEST(KalmanMath, CorrectedAttitudeControlsGravityProjection)
{
    AlphaFilter::NominalStateVector correctedState = makeEskfState();
    AlphaFilter::NominalStateVector tiltedState    = correctedState;
    const Eigen::Index              quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Quaterniond wrongAttitude(
        Eigen::AngleAxisd(0.017453292519943295, Eigen::Vector3d::UnitY()));
    tiltedState(quaternionIndex)     = wrongAttitude.x();
    tiltedState(quaternionIndex + 1) = wrongAttitude.y();
    tiltedState(quaternionIndex + 2) = wrongAttitude.z();
    tiltedState(quaternionIndex + 3) = wrongAttitude.w();
    const Eigen::Vector3d gravity_fixed_mPerS2(0.0, 0.0, -1.62);
    const Eigen::Vector3d stationarySpecificForce_body_mPerS2(0.0, 0.0, 1.62);

    AlphaFilter::NominalStateVector wrongPrediction;
    AlphaFilter::NominalStateVector correctedPrediction;
    AlphaFilter::ErrorStateMatrix   processJacobian;
    AlphaFilter::calculateProcessModel(tiltedState,
                                       stationarySpecificForce_body_mPerS2,
                                       Eigen::Vector3d::Zero(),
                                       gravity_fixed_mPerS2,
                                       1.0,
                                       wrongPrediction,
                                       processJacobian);
    AlphaFilter::calculateProcessModel(correctedState,
                                       stationarySpecificForce_body_mPerS2,
                                       Eigen::Vector3d::Zero(),
                                       gravity_fixed_mPerS2,
                                       1.0,
                                       correctedPrediction,
                                       processJacobian);

    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    EXPECT_GT(std::abs(wrongPrediction(velocityIndex)), 0.02);
    EXPECT_TRUE(correctedPrediction.segment<3>(velocityIndex).isZero(1.0e-12));
}

TEST(KalmanMath, EskfProcessJacobianMatchesFiniteDifference)
{
    AlphaFilter::NominalStateVector state = makeEskfState();
    const Eigen::Index              velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Index accelerometerBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X);
    const Eigen::Index gyroscopeBiasIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X);
    state.segment<3>(velocityIndex) = Eigen::Vector3d(0.2, -0.1, 0.05);
    state.segment<3>(accelerometerBiasIndex) =
        Eigen::Vector3d(0.01, -0.02, 0.005);
    state.segment<3>(gyroscopeBiasIndex) =
        Eigen::Vector3d(0.001, -0.002, 0.003);
    const Eigen::Vector3d specificForce(0.3, -0.2, 1.7);
    const Eigen::Vector3d angularVelocity(0.04, -0.03, 0.02);
    const Eigen::Vector3d gravity(0.0, 0.0, -1.62);
    constexpr double      TIME_STEP_S  = 1.0e-5;
    constexpr double      PERTURBATION = 1.0e-7;

    AlphaFilter::NominalStateVector referencePrediction;
    AlphaFilter::ErrorStateMatrix   processJacobian;
    AlphaFilter::calculateProcessModel(state,
                                       specificForce,
                                       angularVelocity,
                                       gravity,
                                       TIME_STEP_S,
                                       referencePrediction,
                                       processJacobian);
    AlphaFilter::ErrorStateMatrix numericalTransition;
    for (Eigen::Index column = 0; column < AlphaFilter::ERROR_STATE_SIZE;
         ++column)
    {
        AlphaFilter::ErrorStateVector perturbation =
            AlphaFilter::ErrorStateVector::Zero();
        perturbation(column) = PERTURBATION;
        const AlphaFilter::NominalStateVector perturbedState =
            injectEskfError(state, perturbation);
        AlphaFilter::NominalStateVector perturbedPrediction;
        AlphaFilter::ErrorStateMatrix   unusedJacobian;
        AlphaFilter::calculateProcessModel(perturbedState,
                                           specificForce,
                                           angularVelocity,
                                           gravity,
                                           TIME_STEP_S,
                                           perturbedPrediction,
                                           unusedJacobian);
        numericalTransition.col(column) =
            calculateEskfError(referencePrediction, perturbedPrediction) /
            PERTURBATION;
    }
    const AlphaFilter::ErrorStateMatrix analyticalTransition =
        AlphaFilter::ErrorStateMatrix::Identity() +
        processJacobian * TIME_STEP_S;
    EXPECT_TRUE(analyticalTransition.isApprox(numericalTransition, 2.0e-5));
}

TEST(KalmanMath, WheelVelocityJacobianMatchesFiniteDifference)
{
    AlphaFilter::NominalStateVector state = makeEskfState();
    const Eigen::Index              quaternionIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
    const Eigen::Index velocityIndex =
        static_cast<Eigen::Index>(StateIndex::STATE_INDEX_LINEAR_VELOCITY_X);
    const Eigen::Quaterniond attitude(
        Eigen::AngleAxisd(0.2, Eigen::Vector3d(0.3, -0.4, 0.5).normalized()));
    state(quaternionIndex)          = attitude.x();
    state(quaternionIndex + 1)      = attitude.y();
    state(quaternionIndex + 2)      = attitude.z();
    state(quaternionIndex + 3)      = attitude.w();
    state.segment<3>(velocityIndex) = Eigen::Vector3d(0.4, -0.2, 0.1);

    Eigen::Vector2d                             predictedVelocity;
    AlphaFilter::WheelVelocityObservationMatrix analyticalObservation;
    AlphaFilter::calculateWheelVelocityObservation(state,
                                                   predictedVelocity,
                                                   analyticalObservation);
    AlphaFilter::WheelVelocityObservationMatrix numericalObservation;
    constexpr double                            PERTURBATION = 1.0e-7;
    for (Eigen::Index column = 0; column < AlphaFilter::ERROR_STATE_SIZE;
         ++column)
    {
        AlphaFilter::ErrorStateVector perturbation =
            AlphaFilter::ErrorStateVector::Zero();
        perturbation(column) = PERTURBATION;
        Eigen::Vector2d                             perturbedVelocity;
        AlphaFilter::WheelVelocityObservationMatrix unusedObservation;
        AlphaFilter::calculateWheelVelocityObservation(
            injectEskfError(state, perturbation),
            perturbedVelocity,
            unusedObservation);
        numericalObservation.col(column) =
            (perturbedVelocity - predictedVelocity) / PERTURBATION;
    }
    EXPECT_TRUE(analyticalObservation.isApprox(numericalObservation, 1.0e-6));
}

TEST(KalmanMath, PredictAndUpdateRequireInitializeFirst)
{
    Filter                filter;
    const Eigen::VectorXd derivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd jacobian   = Eigen::MatrixXd::Zero(2, 2);
    const Eigen::MatrixXd noise      = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_EQ(filter.predict(0.1, derivative, jacobian, noise),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);

    const Eigen::VectorXd innovation  = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(2, 2);

    EXPECT_EQ(filter.update(innovation, observation, noise),
              FilterStatus::FILTER_STATUS_NOT_INITIALIZED);
}

TEST(KalmanMath, TerminateReturnsToUninitializedState)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(2,
                                Eigen::VectorXd::Zero(2),
                                Eigen::MatrixXd::Identity(2, 2)),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(filter.terminate(), FilterStatus::FILTER_STATUS_SUCCESS);

    const Eigen::VectorXd derivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd jacobian   = Eigen::MatrixXd::Zero(2, 2);
    const Eigen::MatrixXd noise      = Eigen::MatrixXd::Identity(2, 2);
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
    ASSERT_EQ(
        filter.initialize(2, initialState, Eigen::MatrixXd::Identity(2, 2)),
        FilterStatus::FILTER_STATUS_SUCCESS);

    Eigen::VectorXd stateDerivative(2);
    stateDerivative << 2.0, 0.0;
    Eigen::MatrixXd processJacobian    = Eigen::MatrixXd::Zero(2, 2);
    processJacobian(0, 1)              = 1.0;
    const Eigen::MatrixXd processNoise = Eigen::MatrixXd::Zero(2, 2);

    ASSERT_EQ(
        filter.predict(0.5, stateDerivative, processJacobian, processNoise),
        FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_NEAR(filter.getState()(0), 2.0, 1.0e-10);
    EXPECT_NEAR(filter.getState()(1), 2.0, 1.0e-10);
    EXPECT_TRUE(
        filter.getCovariance().isApprox(filter.getCovariance().transpose(),
                                        1.0e-12));
}

TEST(KalmanMath, PredictionPreservesCoupledCovariancePositivity)
{
    Filter          filter;
    Eigen::Matrix2d initialCovariance = Eigen::Matrix2d::Zero();
    initialCovariance(0, 0)           = 1.0e-9;
    initialCovariance(1, 1)           = 1.0;
    ASSERT_EQ(filter.initialize(2, Eigen::Vector2d::Zero(), initialCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Eigen::Matrix2d processJacobian = Eigen::Matrix2d::Zero();
    processJacobian(0, 1)           = 1.0;
    Eigen::Matrix2d processNoise    = Eigen::Matrix2d::Zero();
    processNoise(0, 0)              = 0.002;
    processNoise(1, 1)              = 0.02;

    ASSERT_EQ(filter.predict(0.02,
                             Eigen::Vector2d::Zero(),
                             processJacobian,
                             processNoise),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_TRUE(
        filter.getCovariance().isApprox(filter.getCovariance().transpose(),
                                        1.0e-12));
    EXPECT_GT(filter.getCovariance().determinant(), 0.0);
}

TEST(KalmanMath, CorrectsOnlySelectedMeasurementStates)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(3,
                                Eigen::VectorXd::Zero(3),
                                Eigen::MatrixXd::Identity(3, 3)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    /* Observation matrix selecting only states 0 and 1, leaving state 2
     * entirely unobserved -- matching the old MeasurementMask{0,1}
     * behaviour without needing a dummy zero-effect measurement channel
     * for the unobserved state. */
    Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(2, 3);
    observation(0, 0)           = 1.0;
    observation(1, 1)           = 1.0;

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
    ASSERT_EQ(filter.initialize(2,
                                Eigen::VectorXd::Zero(2),
                                Eigen::MatrixXd::Identity(2, 2)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd       innovation(2);
    innovation << 10.0, 10.0;
    Eigen::MatrixXd measurementNoise = Eigen::MatrixXd::Zero(2, 2);
    measurementNoise(0, 0)           = 0.01;
    measurementNoise(1, 1)           = 100.0;

    ASSERT_EQ(filter.update(innovation, observation, measurementNoise),
              FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_GT(filter.getState()(0), 9.0);
    EXPECT_LT(filter.getState()(1), 0.2);
}

TEST(KalmanMath, AppliesCovarianceCoordinateTransform)
{
    Filter          filter;
    Eigen::Matrix2d initialCovariance;
    initialCovariance << 2.0, 0.5, 0.5, 1.0;
    ASSERT_EQ(filter.initialize(2, Eigen::Vector2d::Zero(), initialCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);

    Eigen::Matrix2d transform;
    transform << 1.0, 0.2, -0.1, 0.9;
    ASSERT_EQ(filter.applyCovarianceTransform(transform),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_TRUE(filter.getCovariance().isApprox(transform * initialCovariance *
                                                    transform.transpose(),
                                                1.0e-12));
}

TEST(KalmanMath, RestoresValidatedCheckpoint)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(2,
                                Eigen::Vector2d::Zero(),
                                Eigen::Matrix2d::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);
    Eigen::Vector2d checkpointState(3.0, -2.0);
    Eigen::Matrix2d checkpointCovariance;
    checkpointCovariance << 2.0, 0.25, 0.25, 1.0;

    ASSERT_EQ(filter.restore(checkpointState, checkpointCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    EXPECT_TRUE(filter.getState().isApprox(checkpointState));
    EXPECT_TRUE(filter.getCovariance().isApprox(checkpointCovariance));
}

TEST(KalmanMath, RejectsIndefiniteCheckpointWithoutMutation)
{
    Filter                filter;
    const Eigen::Vector2d initialState(1.0, 2.0);
    ASSERT_EQ(filter.initialize(2, initialState, Eigen::Matrix2d::Identity()),
              FilterStatus::FILTER_STATUS_SUCCESS);
    Eigen::Matrix2d indefiniteCovariance;
    indefiniteCovariance << 1.0, 2.0, 2.0, 1.0;

    EXPECT_EQ(filter.restore(Eigen::Vector2d::Zero(), indefiniteCovariance),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);
    EXPECT_TRUE(filter.getState().isApprox(initialState));
    EXPECT_TRUE(filter.getCovariance().isApprox(Eigen::Matrix2d::Identity()));
}

TEST(KalmanMath, RollbackUpdateReplayMatchesInOrderArrival)
{
    const Eigen::Vector2d initialState(0.0, 1.0);
    const Eigen::Matrix2d initialCovariance = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d       processJacobian   = Eigen::Matrix2d::Zero();
    processJacobian(0, 1)                   = 1.0;
    const Eigen::Matrix2d processNoise = Eigen::Matrix2d::Identity() * 0.01;
    Eigen::Matrix<double, 1, 2> observation;
    observation << 1.0, 0.0;
    const Eigen::Matrix<double, 1, 1> measurementNoise =
        Eigen::Matrix<double, 1, 1>::Constant(0.1);

    const auto predictOneSecond =
        [&processJacobian, &processNoise](Filter &filter_inout)
    {
        constexpr std::size_t SUBSTEP_COUNT = 50U;
        constexpr double      SUBSTEP_S     = 0.02;
        for (std::size_t substep = 0U; substep < SUBSTEP_COUNT; ++substep)
        {
            Eigen::Vector2d derivative;
            derivative << filter_inout.getState()(1), 0.0;
            const FilterStatus status = filter_inout.predict(SUBSTEP_S,
                                                             derivative,
                                                             processJacobian,
                                                             processNoise);
            if (status != FilterStatus::FILTER_STATUS_SUCCESS)
            {
                return status;
            }
        }
        return FilterStatus::FILTER_STATUS_SUCCESS;
    };
    const auto updateAtOneSecond =
        [&observation, &measurementNoise](Filter &filter_inout)
    {
        Eigen::VectorXd innovation(1);
        innovation(0) = 1.2 - filter_inout.getState()(0);
        return filter_inout.update(innovation, observation, measurementNoise);
    };

    Filter inOrder;
    ASSERT_EQ(inOrder.initialize(2, initialState, initialCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(inOrder), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(updateAtOneSecond(inOrder), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(inOrder), FilterStatus::FILTER_STATUS_SUCCESS);

    Filter delayed;
    ASSERT_EQ(delayed.initialize(2, initialState, initialCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(delayed), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(delayed), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(delayed.restore(initialState, initialCovariance),
              FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(delayed), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(updateAtOneSecond(delayed), FilterStatus::FILTER_STATUS_SUCCESS);
    ASSERT_EQ(predictOneSecond(delayed), FilterStatus::FILTER_STATUS_SUCCESS);

    EXPECT_TRUE(delayed.getState().isApprox(inOrder.getState(), 1.0e-12));
    EXPECT_TRUE(
        delayed.getCovariance().isApprox(inOrder.getCovariance(), 1.0e-12));
}

TEST(KalmanMath, RightErrorResetJacobianMatchesQuaternionFiniteDifference)
{
    const Eigen::Vector3d correction(0.20, -0.10, 0.05);
    const Eigen::Matrix3d analytical =
        AlphaFilter::calculateAttitudeResetJacobian(correction);
    Eigen::Matrix3d          numerical;
    constexpr double         PERTURBATION_RAD = 1.0e-7;
    const Eigen::Quaterniond correctedInverse =
        rotationVectorToQuaternion(correction).conjugate();

    for (Eigen::Index column = 0; column < 3; ++column)
    {
        Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
        perturbation(column)         = PERTURBATION_RAD;
        const Eigen::Quaterniond resetError =
            correctedInverse *
            rotationVectorToQuaternion(correction + perturbation);
        numerical.col(column) =
            quaternionToRotationVector(resetError) / PERTURBATION_RAD;
    }

    EXPECT_TRUE(analytical.isApprox(numerical, 1.0e-8));
}

TEST(KalmanMath, NormalizedInnovationUsesActiveMeasurementDimension)
{
    const Eigen::Vector2d             innovation(2.0, -1.0);
    const Eigen::Matrix<double, 2, 3> observation =
        (Eigen::Matrix<double, 2, 3>() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
            .finished();
    const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    const Eigen::Matrix2d noise      = Eigen::Matrix2d::Identity();
    double                nis        = 0.0;

    ASSERT_TRUE(AlphaFilter::calculateNormalizedInnovationSquared(innovation,
                                                                  observation,
                                                                  covariance,
                                                                  noise,
                                                                  nis));
    EXPECT_NEAR(nis, 2.5, 1.0e-12);
    EXPECT_DOUBLE_EQ(AlphaFilter::selectNisThreshold(0.0, 2), 9.210);
    EXPECT_DOUBLE_EQ(AlphaFilter::selectNisThreshold(4.5, 2), 4.5);
}

TEST(KalmanMath, NormalizedInnovationRejectsNonPositiveCovariance)
{
    const Eigen::VectorXd innovation  = Eigen::VectorXd::Ones(1);
    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(1, 1);
    const Eigen::MatrixXd covariance  = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd       noise(1, 1);
    noise(0, 0) = -1.0;
    double nis  = 0.0;

    EXPECT_FALSE(AlphaFilter::calculateNormalizedInnovationSquared(innovation,
                                                                   observation,
                                                                   covariance,
                                                                   noise,
                                                                   nis));
}

TEST(KalmanMath, OutlierNisExceedsDimensionSpecificGate)
{
    const Eigen::VectorXd innovation  = Eigen::VectorXd::Constant(3, 10.0);
    const Eigen::MatrixXd observation = Eigen::MatrixXd::Identity(3, 3);
    const Eigen::MatrixXd covariance  = Eigen::MatrixXd::Identity(3, 3);
    const Eigen::MatrixXd noise       = Eigen::MatrixXd::Identity(3, 3);
    double                nis         = 0.0;

    ASSERT_TRUE(AlphaFilter::calculateNormalizedInnovationSquared(innovation,
                                                                  observation,
                                                                  covariance,
                                                                  noise,
                                                                  nis));
    EXPECT_GT(nis, AlphaFilter::selectNisThreshold(0.0, 3));
}

TEST(KalmanMath, RejectsMismatchedMatrixDimensions)
{
    Filter filter;
    ASSERT_EQ(filter.initialize(3,
                                Eigen::VectorXd::Zero(3),
                                Eigen::MatrixXd::Identity(3, 3)),
              FilterStatus::FILTER_STATUS_SUCCESS);

    /*!
     * With fixed-size Eigen types this class of error used to be caught
     * at compile time; dynamic sizing makes it a real runtime failure
     * mode the engine must now detect itself.
     */
    const Eigen::VectorXd wrongSizeDerivative = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd rightSizeJacobian   = Eigen::MatrixXd::Zero(3, 3);
    const Eigen::MatrixXd rightSizeNoise      = Eigen::MatrixXd::Identity(3, 3);
    EXPECT_EQ(filter.predict(0.1,
                             wrongSizeDerivative,
                             rightSizeJacobian,
                             rightSizeNoise),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);

    const Eigen::VectorXd innovation              = Eigen::VectorXd::Zero(2);
    const Eigen::MatrixXd wrongColumnsObservation = Eigen::MatrixXd::Zero(2, 4);
    const Eigen::MatrixXd rightSizeMeasurementNoise =
        Eigen::MatrixXd::Identity(2, 2);
    EXPECT_EQ(filter.update(innovation,
                            wrongColumnsObservation,
                            rightSizeMeasurementNoise),
              FilterStatus::FILTER_STATUS_INVALID_INPUT);
}

TEST(KalmanMath, SixWheelRollingSystemRecoversPlanarTwist)
{
    const std::array<double, 6> wheelXM{0.64, 0.64, 0.0, 0.0, -0.72, -0.72};
    const std::array<double, 6> wheelYM{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};
    const Eigen::Vector3d       expectedTwist(0.4, 0.05, 0.2);
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
    const std::array<double, 6> wheelXM{0.64, 0.64, 0.0, 0.0, -0.72, -0.72};
    const std::array<double, 6> wheelYM{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};
    const std::array<double, 6> steeringNoiseRad{0.024,
                                                 -0.018,
                                                 -0.004,
                                                 -0.003,
                                                 -0.007,
                                                 -0.011};
    Eigen::Matrix<double, 6, 3> rollingMatrix;
    Eigen::Matrix<double, 6, 1> wheelSpeeds;
    for (std::size_t index = 0U; index < wheelYM.size(); ++index)
    {
        rollingMatrix(static_cast<Eigen::Index>(index), 0) =
            std::cos(steeringNoiseRad[index]);
        rollingMatrix(static_cast<Eigen::Index>(index), 1) =
            std::sin(steeringNoiseRad[index]);
        rollingMatrix(static_cast<Eigen::Index>(index), 2) =
            -wheelYM[index] * std::cos(steeringNoiseRad[index]) +
            wheelXM[index] * std::sin(steeringNoiseRad[index]);
        wheelSpeeds(static_cast<Eigen::Index>(index)) =
            0.015 + (static_cast<double>(index) - 2.5) * 2.0e-4;
    }
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 3>>
        rollingDecomposition(rollingMatrix);
    rollingDecomposition.setThreshold(0.1);
    const Eigen::Vector3d recoveredTwist =
        rollingDecomposition.solve(wheelSpeeds);

    EXPECT_NEAR(recoveredTwist.x(), 0.015, 5.0e-4);
    EXPECT_NEAR(recoveredTwist.y(), 0.0, 1.0e-3);
}

TEST(KalmanMath, VisualRollingSpeedRecoversWheelSlipRatio)
{
    constexpr double      expectedSlipRatio = 0.20;
    constexpr double      wheelXM           = 0.64;
    constexpr double      wheelYM           = 0.60;
    constexpr double      steeringRad       = 0.31;
    const Eigen::Vector3d visualTwistBody(0.18, 0.03, 0.07);
    const double          expectedRollingSpeedMps =
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
                expectedRollingSpeedMps,
                1.0e-12);
}

TEST(KalmanMath, VisualRollingSpeedRecoversForwardSkidRatio)
{
    constexpr double expectedSlipRatio       = -0.20;
    constexpr double expectedRollingSpeedMps = 0.18;
    const double     rawWheelSpeedMps =
        expectedRollingSpeedMps / (1.0 - expectedSlipRatio);
    const double observedSlipRatio =
        1.0 - expectedRollingSpeedMps / rawWheelSpeedMps;

    EXPECT_NEAR(observedSlipRatio, expectedSlipRatio, 1.0e-12);
    EXPECT_NEAR(rawWheelSpeedMps * (1.0 - observedSlipRatio),
                expectedRollingSpeedMps,
                1.0e-12);
}

} /* namespace */
