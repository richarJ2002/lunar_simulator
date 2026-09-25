/*!
 * @file            test_wheel_solve.cpp
 *
 * @brief           Tests the twelve-row weighted least-squares wheel
 *                  kinematic solve against exact rigid-body kinematics.
 *
 *                  Each case builds steering angles and rolling speeds from
 *                  a known body twist (vx, vy, wz): wheel i at (x_i, y_i)
 *                  moves at (vx - wz y_i, vy + wz x_i), its steering angle
 *                  is that velocity's direction and its rolling speed its
 *                  magnitude. The solve must recover the twist.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <random>

#include "public_functions.h"

namespace
{

namespace wheel = localisation::wheel_odometry;

/* Alpha's wheel layout, front-left..rear-right (wheel_odometry.yaml). */
const std::array<double, 6> WHEEL_X_M{0.64, 0.64, 0.0, 0.0, -0.72, -0.72};
const std::array<double, 6> WHEEL_Y_M{0.60, -0.60, 0.60, -0.60, 0.60, -0.60};

/* Encoder noise 0.015 rad/s at radius 0.1425 m, steering 0.002 rad. */
constexpr double ROLLING_STDDEV_MPS  = 0.1425 * 0.015;
constexpr double STEERING_STDDEV_RAD = 0.002;
constexpr double LATERAL_STDDEV_MPS  = 0.003;

/*!
 * @brief           Wheel angles and speeds produced by an exact body twist.
 */
struct WheelMeasurements
{
    std::array<double, 6> steeringAngle_rad{};
    std::array<double, 6> rollingSpeed_mps{};
};

WheelMeasurements makeExactMeasurements(double vx_mps_in,
                                        double vy_mps_in,
                                        double wz_radps_in)
{
    WheelMeasurements measurements;
    for (std::size_t index = 0U; index < 6U; ++index)
    {
        const double wheelVx = vx_mps_in - wz_radps_in * WHEEL_Y_M[index];
        const double wheelVy = vy_mps_in + wz_radps_in * WHEEL_X_M[index];
        measurements.rollingSpeed_mps[index] = std::hypot(wheelVx, wheelVy);
        /* A stationary wheel keeps its steering at zero. */
        measurements.steeringAngle_rad[index] =
            measurements.rollingSpeed_mps[index] > 0.0
                ? std::atan2(wheelVy, wheelVx)
                : 0.0;
    }
    return measurements;
}

wheel::BodyTwistSolution solve(const WheelMeasurements &measurements_in)
{
    return wheel::solveBodyTwist(measurements_in.steeringAngle_rad,
                                 measurements_in.rollingSpeed_mps,
                                 WHEEL_X_M,
                                 WHEEL_Y_M,
                                 ROLLING_STDDEV_MPS,
                                 STEERING_STDDEV_RAD,
                                 LATERAL_STDDEV_MPS);
}

void expectTwist(const wheel::BodyTwistSolution &solution_in,
                 double                          vx_mps_in,
                 double                          vy_mps_in,
                 double                          wz_radps_in)
{
    ASSERT_TRUE(solution_in.isValid);
    EXPECT_NEAR(solution_in.twist_body.x(), vx_mps_in, 1.0e-12);
    EXPECT_NEAR(solution_in.twist_body.y(), vy_mps_in, 1.0e-12);
    EXPECT_NEAR(solution_in.twist_body.z(), wz_radps_in, 1.0e-12);
}

TEST(WheelSolve, ParallelStraightDriving)
{
    const wheel::BodyTwistSolution solution =
        solve(makeExactMeasurements(0.018, 0.0, 0.0));
    expectTwist(solution, 0.018, 0.0, 0.0);
    /* vy is observed with a real, small covariance, not a 1e3 placeholder:
     * six lateral rows at 3 mm/s give roughly 3/sqrt(6) mm/s. */
    EXPECT_LT(std::sqrt(solution.covariance_body(1, 1)), 0.002);
    EXPECT_GT(std::sqrt(solution.covariance_body(1, 1)), 0.0005);
}

TEST(WheelSolve, NoisyParallelWheelsDoNotAmplifyLateralVelocity)
{
    /* The retired rolling-only solve turned a few milliradians of steering
     * noise into more than 0.1 m/s of lateral velocity on a parked rover.
     * Random steering noise on stationary and slowly driving wheels must
     * leave |vy| at noise level. */
    std::mt19937                     generator(7302028);
    std::normal_distribution<double> steeringNoise(0.0, 0.004);
    std::normal_distribution<double> speedNoise(0.0, ROLLING_STDDEV_MPS);
    for (const double speed_mps : {0.0, 0.018})
    {
        double maximumLateralMps = 0.0;
        for (int trial = 0; trial < 200; ++trial)
        {
            WheelMeasurements measurements =
                makeExactMeasurements(speed_mps, 0.0, 0.0);
            for (std::size_t index = 0U; index < 6U; ++index)
            {
                measurements.steeringAngle_rad[index] +=
                    steeringNoise(generator);
                measurements.rollingSpeed_mps[index] += speedNoise(generator);
            }
            const wheel::BodyTwistSolution solution = solve(measurements);
            ASSERT_TRUE(solution.isValid);
            maximumLateralMps =
                std::max(maximumLateralMps, std::abs(solution.twist_body.y()));
        }
        EXPECT_LT(maximumLateralMps, 0.001) << "speed " << speed_mps;
    }
}

TEST(WheelSolve, AckermannTurnAboutACentreOfRotation)
{
    expectTwist(solve(makeExactMeasurements(0.015, 0.0, 0.01)),
                0.015,
                0.0,
                0.01);
}

TEST(WheelSolve, CrabRecoversLateralVelocity)
{
    expectTwist(solve(makeExactMeasurements(0.012, 0.009, 0.0)),
                0.012,
                0.009,
                0.0);
}

TEST(WheelSolve, PointTurnRecoversYawRate)
{
    expectTwist(solve(makeExactMeasurements(0.0, 0.0, 0.02)), 0.0, 0.0, 0.02);
}

TEST(WheelSolve, ParkedRoverIsExactlyStationary)
{
    WheelMeasurements measurements = makeExactMeasurements(0.0, 0.0, 0.0);
    measurements.steeringAngle_rad = {0.003, -0.002, 0.02, 0.0, -0.004, 0.001};
    const wheel::BodyTwistSolution solution = solve(measurements);
    ASSERT_TRUE(solution.isValid);
    EXPECT_EQ(solution.twist_body.norm(), 0.0);
}

TEST(WheelSolve, CovarianceIsSymmetricPositiveDefinite)
{
    const wheel::BodyTwistSolution solution =
        solve(makeExactMeasurements(0.01, 0.004, 0.015));
    ASSERT_TRUE(solution.isValid);
    EXPECT_TRUE(solution.covariance_body.isApprox(
        solution.covariance_body.transpose()));
    EXPECT_EQ(solution.covariance_body.llt().info(), Eigen::Success);
    EXPECT_GT(solution.normalCondition, 1.0);
}

TEST(WheelSolve, InvalidInputsAreRejected)
{
    const WheelMeasurements measurements =
        makeExactMeasurements(0.018, 0.0, 0.0);
    EXPECT_FALSE(wheel::solveBodyTwist(measurements.steeringAngle_rad,
                                       measurements.rollingSpeed_mps,
                                       WHEEL_X_M,
                                       WHEEL_Y_M,
                                       0.0,
                                       STEERING_STDDEV_RAD,
                                       LATERAL_STDDEV_MPS)
                     .isValid);
    EXPECT_FALSE(wheel::solveBodyTwist(measurements.steeringAngle_rad,
                                       measurements.rollingSpeed_mps,
                                       WHEEL_X_M,
                                       WHEEL_Y_M,
                                       ROLLING_STDDEV_MPS,
                                       STEERING_STDDEV_RAD,
                                       -1.0)
                     .isValid);
    WheelMeasurements corrupt   = measurements;
    corrupt.rollingSpeed_mps[2] = std::nan("");
    EXPECT_FALSE(solve(corrupt).isValid);
}

} /* namespace */
