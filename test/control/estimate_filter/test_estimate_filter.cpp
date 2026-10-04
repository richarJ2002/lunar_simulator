/*!
 * @file            test_estimate_filter.cpp
 *
 * @brief           Tests the ROS-free estimate low-pass filter: step
 *                  response without overshoot, ramp tracking without lag,
 *                  skipped duplicate stamps, yaw across +/-pi, and resets.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "estimate_filter/objects/EstimateLowPassFilterClass.h"
#include "estimate_filter/public_functions/public_functions.h"

namespace
{

namespace lpf = control::estimate_filter;
using Result  = lpf::FilterUpdateResult;

constexpr double SAMPLE_PERIOD_S = 0.02;
constexpr double PI              = 3.141592653589793;

lpf::EstimateSample makeSample(double stamp_s_in)
{
    lpf::EstimateSample sample;
    sample.stamp_s = stamp_s_in;
    return sample;
}

double yawOf(const Eigen::Quaterniond &orientation_in)
{
    const Eigen::Vector3d heading = orientation_in * Eigen::Vector3d::UnitX();
    return std::atan2(heading.y(), heading.x());
}

TEST(EstimateFilter, VelocityStepSettlesWithoutOvershoot)
{
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    ASSERT_EQ(filter.update(makeSample(0.0), false),
              Result::FILTER_UPDATE_RESULT_RESET);
    double previousVelocity = 0.0;
    for (int step = 1; step <= 100; ++step)
    {
        lpf::EstimateSample sample = makeSample(step * SAMPLE_PERIOD_S);
        sample.linearVelocity_body_mPerS.x() = 1.0;
        ASSERT_EQ(filter.update(sample, false),
                  Result::FILTER_UPDATE_RESULT_FILTERED);
        const double velocity =
            filter.getOutput().linearVelocity_body_mPerS.x();
        /* Critically damped: monotone rise, never above the step. */
        EXPECT_GE(velocity, previousVelocity);
        EXPECT_LE(velocity, 1.0 + 1.0e-12);
        previousVelocity = velocity;
    }
    /* Two 1 Hz stages: 1 - exp(-t/tau)(1 + t/tau) > 0.999 at t = 2 s. */
    EXPECT_GT(previousVelocity, 0.999);
}

TEST(EstimateFilter, ConstantVelocityRampHasNoSteadyStateLag)
{
    /* The input position ramps at 0.02 m/s but starts 0.1 m away from the
     * filter's first sample; the error must decay to zero, not to the
     * v * tau lag a first-order filter without feed-forward would keep. */
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    constexpr double           SPEED_MPS = 0.02;
    lpf::EstimateSample        first     = makeSample(0.0);
    first.position_fixed_m.x()           = 0.1;
    first.linearVelocity_body_mPerS.x()  = SPEED_MPS;
    filter.update(first, false);
    double error_m = 0.0;
    for (int step = 1; step <= 1000; ++step)
    {
        const double        time_s           = step * SAMPLE_PERIOD_S;
        lpf::EstimateSample sample           = makeSample(time_s);
        sample.position_fixed_m.x()          = SPEED_MPS * time_s;
        sample.linearVelocity_body_mPerS.x() = SPEED_MPS;
        filter.update(sample, false);
        error_m = filter.getOutput().position_fixed_m.x() - SPEED_MPS * time_s;
    }
    const double firstOrderLag_m = SPEED_MPS / (2.0 * PI * 0.5);
    EXPECT_LT(std::abs(error_m), 1.0e-9);
    EXPECT_GT(firstOrderLag_m, 1.0e-3);
}

TEST(EstimateFilter, DuplicateStampIsSkippedWithoutChangingOutput)
{
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    filter.update(makeSample(1.0), false);
    lpf::EstimateSample next  = makeSample(1.02);
    next.position_fixed_m.x() = 0.5;
    filter.update(next, false);
    const Eigen::Vector3d before    = filter.getOutput().position_fixed_m;
    lpf::EstimateSample   duplicate = next;
    duplicate.position_fixed_m.x()  = 5.0;
    EXPECT_EQ(filter.update(duplicate, false),
              Result::FILTER_UPDATE_RESULT_SKIPPED);
    EXPECT_TRUE(filter.getOutput().position_fixed_m.isApprox(before));
    EXPECT_EQ(filter.update(makeSample(1.01), false),
              Result::FILTER_UPDATE_RESULT_SKIPPED);
}

TEST(EstimateFilter, YawAcrossPiStaysNearPi)
{
    /* Heading swings between +179 and -179 degrees; the filtered heading
     * must stay near +/-180, never swing through 0 as an Euler average
     * would. */
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    for (int step = 0; step <= 200; ++step)
    {
        const double yaw_rad = (step % 2 == 0 ? 179.0 : -179.0) * PI / 180.0;
        lpf::EstimateSample sample     = makeSample(step * SAMPLE_PERIOD_S);
        sample.orientation_bodyToFixed = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
        filter.update(sample, false);
        EXPECT_GT(std::abs(yawOf(filter.getOutput().orientation_bodyToFixed)),
                  178.0 * PI / 180.0);
        EXPECT_NEAR(filter.getOutput().orientation_bodyToFixed.norm(),
                    1.0,
                    1.0e-12);
    }
}

TEST(EstimateFilter, GapOrFrameChangeResetsToTheSample)
{
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    filter.update(makeSample(0.0), false);
    lpf::EstimateSample afterGap = makeSample(0.51);
    afterGap.position_fixed_m    = Eigen::Vector3d(3.0, 2.0, 1.0);
    EXPECT_EQ(filter.update(afterGap, false),
              Result::FILTER_UPDATE_RESULT_RESET);
    EXPECT_TRUE(filter.getOutput().position_fixed_m.isApprox(
        afterGap.position_fixed_m));

    lpf::EstimateSample reframed = makeSample(0.53);
    reframed.position_fixed_m    = Eigen::Vector3d(-1.0, 0.0, 0.0);
    EXPECT_EQ(filter.update(reframed, true),
              Result::FILTER_UPDATE_RESULT_RESET);
    EXPECT_TRUE(filter.getOutput().position_fixed_m.isApprox(
        reframed.position_fixed_m));
}

TEST(EstimateFilter, NonFiniteSampleIsSkipped)
{
    lpf::EstimateLowPassFilter filter(lpf::LowPassConfiguration{});
    filter.update(makeSample(0.0), false);
    lpf::EstimateSample corrupt = makeSample(0.02);
    corrupt.linearVelocity_body_mPerS.y() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(filter.update(corrupt, false),
              Result::FILTER_UPDATE_RESULT_SKIPPED);
    EXPECT_TRUE(filter.getOutput().linearVelocity_body_mPerS.allFinite());
}

TEST(EstimateFilter, ConfigurationMustBePositiveAndFinite)
{
    EXPECT_TRUE(lpf::isConfigurationValid(lpf::LowPassConfiguration{}));
    lpf::LowPassConfiguration zeroCutoff;
    zeroCutoff.positionCutoffHz = 0.0;
    EXPECT_FALSE(lpf::isConfigurationValid(zeroCutoff));
    lpf::LowPassConfiguration infiniteGap;
    infiniteGap.maximumGap_s = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(lpf::isConfigurationValid(infiniteGap));
}

} /* namespace */
