/**
 * @file            test_imu_ring_buffer.cpp
 *
 * @brief           Tests Alpha's fixed-capacity filtered-IMU ring buffer.
 *
 * @date            20/09/2026
 */

/* C++ Standard Library Includes */
#include <cstddef>
#include <limits>

/* C Standard Library Includes */
/* None */

/* External Library Includes */
#include <gtest/gtest.h>

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/ImuRingBuffer.h"

namespace
{

using systems::alpha::alpha_localisation::alpha_kalman_filter::ImuRingBuffer;
using systems::alpha::alpha_localisation::alpha_kalman_filter::ImuSample;

ImuSample createSample(const double timestamp_s_in)
{
    ImuSample sample;
    sample.timestamp_s = timestamp_s_in;
    sample.linearAcceleration_body_mPerS2 =
        Eigen::Vector3d(timestamp_s_in, 2.0, 3.0);
    sample.angularVelocity_body_radPerS =
        Eigen::Vector3d(4.0, 5.0, timestamp_s_in);
    return sample;
}

TEST(ImuRingBuffer, StoresSamplesInChronologicalOrder)
{
    ImuRingBuffer buffer;

    ASSERT_TRUE(buffer.push(createSample(1.0)));
    ASSERT_TRUE(buffer.push(createSample(2.0)));
    ASSERT_TRUE(buffer.push(createSample(3.0)));

    ASSERT_EQ(buffer.getSampleCount(), 3U);
    ASSERT_TRUE(buffer.getSample(0U).has_value());
    ASSERT_TRUE(buffer.getSample(1U).has_value());
    ASSERT_TRUE(buffer.getLatestSample().has_value());
    EXPECT_DOUBLE_EQ(buffer.getSample(0U)->timestamp_s, 1.0);
    EXPECT_DOUBLE_EQ(buffer.getSample(1U)->timestamp_s, 2.0);
    EXPECT_DOUBLE_EQ(buffer.getLatestSample()->timestamp_s, 3.0);
    EXPECT_FALSE(buffer.getSample(3U).has_value());
}

TEST(ImuRingBuffer, RejectsDuplicateReversedAndNonFiniteSamples)
{
    ImuRingBuffer buffer;
    ASSERT_TRUE(buffer.push(createSample(2.0)));

    EXPECT_FALSE(buffer.push(createSample(2.0)));
    EXPECT_FALSE(buffer.push(createSample(1.0)));

    ImuSample invalidTimestampSample = createSample(3.0);
    invalidTimestampSample.timestamp_s =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(buffer.push(invalidTimestampSample));

    ImuSample invalidAccelerationSample = createSample(3.0);
    invalidAccelerationSample.linearAcceleration_body_mPerS2.x() =
        std::numeric_limits<double>::infinity();
    EXPECT_FALSE(buffer.push(invalidAccelerationSample));

    ImuSample invalidAngularVelocitySample = createSample(3.0);
    invalidAngularVelocitySample.angularVelocity_body_radPerS.z() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(buffer.push(invalidAngularVelocitySample));

    EXPECT_EQ(buffer.getSampleCount(), 1U);
}

TEST(ImuRingBuffer, OverwritesOldestSampleAtCapacity)
{
    ImuRingBuffer         buffer;
    constexpr std::size_t BUFFER_CAPACITY = 512U;

    for (std::size_t sampleIndex = 0U; sampleIndex < BUFFER_CAPACITY + 1U;
         ++sampleIndex)
    {
        ASSERT_TRUE(
            buffer.push(createSample(static_cast<double>(sampleIndex + 1U))));
    }

    ASSERT_EQ(buffer.getSampleCount(), BUFFER_CAPACITY);
    ASSERT_TRUE(buffer.getSample(0U).has_value());
    ASSERT_TRUE(buffer.getLatestSample().has_value());
    EXPECT_DOUBLE_EQ(buffer.getSample(0U)->timestamp_s, 2.0);
    EXPECT_DOUBLE_EQ(buffer.getLatestSample()->timestamp_s, 513.0);
}

TEST(ImuRingBuffer, ClearRestoresEmptyState)
{
    ImuRingBuffer buffer;
    ASSERT_TRUE(buffer.push(createSample(1.0)));

    buffer.clear();

    EXPECT_EQ(buffer.getSampleCount(), 0U);
    EXPECT_FALSE(buffer.getSample(0U).has_value());
    EXPECT_FALSE(buffer.getLatestSample().has_value());
    EXPECT_TRUE(buffer.push(createSample(0.5)));
}

} /* namespace */
