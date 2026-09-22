/**
 * @file            test_inertial_gravity.cpp
 *
 * @brief           Verifies tilt-aware stationary gravity removal.
 *
 * @date            21/09/2026
 */

#include "objects/InertialOdometryNode.h"

#include <gtest/gtest.h>

#include <array>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace
{

using localisation::inertial_odometry::InertialOdometryNode;

TEST(InertialGravity, StationaryMeansDefineArbitraryFixedGravityDirections)
{
    const std::array<tf2::Vector3, 3> stationaryMeans{
        tf2::Vector3(0.0, 0.0, 1.70),
        tf2::Vector3(0.84, -1.21, 0.73),
        tf2::Vector3(-1.31, 0.44, -0.91)};

    for (const tf2::Vector3 &stationaryMean : stationaryMeans)
    {
        const tf2::Vector3 gravitySpecificForce =
            InertialOdometryNode::calculateGravitySpecificForceFixed(
                stationaryMean, 1.62);
        EXPECT_NEAR(gravitySpecificForce.length(), 1.62, 1.0e-12);
        const tf2::Vector3 acceleration =
            InertialOdometryNode::calculateGravityFreeAccelerationFixed(
                tf2::Quaternion::getIdentity(),
                gravitySpecificForce,
                gravitySpecificForce);
        EXPECT_NEAR(acceleration.length(), 0.0, 1.0e-12);
    }
}

TEST(InertialGravity, RelativeAttitudeRotatesGravityBeforeRemoval)
{
    const tf2::Vector3 gravitySpecificForceFixed =
        tf2::Vector3(0.84, -1.21, 0.73).normalized() * 1.62;
    tf2::Quaternion orientationBodyToFixed;
    orientationBodyToFixed.setRPY(0.42, -0.31, 0.77);
    orientationBodyToFixed.normalize();
    const tf2::Vector3 specificForceBody =
        tf2::Matrix3x3(orientationBodyToFixed).transpose() *
        gravitySpecificForceFixed;

    const tf2::Vector3 acceleration =
        InertialOdometryNode::calculateGravityFreeAccelerationFixed(
            orientationBodyToFixed,
            specificForceBody,
            gravitySpecificForceFixed);

    EXPECT_NEAR(acceleration.x(), 0.0, 1.0e-12);
    EXPECT_NEAR(acceleration.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(acceleration.z(), 0.0, 1.0e-12);
}

} /* namespace */
