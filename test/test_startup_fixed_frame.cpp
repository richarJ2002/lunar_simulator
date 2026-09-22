/**
 * @file            test_startup_fixed_frame.cpp
 *
 * @brief           Verifies startup-fixed transform composition.
 *
 * @date            21/09/2026
 */

#include "objects/GroundTruthNode.h"

#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace
{

using localisation::ground_truth::GroundTruthNode;

void expectTransformNear(const tf2::Transform &actual_in,
                         const tf2::Transform &expected_in)
{
    EXPECT_NEAR(actual_in.getOrigin().x(), expected_in.getOrigin().x(), 1.0e-6);
    EXPECT_NEAR(actual_in.getOrigin().y(), expected_in.getOrigin().y(), 1.0e-6);
    EXPECT_NEAR(actual_in.getOrigin().z(), expected_in.getOrigin().z(), 1.0e-6);

    tf2::Quaternion actualRotation = actual_in.getRotation();
    tf2::Quaternion expectedRotation = expected_in.getRotation();
    actualRotation.normalize();
    expectedRotation.normalize();
    if (actualRotation.dot(expectedRotation) < 0.0)
    {
        expectedRotation = tf2::Quaternion(-expectedRotation.x(),
                                           -expectedRotation.y(),
                                           -expectedRotation.z(),
                                           -expectedRotation.w());
    }
    EXPECT_NEAR(actualRotation.x(), expectedRotation.x(), 1.0e-6);
    EXPECT_NEAR(actualRotation.y(), expectedRotation.y(), 1.0e-6);
    EXPECT_NEAR(actualRotation.z(), expectedRotation.z(), 1.0e-6);
    EXPECT_NEAR(actualRotation.w(), expectedRotation.w(), 1.0e-6);
}

TEST(StartupFixedFrame, FirstPoseRebasesToIdentity)
{
    tf2::Quaternion rotation;
    rotation.setRPY(0.31, -0.22, 1.17);
    const tf2::Transform mapFromFixed(rotation,
                                      tf2::Vector3(4.2, -3.1, 0.73));

    const tf2::Transform fixedFromBody =
        GroundTruthNode::calculateFixedFromBody(mapFromFixed, mapFromFixed);

    expectTransformNear(fixedFromBody, tf2::Transform::getIdentity());
}

TEST(StartupFixedFrame, RebasedPoseRoundTripsToMap)
{
    tf2::Quaternion fixedRotation;
    fixedRotation.setRPY(0.31, -0.22, 1.17);
    const tf2::Transform mapFromFixed(fixedRotation,
                                      tf2::Vector3(4.2, -3.1, 0.73));

    tf2::Quaternion bodyRotation;
    bodyRotation.setRPY(-0.18, 0.27, -0.64);
    const tf2::Transform mapFromBody(bodyRotation,
                                     tf2::Vector3(5.8, -2.4, 1.06));

    const tf2::Transform fixedFromBody =
        GroundTruthNode::calculateFixedFromBody(mapFromFixed, mapFromBody);
    const tf2::Transform reconstructedMapFromBody =
        mapFromFixed * fixedFromBody;

    expectTransformNear(reconstructedMapFromBody, mapFromBody);
}

} /* namespace */
