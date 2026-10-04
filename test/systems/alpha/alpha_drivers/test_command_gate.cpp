/*!
 * @file            test_command_gate.cpp
 *
 * @brief           Tests the driver's pure command-gate decision.
 */

#include <gtest/gtest.h>

#include <limits>
#include <optional>

#include "objects/AlphaDriverNodeClass.h"

namespace
{

using Driver      = systems::alpha::alpha_drivers::AlphaDriverNode;
using SystemState = systems::alpha::alpha_supervisor::SystemState;
using Decision    = systems::alpha::alpha_drivers::CommandGateDecision;

constexpr double MAXIMUM_HEARTBEAT_AGE_S = 1.5;

TEST(CommandGate, OpensOnlyForAFreshReadyState)
{
    const Decision decision =
        Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_READY,
                                    0.4,
                                    MAXIMUM_HEARTBEAT_AGE_S);
    EXPECT_TRUE(decision.isOpen);
    EXPECT_TRUE(decision.reason.empty());
}

TEST(CommandGate, ClosedWithoutAnyState)
{
    const Decision decision =
        Driver::evaluateCommandGate(std::nullopt, 0.0, MAXIMUM_HEARTBEAT_AGE_S);
    EXPECT_FALSE(decision.isOpen);
    EXPECT_EQ(decision.reason, "no system state");
}

TEST(CommandGate, ClosedWhileInitialisingOrHeld)
{
    const Decision initialising =
        Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_INITIALISING,
                                    0.1,
                                    MAXIMUM_HEARTBEAT_AGE_S);
    EXPECT_FALSE(initialising.isOpen);
    EXPECT_EQ(initialising.reason, "system INITIALISING");

    const Decision held =
        Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_HOLD,
                                    0.1,
                                    MAXIMUM_HEARTBEAT_AGE_S);
    EXPECT_FALSE(held.isOpen);
    EXPECT_EQ(held.reason, "system HOLD");
}

TEST(CommandGate, StaleHeartbeatClosesAReadyGate)
{
    EXPECT_TRUE(Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_READY,
                                            1.5,
                                            MAXIMUM_HEARTBEAT_AGE_S)
                    .isOpen);
    const Decision stale =
        Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_READY,
                                    1.6,
                                    MAXIMUM_HEARTBEAT_AGE_S);
    EXPECT_FALSE(stale.isOpen);
    EXPECT_EQ(stale.reason, "heartbeat 1.6 s old");
}

TEST(CommandGate, NegativeOrNonFiniteAgeClosesTheGate)
{
    EXPECT_FALSE(Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_READY,
                                             -0.2,
                                             MAXIMUM_HEARTBEAT_AGE_S)
                     .isOpen);
    EXPECT_FALSE(
        Driver::evaluateCommandGate(SystemState::SYSTEM_STATE_READY,
                                    std::numeric_limits<double>::quiet_NaN(),
                                    MAXIMUM_HEARTBEAT_AGE_S)
            .isOpen);
}

} /* namespace */
