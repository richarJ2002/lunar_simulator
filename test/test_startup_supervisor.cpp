/*!
 * @file            test_startup_supervisor.cpp
 *
 * @brief           Tests the start-up supervisor's pure state machine and
 *                  the SystemState text contract shared with the command
 *                  gate.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "alpha_supervisor/public_functions.h"

namespace
{

namespace supervisor = systems::alpha::alpha_supervisor;
using supervisor::SystemState;

constexpr double MINIMUM_WINDOW_S    = 10.0;
constexpr double COMPONENT_TIMEOUT_S = 2.0;
constexpr double START_S             = 100.0;

/*!
 * @brief           Builds a component that last reported at a given time.
 */
supervisor::ComponentReadiness makeComponent(const std::string &name_in,
                                             bool               isReady_in,
                                             double             reportTime_s_in,
                                             const std::string &reason_in)
{
    supervisor::ComponentReadiness component;
    component.name         = name_in;
    component.hasReport    = true;
    component.isReady      = isReady_in;
    component.reportTime_s = reportTime_s_in;
    component.reason       = reason_in;
    return component;
}

/*!
 * @brief           Five components that all reported ready at a time.
 */
std::vector<supervisor::ComponentReadiness> makeAllReady(double time_s_in)
{
    return {makeComponent("alpha_driver_node", true, time_s_in, "fresh"),
            makeComponent("inertial_odometry", true, time_s_in, "calibrated"),
            makeComponent("visual_odometry", true, time_s_in, "tracking"),
            makeComponent("wheel_odometry", true, time_s_in, "publishing"),
            makeComponent("continuous_ekf", true, time_s_in, "tracking")};
}

supervisor::SupervisorDecision
    evaluate(SystemState                                        previous_in,
             const std::vector<supervisor::ComponentReadiness> &components_in,
             double                                             now_s_in)
{
    return supervisor::evaluateSystemState(previous_in,
                                           components_in,
                                           now_s_in,
                                           START_S,
                                           MINIMUM_WINDOW_S,
                                           COMPONENT_TIMEOUT_S);
}

TEST(StartupSupervisor, EveryComponentReadyAfterWindowIsReady)
{
    const supervisor::SupervisorDecision decision =
        evaluate(SystemState::SYSTEM_STATE_INITIALISING,
                 makeAllReady(111.0),
                 111.5);
    EXPECT_EQ(decision.state, SystemState::SYSTEM_STATE_READY);
    EXPECT_EQ(decision.readyCount, 5U);
    EXPECT_EQ(decision.requiredCount, 5U);
    EXPECT_TRUE(decision.blockingComponent.empty());
}

TEST(StartupSupervisor, MinimumWindowHoldsInitialisation)
{
    const supervisor::SupervisorDecision decision =
        evaluate(SystemState::SYSTEM_STATE_INITIALISING,
                 makeAllReady(105.0),
                 105.0);
    EXPECT_EQ(decision.state, SystemState::SYSTEM_STATE_INITIALISING);
    EXPECT_EQ(decision.readyCount, 5U);
    EXPECT_EQ(decision.blockingReason, "minimum window");
}

TEST(StartupSupervisor, FirstPendingComponentIsReported)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(120.0);
    components[2].isReady = false;
    components[2].reason  = "1/3 consecutive poses";
    components[4].isReady = false;
    const supervisor::SupervisorDecision decision =
        evaluate(SystemState::SYSTEM_STATE_INITIALISING, components, 120.0);
    EXPECT_EQ(decision.state, SystemState::SYSTEM_STATE_INITIALISING);
    EXPECT_EQ(decision.readyCount, 3U);
    EXPECT_EQ(decision.blockingComponent, "visual_odometry");
    EXPECT_EQ(decision.blockingReason, "1/3 consecutive poses");
}

TEST(StartupSupervisor, MissingReportBlocksStartup)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(120.0);
    components[0]      = supervisor::ComponentReadiness{};
    components[0].name = "alpha_driver_node";
    const supervisor::SupervisorDecision decision =
        evaluate(SystemState::SYSTEM_STATE_INITIALISING, components, 120.0);
    EXPECT_EQ(decision.state, SystemState::SYSTEM_STATE_INITIALISING);
    EXPECT_EQ(decision.blockingReason, "no report");
}

TEST(StartupSupervisor, StaleComponentHoldsAReadySystem)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(200.0);
    components[3].reportTime_s = 197.9;
    const supervisor::SupervisorDecision decision =
        evaluate(SystemState::SYSTEM_STATE_READY, components, 200.0);
    EXPECT_EQ(decision.state, SystemState::SYSTEM_STATE_HOLD);
    EXPECT_EQ(decision.blockingComponent, "wheel_odometry");
    EXPECT_EQ(decision.blockingReason, "stale");
}

TEST(StartupSupervisor, ReportExactlyAtTheTimeoutStillCounts)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(200.0);
    components[3].reportTime_s = 198.0;
    EXPECT_EQ(
        evaluate(SystemState::SYSTEM_STATE_READY, components, 200.0).state,
        SystemState::SYSTEM_STATE_READY);
}

TEST(StartupSupervisor, RegressionHoldsAndRecoveryRestoresReady)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(300.0);
    components[4].isReady = false;
    components[4].reason  = "position variance 1.20 m2";
    const supervisor::SupervisorDecision held =
        evaluate(SystemState::SYSTEM_STATE_READY, components, 300.0);
    EXPECT_EQ(held.state, SystemState::SYSTEM_STATE_HOLD);
    EXPECT_EQ(held.blockingReason, "position variance 1.20 m2");

    /* Recovery from HOLD does not wait for the start-up window again. */
    const supervisor::SupervisorDecision recovered =
        supervisor::evaluateSystemState(SystemState::SYSTEM_STATE_HOLD,
                                        makeAllReady(300.5),
                                        300.5,
                                        300.0,
                                        MINIMUM_WINDOW_S,
                                        COMPONENT_TIMEOUT_S);
    EXPECT_EQ(recovered.state, SystemState::SYSTEM_STATE_READY);
}

TEST(StartupSupervisor, HoldPersistsWhileAnyComponentIsNotReady)
{
    std::vector<supervisor::ComponentReadiness> components =
        makeAllReady(300.0);
    components[1].isReady = false;
    EXPECT_EQ(evaluate(SystemState::SYSTEM_STATE_HOLD, components, 300.0).state,
              SystemState::SYSTEM_STATE_HOLD);
}

TEST(StartupSupervisor, StateTextRoundTrips)
{
    for (const SystemState state : {SystemState::SYSTEM_STATE_INITIALISING,
                                    SystemState::SYSTEM_STATE_READY,
                                    SystemState::SYSTEM_STATE_HOLD})
    {
        const std::optional<SystemState> parsed =
            supervisor::parseSystemState(supervisor::systemStateName(state));
        ASSERT_TRUE(parsed.has_value());
        EXPECT_EQ(*parsed, state);
    }
    EXPECT_EQ(supervisor::systemStateName(SystemState::SYSTEM_STATE_READY),
              "READY");
    EXPECT_FALSE(supervisor::parseSystemState("ready").has_value());
    EXPECT_FALSE(supervisor::parseSystemState("").has_value());
}

} /* namespace */
