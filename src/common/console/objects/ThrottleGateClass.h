/*!
 * @file            ThrottleGateClass.h
 *
 * @brief           Declares the per-call-site rate limiter used by the
 *                  throttled console logging macros.
 *
 * @date            24/09/2026
 */

#ifndef LUNAR_SIMULATOR_COMMON_CONSOLE_THROTTLE_GATE_CLASS_H
#define LUNAR_SIMULATOR_COMMON_CONSOLE_THROTTLE_GATE_CLASS_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace common::console
{

/*!
 * @brief           Decides whether a rate-limited message may be emitted.
 *
 * One instance exists per throttled call site (a function-local static
 * created by LUNAR_LOG_*_THROTTLE), matching the per-call-site state of
 * RCLCPP_*_THROTTLE. The caller supplies the current time, so the gate
 * works with simulation or wall time and is deterministic under test. It
 * is not internally synchronised: like RCLCPP_*_THROTTLE it relies on a
 * call site running in one mutually exclusive callback group.
 */
class ThrottleGate
{
  public:
    /*!
     * @brief           Constructs a gate that admits its first message.
     */
    ThrottleGate() noexcept = default;

    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Reports whether a message may be emitted now and, if
     *                  so, records now as the latest emission time.
     *
     *                  The first message is always admitted. A time earlier
     *                  than the latest emission (a simulation reset or
     *                  clock jump) also admits the message and restarts the
     *                  period from that time.
     *
     * @param[in]       now_ns
     *                  Current time of the caller's clock.
     *
     * @param[in]       period_ns
     *                  Minimum interval between admitted messages; a
     *                  non-positive period admits every message.
     *
     * @return          True when the message may be emitted.
     */
    [[nodiscard]] bool shouldEmit(std::int64_t now_ns, std::int64_t period_ns);

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief           Whether any message has been admitted yet.
     *
     * @frame           N/A
     * @units           N/A
     */
    bool hasEmitted{false};

    /*!
     * @brief           Time of the latest admitted message.
     *
     * @frame           N/A
     * @units           nanoseconds of the caller's clock
     */
    std::int64_t lastEmission_ns{0};
};

} /* namespace common::console */

#endif /* LUNAR_SIMULATOR_COMMON_CONSOLE_THROTTLE_GATE_CLASS_H */
