/*!
 * @file            shouldEmit.cc
 *
 * @brief           Implements the per-call-site console rate limit.
 *
 * @date            24/09/2026
 */

/* Matching Declaration Include */
#include "console/objects/ThrottleGateClass.h"

namespace common::console
{

bool ThrottleGate::shouldEmit(std::int64_t now_ns, std::int64_t period_ns)
{
    /* Admit the first message, a message after a backwards clock jump, and
     * any message once a full period has elapsed since the last one. */
    const bool isAdmitted = !hasEmitted || now_ns < lastEmission_ns ||
                            now_ns - lastEmission_ns >= period_ns;
    if (isAdmitted)
    {
        hasEmitted      = true;
        lastEmission_ns = now_ns;
    }
    return isAdmitted;
}

} /* namespace common::console */
