/**
 * @file            SourceDiagnostics.h
 *
 * @brief           Declares bounded estimator source diagnostics.
 *
 * @date            21/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_H

/* C++ Standard Library Includes */
#include <cstdint>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/**
 * @brief           Accumulates one source's counts and latest timing values.
 */
struct SourceDiagnostics
{
public:
    /** @brief Number of callbacks admitted. */
    std::uint64_t receivedCount{0U};

    /** @brief Number of measurements passing boundary and age validation. */
    std::uint64_t acceptedCount{0U};

    /** @brief Number of measurements rejected for age or timestamp order. */
    std::uint64_t ageRejectedCount{0U};

    /** @brief Number of measurements rejected by a statistical gate. */
    std::uint64_t nisRejectedCount{0U};

    /** @brief Number of malformed or numerically rejected measurements. */
    std::uint64_t numericalRejectedCount{0U};

    /** @brief Number of measurements successfully applied to the estimate. */
    std::uint64_t fusedCount{0U};

    /** @brief Latest source publication timestamp in ROS seconds. */
    double publicationTimestamp_s{0.0};

    /** @brief Latest callback admission timestamp in ROS seconds. */
    double callbackAdmissionTimestamp_s{0.0};

    /** @brief Latest processing-start timestamp in ROS seconds. */
    double processingStartTimestamp_s{0.0};

    /** @brief Latest callback completion timestamp in ROS seconds. */
    double processingEndTimestamp_s{0.0};

    /** @brief Estimator epoch after the latest callback, in ROS seconds. */
    double estimatorTimestamp_s{0.0};

    /** @brief Latest normalized innovation squared value. */
    double normalizedInnovationSquared{0.0};

    /** @brief Norm of the latest posterior error-state correction. */
    double correctionNorm{0.0};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_SOURCE_DIAGNOSTICS_H */
