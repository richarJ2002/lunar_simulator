/**
 * @file            logDiagnostics.cc
 *
 * @brief           Implements periodic estimator diagnostics logging.
 *
 * @date            21/09/2026
 */

/* Matching Declaration Include */
#include "objects/AlphaKalmanFilterNode.h"

/* External Library Includes */
#include <Eigen/Eigenvalues>

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

void AlphaKalmanFilterNode::logDiagnostics()
{
    double covarianceTrace             = 0.0;
    double minimumCovarianceEigenvalue = 0.0;
    double minimumCovarianceDiagonal   = 0.0;
    double maximumCovarianceDiagonal   = 0.0;
    if (hasInitialState && latestCovariance.allFinite())
    {
        covarianceTrace           = latestCovariance.trace();
        minimumCovarianceDiagonal = latestCovariance.diagonal().minCoeff();
        maximumCovarianceDiagonal = latestCovariance.diagonal().maxCoeff();
        const Eigen::SelfAdjointEigenSolver<ErrorStateMatrix> decomposition(
            0.5 * (latestCovariance + latestCovariance.transpose()),
            Eigen::EigenvaluesOnly);
        if (decomposition.info() == Eigen::Success)
        {
            minimumCovarianceEigenvalue =
                decomposition.eigenvalues().minCoeff();
        }
    }

    double          quaternionNorm    = 0.0;
    Eigen::Vector3d accelerometerBias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscopeBias     = Eigen::Vector3d::Zero();
    if (hasInitialState)
    {
        const Eigen::Index quaternionIndex =
            static_cast<Eigen::Index>(StateIndex::STATE_INDEX_QUATERNION_X);
        quaternionNorm    = latestState.segment<4>(quaternionIndex).norm();
        accelerometerBias = latestState.segment<3>(static_cast<Eigen::Index>(
            StateIndex::STATE_INDEX_ACCELEROMETER_BIAS_X));
        gyroscopeBias     = latestState.segment<3>(static_cast<Eigen::Index>(
            StateIndex::STATE_INDEX_GYROSCOPE_BIAS_X));
    }

    const SourceDiagnostics *sources[] = {&imuDiagnostics,
                                          &visualDiagnostics,
                                          &wheelDiagnostics};
    const char              *names[]   = {"imu", "visual", "wheel"};
    for (std::size_t sourceIndex = 0U; sourceIndex < 3U; ++sourceIndex)
    {
        const SourceDiagnostics &source = *sources[sourceIndex];
        RCLCPP_INFO(
            get_logger(),
            "localisation_diag source=%s received=%llu accepted=%llu "
            "age_rejected=%llu nis_rejected=%llu numerical_rejected=%llu "
            "fused=%llu publication=%.6f admission=%.6f start=%.6f end=%.6f "
            "estimator_epoch=%.6f nis=%.6f correction_norm=%.6f "
            "covariance_trace=%.6f covariance_min_eigenvalue=%.6e "
            "covariance_diagonal_range=[%.6e,%.6e] quaternion_norm=%.12f "
            "accel_bias_body_mps2=[%.6f,%.6f,%.6f] "
            "gyro_bias_body_radps=[%.6f,%.6f,%.6f]",
            names[sourceIndex],
            static_cast<unsigned long long>(source.receivedCount),
            static_cast<unsigned long long>(source.acceptedCount),
            static_cast<unsigned long long>(source.ageRejectedCount),
            static_cast<unsigned long long>(source.nisRejectedCount),
            static_cast<unsigned long long>(source.numericalRejectedCount),
            static_cast<unsigned long long>(source.fusedCount),
            source.publicationTimestamp_s,
            source.callbackAdmissionTimestamp_s,
            source.processingStartTimestamp_s,
            source.processingEndTimestamp_s,
            source.estimatorTimestamp_s,
            source.normalizedInnovationSquared,
            source.correctionNorm,
            covarianceTrace,
            minimumCovarianceEigenvalue,
            minimumCovarianceDiagonal,
            maximumCovarianceDiagonal,
            quaternionNorm,
            accelerometerBias.x(),
            accelerometerBias.y(),
            accelerometerBias.z(),
            gyroscopeBias.x(),
            gyroscopeBias.y(),
            gyroscopeBias.z());
    }
}

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */
