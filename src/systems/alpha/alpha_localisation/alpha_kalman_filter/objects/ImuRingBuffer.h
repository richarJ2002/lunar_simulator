/**
 * @file            ImuRingBuffer.h
 *
 * @brief           Declares Alpha's fixed-capacity filtered-IMU ring buffer.
 *
 * @date            20/09/2026
 */

#ifndef LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_RING_BUFFER_H
#define LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_RING_BUFFER_H

/* C++ Standard Library Includes */
#include <array>
#include <cstddef>
#include <optional>

/* C Standard Library Includes */
/* None */

/* External Library Includes */
/* None */

/* Other Project Module Includes */
/* None */

/* Object Includes */
#include "objects/ImuSample.h"

namespace systems::alpha::alpha_localisation::alpha_kalman_filter
{

/**
 * @brief           Retains recent IMU samples in chronological order.
 *
 * The filter node owns one buffer and accesses it only through its mutually
 * exclusive callback group. Samples are copied into fixed storage, so no
 * borrowed ROS message survives its callback and no allocation occurs while
 * prediction is running. When capacity is reached, the oldest sample is
 * overwritten because it is already older than the filter's retained time.
 */
class ImuRingBuffer
{
  public:
    /**
     * @brief           Constructs an empty IMU ring buffer.
     */
    ImuRingBuffer() noexcept = default;

    /**
     * @brief           Destroys the ring buffer without external effects.
     */
    ~ImuRingBuffer() noexcept = default;

    ImuRingBuffer(const ImuRingBuffer &otherBuffer_in)            = delete;
    ImuRingBuffer &operator=(const ImuRingBuffer &otherBuffer_in) = delete;
    ImuRingBuffer(ImuRingBuffer &&otherBuffer_in)                 = delete;
    ImuRingBuffer &operator=(ImuRingBuffer &&otherBuffer_in)      = delete;

    /**
     * @brief           Inserts one sample in timestamp order.
     *
     * A sample whose timestamp is not newer than the latest retained sample
     * is rejected so prediction never consumes duplicate or reversed time.
     *
     * @param[in]       sample_in
     *                  Finite filtered IMU sample to retain.
     *
     * @return          True when the sample was stored.
     */
    [[nodiscard]] bool push(const ImuSample &sample_in) noexcept;

    /**
     * @brief           Returns one sample by chronological position.
     *
     * @param[in]       sampleIndex_in
     *                  Zero-based index from the oldest retained sample.
     *
     * @return          Requested sample, or no value when the index is out of
     *                  range.
     */
    [[nodiscard]] std::optional<ImuSample>
        getSample(std::size_t sampleIndex_in) const noexcept;

    /**
     * @brief           Returns the newest retained sample.
     *
     * @return          Newest sample, or no value when the buffer is empty.
     */
    [[nodiscard]] std::optional<ImuSample> getLatestSample() const noexcept;

    /**
     * @brief           Returns the number of retained samples.
     *
     * @return          Number of samples in chronological storage.
     */
    [[nodiscard]] std::size_t getSampleCount() const noexcept;

    /**
     * @brief           Removes every retained sample.
     */
    void clear() noexcept;

  private:
    /**
     * @brief           Maximum number of retained IMU samples.
     *
     * @frame           N/A
     * @units           samples
     */
    static constexpr std::size_t CAPACITY = 512U;

    /**
     * @brief           Fixed storage containing copied IMU samples.
     *
     * @frame           N/A
     * @units           N/A
     */
    std::array<ImuSample, CAPACITY> samples{};

    /**
     * @brief           Storage index where the next sample will be written.
     *
     * @frame           N/A
     * @units           array elements
     */
    std::size_t nextWriteIndex{0U};

    /**
     * @brief           Number of valid samples currently retained.
     *
     * @frame           N/A
     * @units           samples
     */
    std::size_t sampleCount{0U};
};

} /* namespace systems::alpha::alpha_localisation::alpha_kalman_filter */

#endif /* LUNAR_SIMULATOR_SYSTEMS_ALPHA_ALPHA_LOCALISATION_ALPHA_KALMAN_FILTER_IMU_RING_BUFFER_H \
        */
