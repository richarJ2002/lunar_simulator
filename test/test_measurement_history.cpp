/*!
 * @file            test_measurement_history.cpp
 *
 * @brief           Tests Alpha's stamp-ordered fused-measurement history,
 *                  which defines the order of a rollback replay.
 *
 * @date            25/09/2026
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
#include "objects/MeasurementHistory.h"

namespace
{

using systems::alpha::alpha_localisation::alpha_kalman_filter::FusedMeasurement;
using systems::alpha::alpha_localisation::alpha_kalman_filter::
    MeasurementHistory;
using systems::alpha::alpha_localisation::alpha_kalman_filter::MeasurementKind;

/*!
 * @brief           Creates a record whose first measured value tags it, so
 *                  arrival order can be told apart from stamp order.
 */
FusedMeasurement createRecord(const double timestamp_s_in, const double tag_in)
{
    FusedMeasurement record;
    record.kind              = MeasurementKind::MEASUREMENT_KIND_WHEEL;
    record.timestamp_s       = timestamp_s_in;
    record.measuredValues(0) = tag_in;
    return record;
}

TEST(MeasurementHistory, InsertsDelayedRecordAtItsStampPosition)
{
    MeasurementHistory history;
    ASSERT_TRUE(history.insert(createRecord(1.00, 1.0)));
    ASSERT_TRUE(history.insert(createRecord(1.02, 2.0)));
    ASSERT_TRUE(history.insert(createRecord(1.04, 3.0)));

    /* A visual measurement arrives late, stamped between wheel records. */
    ASSERT_TRUE(history.insert(createRecord(1.01, 4.0)));

    ASSERT_EQ(history.getRecordCount(), 4U);
    const double expectedStamps[] = {1.00, 1.01, 1.02, 1.04};
    const double expectedTags[]   = {1.0, 4.0, 2.0, 3.0};
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        ASSERT_TRUE(history.getRecord(index).has_value());
        EXPECT_DOUBLE_EQ(history.getRecord(index)->timestamp_s,
                         expectedStamps[index]);
        EXPECT_DOUBLE_EQ(history.getRecord(index)->measuredValues(0),
                         expectedTags[index]);
    }
    EXPECT_FALSE(history.getRecord(4U).has_value());
}

TEST(MeasurementHistory, EqualStampsKeepArrivalOrder)
{
    MeasurementHistory history;
    ASSERT_TRUE(history.insert(createRecord(2.0, 1.0)));
    ASSERT_TRUE(history.insert(createRecord(3.0, 2.0)));
    ASSERT_TRUE(history.insert(createRecord(2.0, 3.0)));

    ASSERT_EQ(history.getRecordCount(), 3U);
    EXPECT_DOUBLE_EQ(history.getRecord(0U)->measuredValues(0), 1.0);
    EXPECT_DOUBLE_EQ(history.getRecord(1U)->measuredValues(0), 3.0);
    EXPECT_DOUBLE_EQ(history.getRecord(2U)->measuredValues(0), 2.0);
}

TEST(MeasurementHistory, RejectsNonFiniteStamp)
{
    MeasurementHistory history;
    EXPECT_FALSE(history.insert(
        createRecord(std::numeric_limits<double>::quiet_NaN(), 1.0)));
    EXPECT_FALSE(history.insert(
        createRecord(std::numeric_limits<double>::infinity(), 1.0)));
    EXPECT_EQ(history.getRecordCount(), 0U);
}

TEST(MeasurementHistory, DiscardsOldestAtCapacity)
{
    MeasurementHistory    history;
    constexpr std::size_t HISTORY_CAPACITY = 128U;
    for (std::size_t index = 0U; index < HISTORY_CAPACITY + 2U; ++index)
    {
        ASSERT_TRUE(
            history.insert(createRecord(static_cast<double>(index + 1U), 0.0)));
    }

    ASSERT_EQ(history.getRecordCount(), HISTORY_CAPACITY);
    EXPECT_DOUBLE_EQ(history.getRecord(0U)->timestamp_s, 3.0);
    EXPECT_DOUBLE_EQ(history.getRecord(HISTORY_CAPACITY - 1U)->timestamp_s,
                     130.0);

    /* Older than every retained record: refused, nothing displaced. */
    EXPECT_FALSE(history.insert(createRecord(2.5, 0.0)));
    EXPECT_DOUBLE_EQ(history.getRecord(0U)->timestamp_s, 3.0);

    /* A delayed record inside the retained span displaces the oldest. */
    EXPECT_TRUE(history.insert(createRecord(3.5, 0.0)));
    ASSERT_EQ(history.getRecordCount(), HISTORY_CAPACITY);
    EXPECT_DOUBLE_EQ(history.getRecord(0U)->timestamp_s, 3.5);
    EXPECT_DOUBLE_EQ(history.getRecord(1U)->timestamp_s, 4.0);
}

TEST(MeasurementHistory, ClearRestoresEmptyState)
{
    MeasurementHistory history;
    ASSERT_TRUE(history.insert(createRecord(5.0, 1.0)));

    history.clear();

    EXPECT_EQ(history.getRecordCount(), 0U);
    EXPECT_FALSE(history.getRecord(0U).has_value());
    EXPECT_TRUE(history.insert(createRecord(0.5, 1.0)));
}

} /* namespace */
