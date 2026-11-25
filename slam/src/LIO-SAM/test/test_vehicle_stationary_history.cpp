#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "vehicle_stationary_history.hpp"

namespace
{
constexpr double kTimeoutSec = 0.25;
constexpr int kMinSamples = 5;
constexpr double kStationarySpeedKmh = 0.10;

constexpr int64_t stampMs(int64_t offsetMs)
{
    return 1795407558000000000LL + offsetMs * 1000000;
}

class VehicleStationaryHistoryTest : public testing::Test
{
protected:
    void addStationarySamples(int64_t firstOffsetMs, int count)
    {
        for (int index = 0; index < count; ++index)
            history_.add(stampMs(firstOffsetMs + index * 10), 0.0);
    }

    auto assess(int64_t offsetMs) const
    {
        return history_.evaluate(
            stampMs(offsetMs), kTimeoutSec, kMinSamples, kStationarySpeedKmh);
    }

    lio_sam::VehicleStationaryHistory history_;
};

TEST_F(VehicleStationaryHistoryTest, SelectsRecentStationarySamplesAtCorrectionTime)
{
    addStationarySamples(0, 5);

    const auto decision = assess(45);

    EXPECT_TRUE(decision.stationary);
    EXPECT_EQ(decision.sample_stamp_ns, stampMs(40));
    EXPECT_DOUBLE_EQ(decision.speed_kmh, 0.0);
}

TEST_F(VehicleStationaryHistoryTest, LaterStationaryFeedbackCannotOverrideEarlierReverseMotion)
{
    addStationarySamples(0, 5);
    history_.add(stampMs(919), 0.4);
    history_.add(stampMs(929), 0.4);
    addStationarySamples(1246, 5);

    const auto moving = assess(926);
    const auto stopped = assess(1287);

    EXPECT_FALSE(moving.stationary);
    EXPECT_EQ(moving.sample_stamp_ns, stampMs(919));
    EXPECT_DOUBLE_EQ(moving.speed_kmh, 0.4);
    EXPECT_TRUE(stopped.stationary);
    EXPECT_EQ(stopped.sample_stamp_ns, stampMs(1286));
}

TEST_F(VehicleStationaryHistoryTest, NeverCountsStationarySamplesFromAfterCorrection)
{
    addStationarySamples(0, 7);

    EXPECT_FALSE(assess(-1).stationary);
    EXPECT_FALSE(assess(39).stationary);
    EXPECT_TRUE(assess(40).stationary);
}

TEST_F(VehicleStationaryHistoryTest, RequiresAllStationaryEvidenceWithinFreshnessWindow)
{
    history_.add(stampMs(0), 0.0);
    history_.add(stampMs(70), 0.0);
    history_.add(stampMs(140), 0.0);
    history_.add(stampMs(210), 0.0);
    history_.add(stampMs(251), 0.0);

    EXPECT_FALSE(assess(251).stationary);

    history_.add(stampMs(261), 0.0);

    EXPECT_TRUE(assess(261).stationary);
}

TEST_F(VehicleStationaryHistoryTest, GapsBetweenSamplesBreakTheStationarySequence)
{
    addStationarySamples(0, 4);
    history_.add(stampMs(131), 0.0);

    EXPECT_FALSE(assess(132).stationary);

    addStationarySamples(141, 4);

    EXPECT_TRUE(assess(171).stationary);
}

TEST_F(VehicleStationaryHistoryTest, GapFromLatestSampleToCorrectionInvalidatesStationaryEvidence)
{
    addStationarySamples(0, 5);

    EXPECT_TRUE(assess(140).stationary);
    EXPECT_FALSE(assess(141).stationary);
}

TEST_F(VehicleStationaryHistoryTest, MotionBreaksTheStationarySequence)
{
    addStationarySamples(0, 5);
    history_.add(stampMs(50), -0.2);
    addStationarySamples(60, 4);

    EXPECT_FALSE(assess(50).stationary);
    EXPECT_FALSE(assess(90).stationary);

    history_.add(stampMs(100), 0.0);
    EXPECT_TRUE(assess(100).stationary);
}

TEST_F(VehicleStationaryHistoryTest, InvalidSpeedBreaksTheStationarySequence)
{
    addStationarySamples(0, 5);
    history_.add(stampMs(50), std::numeric_limits<double>::quiet_NaN());
    addStationarySamples(60, 4);

    EXPECT_FALSE(assess(50).stationary);
    EXPECT_FALSE(assess(90).stationary);

    history_.add(stampMs(100), 0.0);
    EXPECT_TRUE(assess(100).stationary);
}

TEST_F(VehicleStationaryHistoryTest, InvalidTimestampsAreNotStationaryEvidence)
{
    EXPECT_FALSE(assess(10).stationary);

    history_.add(-10000000, 0.0);
    history_.add(-1, 0.0);
    history_.add(0, 0.0);
    history_.add(10000000, 0.0);
    history_.add(20000000, 0.0);

    const auto too_few_samples = history_.evaluate(
        20000000, kTimeoutSec, kMinSamples, kStationarySpeedKmh);
    EXPECT_FALSE(too_few_samples.stationary);
    EXPECT_FALSE(history_.evaluate(0, kTimeoutSec, 1, kStationarySpeedKmh).stationary);
    EXPECT_FALSE(history_.evaluate(-1, kTimeoutSec, 1, kStationarySpeedKmh).stationary);
}

TEST_F(VehicleStationaryHistoryTest, DuplicateTimestampsDoNotIncreaseSampleCount)
{
    addStationarySamples(0, 4);
    history_.add(stampMs(30), 0.0);
    history_.add(stampMs(30), 0.0);

    EXPECT_FALSE(assess(35).stationary);

    history_.add(stampMs(40), 0.0);
    EXPECT_TRUE(assess(40).stationary);
}

TEST_F(VehicleStationaryHistoryTest, ConflictingDuplicateRemainsInvalidAfterAnotherDuplicate)
{
    addStationarySamples(0, 5);
    history_.add(stampMs(40), 0.4);
    history_.add(stampMs(40), 0.0);

    const auto decision = assess(45);

    EXPECT_FALSE(decision.stationary);
    EXPECT_EQ(decision.sample_stamp_ns, stampMs(40));

    addStationarySamples(50, 5);
    EXPECT_TRUE(assess(90).stationary);
}

TEST_F(VehicleStationaryHistoryTest, OrdersDelayedFeedbackByTimestamp)
{
    addStationarySamples(0, 4);
    history_.add(stampMs(50), 0.0);
    history_.add(stampMs(40), 0.0);

    const auto decision = assess(45);

    EXPECT_TRUE(decision.stationary);
    EXPECT_EQ(decision.sample_stamp_ns, stampMs(40));
    EXPECT_DOUBLE_EQ(decision.speed_kmh, 0.0);
}
}  // namespace
