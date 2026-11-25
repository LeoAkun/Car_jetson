#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "continuity_offset_release.hpp"

namespace
{
constexpr double kDt = 0.01;
constexpr double kPlanarStep = 0.05 * kDt;
constexpr double kVerticalStep = 0.05 * kDt;
constexpr double kAngularStep = 0.5 * M_PI / 180.0 * kDt;

Eigen::Isometry3d pose(
    double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(x, y, z);
    result.linear() = (
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    return result;
}

double rotationDistance(
    const Eigen::Isometry3d & first, const Eigen::Isometry3d & second)
{
    return Eigen::AngleAxisd(
        Eigen::Quaterniond(first.rotation()).inverse() *
        Eigen::Quaterniond(second.rotation())).angle();
}

TEST(ContinuityOffsetRelease, BoundsPlanarVerticalAndAngularSteps)
{
    const Eigen::Isometry3d raw = pose(500.0, -80.0, 10.0, 0.1, -0.05, 0.4);
    const Eigen::Isometry3d current = pose(497.0, -76.0, 7.0, -0.2, 0.1, -0.3);
    Eigen::Isometry3d offset = current * raw.inverse();

    EXPECT_FALSE(lio_sam::releaseContinuityOffset(raw, kDt, offset));
    const Eigen::Isometry3d released = offset * raw;

    EXPECT_NEAR(
        (released.translation().head<2>() - current.translation().head<2>()).norm(),
        kPlanarStep, 1e-9);
    EXPECT_NEAR(
        std::abs(released.translation().z() - current.translation().z()),
        kVerticalStep, 1e-9);
    EXPECT_NEAR(rotationDistance(current, released), kAngularStep, 1e-9);
}

TEST(ContinuityOffsetRelease, RotationReleaseDoesNotDependOnMapOrigin)
{
    const Eigen::Isometry3d rawNear = pose(5.0, 8.0, 1.0, 0.0, 0.0, 0.2);
    const Eigen::Isometry3d currentNear = pose(4.0, 7.0, 0.0, 0.0, 0.0, -0.2);
    const Eigen::Vector3d shift(1000.0, -700.0, 30.0);
    Eigen::Isometry3d rawFar = rawNear;
    Eigen::Isometry3d currentFar = currentNear;
    rawFar.translation() += shift;
    currentFar.translation() += shift;
    Eigen::Isometry3d nearOffset = currentNear * rawNear.inverse();
    Eigen::Isometry3d farOffset = currentFar * rawFar.inverse();

    lio_sam::releaseContinuityOffset(rawNear, kDt, nearOffset);
    lio_sam::releaseContinuityOffset(rawFar, kDt, farOffset);
    const Eigen::Isometry3d nearReleased = nearOffset * rawNear;
    const Eigen::Isometry3d farReleased = farOffset * rawFar;

    EXPECT_NEAR(
        ((farReleased.translation() - shift) - nearReleased.translation()).norm(),
        0.0, 1e-9);
    EXPECT_NEAR(
        rotationDistance(nearReleased, farReleased), 0.0, 1e-9);
}

TEST(ContinuityOffsetRelease, CapsReleaseTimeAfterADataGap)
{
    const Eigen::Isometry3d raw = pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Eigen::Isometry3d current = pose(-1.0, 0.0, -1.0, 0.0, 0.0, -0.5);
    Eigen::Isometry3d offset = current * raw.inverse();

    lio_sam::releaseContinuityOffset(raw, 2.0, offset);
    const Eigen::Isometry3d released = offset * raw;

    EXPECT_NEAR(
        (released.translation().head<2>() - current.translation().head<2>()).norm(),
        0.005, 1e-9);
    EXPECT_NEAR(released.translation().z() - current.translation().z(), 0.005, 1e-9);
    EXPECT_NEAR(
        rotationDistance(current, released), 0.05 * M_PI / 180.0, 1e-9);
}

TEST(ContinuityOffsetRelease, ConvergesAcrossRepeatedEpochReanchoring)
{
    const Eigen::Isometry3d raw = pose(80.0, 3.0, 0.0, 0.0, 0.0, 0.0);
    Eigen::Isometry3d lastPublished = pose(80.0, 3.0, -0.5, 0.0, 0.0, 0.0);
    Eigen::Isometry3d offset = lastPublished * raw.inverse();

    for (int sample = 0; sample < 1000; ++sample)
    {
        if (sample % 10 == 0)
            offset = lastPublished * raw.inverse();
        lio_sam::releaseContinuityOffset(raw, kDt, offset);
        lastPublished = offset * raw;
    }

    EXPECT_NEAR(lastPublished.translation().z(), raw.translation().z(), 1e-6);
}

TEST(ContinuityOffsetRelease, InvalidTimeDoesNotChangeOffset)
{
    const Eigen::Isometry3d raw = pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Eigen::Isometry3d current = pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    Eigen::Isometry3d offset = current * raw.inverse();
    const Eigen::Matrix4d before = offset.matrix();

    EXPECT_FALSE(lio_sam::releaseContinuityOffset(raw, -1.0, offset));
    EXPECT_TRUE(offset.matrix().isApprox(before));
}

TEST(ContinuityStepGuard, UsesConfiguredOutputLimitAtNormalCadence)
{
    const auto accepted = lio_sam::evaluateContinuityStep(
        0.10, 0.0, 3.0 * M_PI / 180.0, 0.01, 0.5, 0.25);
    EXPECT_TRUE(accepted.accepted);
    EXPECT_NEAR(accepted.translation_limit_m, 0.10, 1e-12);
    EXPECT_NEAR(accepted.yaw_limit_rad, 3.0 * M_PI / 180.0, 1e-12);

    const auto translationJump = lio_sam::evaluateContinuityStep(
        0.101, 0.0, 0.0, 0.01, 0.5, 0.0);
    EXPECT_FALSE(translationJump.accepted);
    const auto yawJump = lio_sam::evaluateContinuityStep(
        0.0, 0.0, 3.01 * M_PI / 180.0, 0.01, 0.0, 0.25);
    EXPECT_FALSE(yawJump.accepted);
}

TEST(ContinuityStepGuard, AcceptsRecordedDelayedTranslationSamples)
{
    struct Sample
    {
        double step;
        double vertical;
        double dt;
        double speed;
    };
    const Sample samples[] = {
        {0.051343, 0.005255, 0.089955, 0.513038},
        {0.050248, 0.000024, 0.090031, 0.503579},
        {0.050830, 0.001221, 0.090004, 0.489031},
    };

    for (const auto & sample : samples)
    {
        const auto decision = lio_sam::evaluateContinuityStep(
            sample.step, sample.vertical, 0.0,
            sample.dt, sample.speed, 0.0);
        EXPECT_TRUE(decision.accepted);
        EXPECT_GE(decision.translation_limit_m, sample.step);
        EXPECT_NEAR(decision.translation_limit_m, 0.10, 1e-12);
    }
}

TEST(ContinuityStepGuard, AcceptsRecordedDelayedYawSamples)
{
    const auto first = lio_sam::evaluateContinuityStep(
        0.051343, 0.0, 0.023260, 0.089955, 0.513038, 0.259042);
    EXPECT_TRUE(first.accepted);
    EXPECT_GT(first.yaw_limit_rad, 0.023260);

    const auto second = lio_sam::evaluateContinuityStep(
        0.025129, 0.0, 0.018687, 0.050021, 0.400574, 0.364668);
    EXPECT_TRUE(second.accepted);
    EXPECT_GT(second.yaw_limit_rad, 0.018687);
}

TEST(ContinuityStepGuard, LimitsAcceptedDelayedOutputToOriginalStepBounds)
{
    const Eigen::Isometry3d previous = pose(
        10.0, -2.0, 0.1, 0.0, 0.0, -0.3);
    const Eigen::Isometry3d candidate = previous * pose(
        0.12, 0.0, 0.0, 0.0, 0.0, 0.07);

    const Eigen::Isometry3d limited = lio_sam::limitContinuityPoseStep(
        previous, candidate, 0.10, 3.0 * M_PI / 180.0);
    const Eigen::Isometry3d delta = previous.inverse() * limited;

    EXPECT_NEAR(delta.translation().norm(), 0.10, 1e-12);
    EXPECT_NEAR(
        rotationDistance(previous, limited), 3.0 * M_PI / 180.0, 1e-12);
}

TEST(ContinuityStepGuard, StillRejectsHardJumpsAfterDelay)
{
    const auto translationJump = lio_sam::evaluateContinuityStep(
        0.51, 0.0, 0.0, 2.0, 50.0, 0.0);
    EXPECT_FALSE(translationJump.accepted);
    EXPECT_LE(translationJump.translation_limit_m, 0.50);
    EXPECT_GT(translationJump.translation_limit_m, 0.10);

    const auto yawJump = lio_sam::evaluateContinuityStep(
        0.0, 0.0, 10.1 * M_PI / 180.0, 2.0, 0.0, 50.0);
    EXPECT_FALSE(yawJump.accepted);
    EXPECT_NEAR(yawJump.yaw_limit_rad, 10.0 * M_PI / 180.0, 1e-12);
}

TEST(ContinuityStepGuard, VehicleSpeedCannotExcuseVerticalJump)
{
    const auto decision = lio_sam::evaluateContinuityStep(
        0.0, 0.051, 0.0, 0.20, 2.0, 0.0);
    EXPECT_FALSE(decision.accepted);
    EXPECT_NEAR(decision.vertical_limit_m, 0.05, 1e-12);
}

TEST(ContinuityStepGuard, ClassifiesOnlyHardBoundaryViolationsAsStrong)
{
    EXPECT_FALSE(lio_sam::isStrongContinuityViolation(
        0.49, 0.049, 9.9 * M_PI / 180.0));
    EXPECT_TRUE(lio_sam::isStrongContinuityViolation(
        0.51, 0.0, 0.0));
    EXPECT_TRUE(lio_sam::isStrongContinuityViolation(
        0.0, 0.051, 0.0));
    EXPECT_TRUE(lio_sam::isStrongContinuityViolation(
        0.0, 0.0, 10.1 * M_PI / 180.0));
}

TEST(ContinuityStepGuard, InvalidTwistCannotDisableGuard)
{
    const auto decision = lio_sam::evaluateContinuityStep(
        0.11, 0.0, 0.0, 0.01,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity());
    EXPECT_FALSE(decision.accepted);
    EXPECT_NEAR(decision.translation_limit_m, 0.10, 1e-12);
    EXPECT_NEAR(decision.yaw_limit_rad, 3.0 * M_PI / 180.0, 1e-12);
}

TEST(ContinuityPrediction, UsesLastLegalPlanarMotionWithinOutputLimits)
{
    const Eigen::Isometry3d previous = pose(
        1.0, 2.0, 0.3, 0.0, 0.0, 0.0);
    const Eigen::Isometry3d predicted = lio_sam::predictBoundedPlanarMotion(
        previous, 0.20, 2.0, 0.0, 1.0);
    const Eigen::Isometry3d delta = previous.inverse() * predicted;

    EXPECT_NEAR(delta.translation().norm(), 0.10, 1e-12);
    EXPECT_NEAR(rotationDistance(previous, predicted),
        3.0 * M_PI / 180.0, 1e-12);
    EXPECT_NEAR(predicted.translation().z(), previous.translation().z(), 1e-12);
}

TEST(ContinuityRecovery, RecoversAfterThreeConsecutiveValidSamples)
{
    lio_sam::ContinuityRecoveryTracker tracker;
    EXPECT_TRUE(tracker.observeInvalid(1.0, false).entered_suspect);
    EXPECT_FALSE(tracker.observeValid(1.01).recovered);
    EXPECT_FALSE(tracker.observeValid(1.02).recovered);
    EXPECT_TRUE(tracker.observeValid(1.03).recovered);
    EXPECT_FALSE(tracker.suspect());
}

TEST(ContinuityRecovery, FaultsAtTenInvalidSamplesOrHalfSecond)
{
    lio_sam::ContinuityRecoveryTracker countTracker;
    for (int index = 0; index < 9; ++index)
        EXPECT_FALSE(countTracker.observeInvalid(1.0 + 0.01 * index, false)
            .fault_confirmed);
    EXPECT_TRUE(countTracker.observeInvalid(1.09, false).fault_confirmed);

    lio_sam::ContinuityRecoveryTracker timeTracker;
    EXPECT_FALSE(timeTracker.observeInvalid(2.0, false).fault_confirmed);
    EXPECT_TRUE(timeTracker.checkTimeout(2.50).fault_confirmed);
}

TEST(ContinuityRecovery, StrongFaultRequiresThreeSamplesOrPointOneSecond)
{
    lio_sam::ContinuityRecoveryTracker sampleTracker;
    EXPECT_FALSE(sampleTracker.observeInvalid(1.0, true).fault_confirmed);
    EXPECT_FALSE(sampleTracker.observeInvalid(1.01, true).fault_confirmed);
    EXPECT_TRUE(sampleTracker.observeInvalid(1.02, true).fault_confirmed);

    lio_sam::ContinuityRecoveryTracker timeTracker;
    EXPECT_FALSE(timeTracker.observeInvalid(2.0, true).fault_confirmed);
    EXPECT_TRUE(timeTracker.checkTimeout(2.10).fault_confirmed);
}

TEST(ContinuityRecovery, InvalidSampleBreaksRecoveryStreak)
{
    lio_sam::ContinuityRecoveryTracker tracker;
    tracker.observeInvalid(1.0, false);
    tracker.observeValid(1.01);
    tracker.observeValid(1.02);
    tracker.observeInvalid(1.03, false);
    EXPECT_FALSE(tracker.observeValid(1.04).recovered);
    EXPECT_FALSE(tracker.observeValid(1.05).recovered);
    EXPECT_TRUE(tracker.observeValid(1.06).recovered);
}
}  // namespace
