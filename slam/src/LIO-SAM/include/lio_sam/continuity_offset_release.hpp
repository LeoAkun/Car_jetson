#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace lio_sam
{

struct ContinuityReleaseLimits
{
    double planar_speed_mps = 0.05;
    double vertical_speed_mps = 0.05;
    double angular_speed_radps = 0.5 * M_PI / 180.0;
    double max_dt_sec = 0.1;
    double translation_epsilon_m = 1e-6;
    double rotation_epsilon_rad = 1e-6;
};

struct ContinuityStepLimits
{
    // Preserve the original per-sample guard at the normal 100 Hz cadence.
    double minimum_translation_m = 0.05;
    double maximum_vertical_step_m = 0.05;
    double minimum_yaw_rad = M_PI / 180.0;

    // A delayed sample may account for real vehicle motion, but can never
    // make the guard looser than the scan-to-map hard correction limits.
    double maximum_translation_m = 0.30;
    double maximum_yaw_rad = 5.0 * M_PI / 180.0;
    double maximum_dt_sec = 0.20;

    // Allow estimator noise plus the deliberately rate-limited continuity
    // offset release in addition to motion reported by the odometry sample.
    double translation_margin_m = 0.005;
    double yaw_margin_rad = 0.2 * M_PI / 180.0;
    double release_translation_speed_mps = std::hypot(0.05, 0.05);
    double release_yaw_speed_radps = 0.5 * M_PI / 180.0;

    // Invalid or implausible twist data must not open the guard without
    // bound. These are only clamps for calculating the dynamic allowance.
    double maximum_linear_speed_mps = 2.0;
    double maximum_yaw_speed_radps = 1.0;
};

struct ContinuityStepDecision
{
    bool accepted = false;
    double translation_limit_m = 0.0;
    double vertical_limit_m = 0.0;
    double yaw_limit_rad = 0.0;
};

inline double finiteNonnegativeOrZero(double value)
{
    return std::isfinite(value) && value > 0.0 ? value : 0.0;
}

inline ContinuityStepDecision evaluateContinuityStep(
    double planarStep,
    double verticalStep,
    double yawStep,
    double dt,
    double linearSpeed,
    double yawSpeed,
    const ContinuityStepLimits & limits = ContinuityStepLimits())
{
    const double minimumTranslation =
        finiteNonnegativeOrZero(limits.minimum_translation_m);
    const double minimumYaw = finiteNonnegativeOrZero(limits.minimum_yaw_rad);
    const double maximumTranslation = std::max(
        minimumTranslation,
        finiteNonnegativeOrZero(limits.maximum_translation_m));
    const double maximumYaw = std::max(
        minimumYaw, finiteNonnegativeOrZero(limits.maximum_yaw_rad));
    const double boundedDt = std::min(
        finiteNonnegativeOrZero(dt),
        finiteNonnegativeOrZero(limits.maximum_dt_sec));
    const double boundedLinearSpeed = std::min(
        finiteNonnegativeOrZero(std::abs(linearSpeed)),
        finiteNonnegativeOrZero(limits.maximum_linear_speed_mps));
    const double boundedYawSpeed = std::min(
        finiteNonnegativeOrZero(std::abs(yawSpeed)),
        finiteNonnegativeOrZero(limits.maximum_yaw_speed_radps));

    const double motionTranslation =
        (boundedLinearSpeed + finiteNonnegativeOrZero(
            limits.release_translation_speed_mps)) * boundedDt +
        finiteNonnegativeOrZero(limits.translation_margin_m);
    const double motionYaw =
        (boundedYawSpeed + finiteNonnegativeOrZero(
            limits.release_yaw_speed_radps)) * boundedDt +
        finiteNonnegativeOrZero(limits.yaw_margin_rad);

    ContinuityStepDecision decision;
    decision.translation_limit_m = std::min(
        maximumTranslation, std::max(minimumTranslation, motionTranslation));
    decision.vertical_limit_m = finiteNonnegativeOrZero(
        limits.maximum_vertical_step_m);
    decision.yaw_limit_rad = std::min(
        maximumYaw, std::max(minimumYaw, motionYaw));
    decision.accepted =
        std::isfinite(planarStep) && planarStep >= 0.0 &&
        std::isfinite(verticalStep) && verticalStep >= 0.0 &&
        std::isfinite(yawStep) && yawStep >= 0.0 &&
        planarStep <= decision.translation_limit_m &&
        verticalStep <= decision.vertical_limit_m &&
        yawStep <= decision.yaw_limit_rad;
    return decision;
}

inline Eigen::Isometry3d limitContinuityPoseStep(
    const Eigen::Isometry3d & previous,
    const Eigen::Isometry3d & candidate,
    double maximumTranslation,
    double maximumRotation)
{
    if (!previous.matrix().allFinite() || !candidate.matrix().allFinite())
        return previous;

    Eigen::Isometry3d delta = previous.inverse() * candidate;
    const double translationDistance = delta.translation().norm();
    const double translationLimit = finiteNonnegativeOrZero(maximumTranslation);
    if (translationDistance > translationLimit && translationDistance > 0.0)
        delta.translation() *= translationLimit / translationDistance;

    Eigen::Quaterniond rotation(delta.rotation());
    rotation.normalize();
    const double rotationDistance = Eigen::AngleAxisd(rotation).angle();
    const double rotationLimit = finiteNonnegativeOrZero(maximumRotation);
    if (rotationDistance > rotationLimit && rotationDistance > 0.0)
    {
        delta.linear() = Eigen::Quaterniond::Identity()
            .slerp(rotationLimit / rotationDistance, rotation)
            .normalized().toRotationMatrix();
    }

    return previous * delta;
}

inline bool releaseContinuityOffset(
    const Eigen::Isometry3d & rawCurrent,
    double dt,
    Eigen::Isometry3d & continuityOffset,
    const ContinuityReleaseLimits & limits = ContinuityReleaseLimits())
{
    if (!std::isfinite(dt) || dt <= 0.0 ||
        !rawCurrent.matrix().allFinite() ||
        !continuityOffset.matrix().allFinite())
    {
        return false;
    }

    const double boundedDt = std::min(dt, limits.max_dt_sec);
    const Eigen::Isometry3d current = continuityOffset * rawCurrent;
    Eigen::Isometry3d released = current;

    const Eigen::Vector2d planarRemaining =
        rawCurrent.translation().head<2>() - current.translation().head<2>();
    const double planarDistance = planarRemaining.norm();
    if (planarDistance > limits.translation_epsilon_m)
    {
        const double planarFraction = std::min(
            1.0, limits.planar_speed_mps * boundedDt / planarDistance);
        released.translation().head<2>() += planarFraction * planarRemaining;
    }

    const double verticalRemaining =
        rawCurrent.translation().z() - current.translation().z();
    if (std::abs(verticalRemaining) > limits.translation_epsilon_m)
    {
        const double verticalStep = std::min(
            std::abs(verticalRemaining), limits.vertical_speed_mps * boundedDt);
        released.translation().z() += std::copysign(verticalStep, verticalRemaining);
    }

    Eigen::Quaterniond from(current.rotation());
    Eigen::Quaterniond to(rawCurrent.rotation());
    from.normalize();
    to.normalize();
    const double rotationRemaining =
        Eigen::AngleAxisd(from.inverse() * to).angle();
    if (rotationRemaining > limits.rotation_epsilon_rad)
    {
        const double rotationFraction = std::min(
            1.0, limits.angular_speed_radps * boundedDt / rotationRemaining);
        released.linear() =
            from.slerp(rotationFraction, to).normalized().toRotationMatrix();
    }
    else
    {
        released.linear() = rawCurrent.rotation();
    }

    const Eigen::Vector3d remainingTranslation =
        rawCurrent.translation() - released.translation();
    Eigen::Quaterniond releasedRotation(released.rotation());
    releasedRotation.normalize();
    const double remainingRotation = Eigen::AngleAxisd(
        releasedRotation.inverse() * to).angle();
    const bool stable =
        remainingTranslation.norm() <= limits.translation_epsilon_m &&
        remainingRotation <= limits.rotation_epsilon_rad;

    continuityOffset = stable ?
        Eigen::Isometry3d::Identity() : released * rawCurrent.inverse();
    return stable;
}

}  // namespace lio_sam
