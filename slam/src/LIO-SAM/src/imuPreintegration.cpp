#include "utility.hpp"

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <map>
#include <sstream>
#include <string>
#include <thread>

#include <multi_map_navigation_msgs/msg/epoch_odometry.hpp>
#include <multi_map_navigation_msgs/msg/lio_correction_event.hpp>
#include <multi_map_navigation_msgs/msg/lio_map_odom_correction.hpp>
#include <multi_map_navigation_msgs/msg/lio_readiness.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <yunle_msgs/msg/vehicle_status.hpp>
#include "vehicle_stationary_history.hpp"
#include "continuity_offset_release.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/linearExceptions.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

namespace
{
constexpr double kDefaultImuDt = 1.0 / 100.0;
constexpr double kMinImuDt = 1e-6;
constexpr size_t kReadyOptimizationCount = 3;
constexpr double kSmallCorrectionTranslation = 0.05;
constexpr double kSmallCorrectionYaw = M_PI / 180.0;

bool isPositiveImuDt(double dt)
{
    return std::isfinite(dt) && dt > kMinImuDt;
}

bool isFiniteImuMeasurement(const sensor_msgs::msg::Imu & imu)
{
    return std::isfinite(imu.linear_acceleration.x) &&
           std::isfinite(imu.linear_acceleration.y) &&
           std::isfinite(imu.linear_acceleration.z) &&
           std::isfinite(imu.angular_velocity.x) &&
           std::isfinite(imu.angular_velocity.y) &&
           std::isfinite(imu.angular_velocity.z);
}
} // namespace

class TransformFusion : public ParamServer
{
public:
    struct Anchor
    {
        bool valid = false;
        std::string instanceId;
        uint64_t generation = 0;
        uint64_t epoch = 0;
        bool accepted = false;
        int64_t stamp = 0;
        Eigen::Isometry3d optimizedPose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d correctedIncrementalPose = Eigen::Isometry3d::Identity();
        double translationResidual = 0.0;
        double yawResidual = 0.0;
    };

    enum class CorrectionState
    {
        WAITING_FOR_EPOCH,
        STABLE,
        RELEASING_SMALL,
        REJECTED_LARGE,
        CONTINUITY_FAULT
    };

    std::mutex mtx;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;
    rclcpp::Subscription<multi_map_navigation_msgs::msg::EpochOdometry>::SharedPtr
        subEpochOdometry;
    rclcpp::Subscription<multi_map_navigation_msgs::msg::LioCorrectionEvent>::SharedPtr
        subCorrectionEvent;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubLocalizationFault;
    rclcpp::Publisher<multi_map_navigation_msgs::msg::LioMapOdomCorrection>::SharedPtr
        pubMapOdomCorrection;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2::Stamped<tf2::Transform> lidar2Baselink;

    std::map<int64_t, nav_msgs::msg::Odometry> mappingAnchorsByStamp;
    std::map<int64_t, multi_map_navigation_msgs::msg::LioCorrectionEvent>
        scanEventsByStamp;
    Anchor activeAnchor;
    uint64_t outputEpoch = 0;
    CorrectionState state = CorrectionState::WAITING_FOR_EPOCH;
    Eigen::Isometry3d continuityOffset = Eigen::Isometry3d::Identity();
    bool hasLastPublishedPose = false;
    bool faultLatched = false;
    bool continuityOffsetReleasePending = false;
    Eigen::Isometry3d lastPublishedPose = Eigen::Isometry3d::Identity();
    int64_t lastPublishedStamp = 0;
    lio_sam::ContinuityStepLimits continuityStepLimits;
    nav_msgs::msg::Path imuPath;
    double lastPathTime = -1.0;

    TransformFusion(const rclcpp::NodeOptions & options)
        : ParamServer("lio_sam_transformFusion", options)
    {
        // The dynamic guard only expands the original 5 cm / 1 degree
        // per-sample limit when the output interval is delayed. Its hard
        // ceilings intentionally share the scan-to-map correction limits.
        continuityStepLimits.maximum_translation_m =
            scanToMapMaxCorrectionTranslation;
        continuityStepLimits.maximum_yaw_rad =
            scanToMapMaxCorrectionRotation;
        declare_parameter("odomContinuityMaximumDtSec", 0.20);
        get_parameter(
            "odomContinuityMaximumDtSec",
            continuityStepLimits.maximum_dt_sec);
        declare_parameter("odomContinuityTranslationMargin", 0.005);
        get_parameter(
            "odomContinuityTranslationMargin",
            continuityStepLimits.translation_margin_m);
        declare_parameter("odomContinuityYawMargin", 0.003490659);
        get_parameter(
            "odomContinuityYawMargin",
            continuityStepLimits.yaw_margin_rad);
        declare_parameter("odomContinuityMaximumLinearSpeed", 2.0);
        get_parameter(
            "odomContinuityMaximumLinearSpeed",
            continuityStepLimits.maximum_linear_speed_mps);
        declare_parameter("odomContinuityMaximumYawSpeed", 1.0);
        get_parameter(
            "odomContinuityMaximumYawSpeed",
            continuityStepLimits.maximum_yaw_speed_radps);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        callbackGroupImuOdometry = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLaserOdometry = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOdomOpt = rclcpp::SubscriptionOptions();
        imuOdomOpt.callback_group = callbackGroupImuOdometry;
        auto laserOdomOpt = rclcpp::SubscriptionOptions();
        laserOdomOpt.callback_group = callbackGroupLaserOdometry;
        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos,
            std::bind(&TransformFusion::lidarOdometryHandler, this,
                std::placeholders::_1), laserOdomOpt);
        subEpochOdometry =
            create_subscription<multi_map_navigation_msgs::msg::EpochOdometry>(
                "lio_sam/epoch_odometry", qos_imu,
                std::bind(&TransformFusion::epochOdometryHandler, this,
                    std::placeholders::_1), imuOdomOpt);
        subCorrectionEvent =
            create_subscription<multi_map_navigation_msgs::msg::LioCorrectionEvent>(
                "lio_sam/correction_event", rclcpp::QoS(100).reliable(),
                std::bind(&TransformFusion::correctionEventHandler, this,
                    std::placeholders::_1), laserOdomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
        pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);
        pubLocalizationFault = create_publisher<std_msgs::msg::String>(
            "lio_sam/localization_fault", rclcpp::QoS(10).reliable());
        pubMapOdomCorrection = create_publisher<
            multi_map_navigation_msgs::msg::LioMapOdomCorrection>(
                "lio_sam/map_odom_correction", rclcpp::QoS(20).reliable());
        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    static int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
               static_cast<int64_t>(stamp.nanosec);
    }

    static Eigen::Isometry3d poseToAffine(const geometry_msgs::msg::Pose & pose)
    {
        tf2::Transform transform;
        tf2::fromMsg(pose, transform);
        return tf2::transformToEigen(tf2::toMsg(transform));
    }

    static Eigen::Isometry3d odomToAffine(const nav_msgs::msg::Odometry & odom)
    {
        return poseToAffine(odom.pose.pose);
    }

    static double yawOf(const Eigen::Isometry3d & transform)
    {
        return std::atan2(transform.rotation()(1, 0), transform.rotation()(0, 0));
    }

    static const char * stateName(CorrectionState value)
    {
        switch (value)
        {
            case CorrectionState::WAITING_FOR_EPOCH: return "WAITING_FOR_EPOCH";
            case CorrectionState::STABLE: return "STABLE";
            case CorrectionState::RELEASING_SMALL: return "RELEASING_SMALL";
            case CorrectionState::REJECTED_LARGE: return "REJECTED_LARGE";
            case CorrectionState::CONTINUITY_FAULT: return "CONTINUITY_FAULT";
        }
        return "UNKNOWN";
    }

    static bool sameIdentity(
        const std::string & instanceId, uint64_t generation,
        const Anchor & anchor)
    {
        return anchor.valid && !instanceId.empty() && generation != 0 &&
               instanceId == anchor.instanceId && generation == anchor.generation;
    }

    void publishFaultOnce(const std::string & reason)
    {
        if (faultLatched)
            return;
        faultLatched = true;
        std_msgs::msg::String message;
        message.data = reason;
        pubLocalizationFault->publish(message);
        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    }

    void tryActivateAnchor(int64_t stamp)
    {
        const auto anchorIt = mappingAnchorsByStamp.find(stamp);
        const auto eventIt = scanEventsByStamp.find(stamp);
        if (anchorIt == mappingAnchorsByStamp.end() ||
            eventIt == scanEventsByStamp.end())
            return;

        const auto & event = eventIt->second;
        if (event.instance_id.empty() || event.generation == 0 ||
            event.correction_epoch == 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "TFUSION_WAITING stamp=%.9f reason=missing_identity_or_epoch",
                stamp2Sec(event.header.stamp));
            return;
        }

        activeAnchor.valid = true;
        activeAnchor.instanceId = event.instance_id;
        activeAnchor.generation = event.generation;
        activeAnchor.epoch = event.correction_epoch;
        activeAnchor.accepted = event.accepted;
        activeAnchor.stamp = stamp;
        // A rejected scan-to-map result must not re-enter through the mapping
        // odometry topic: mapOptimization may already have written the
        // attempted graph pose there.  Use the pre-optimization prediction as
        // the anchor in that case; accepted events may use the optimized
        // mapping anchor.
        activeAnchor.optimizedPose = event.accepted ?
            odomToAffine(anchorIt->second) :
            poseToAffine(event.predicted_pose_at_stamp);
        activeAnchor.correctedIncrementalPose =
            poseToAffine(event.incremental_pose_at_stamp);
        activeAnchor.translationResidual = event.accepted ?
            std::abs(event.translation_residual) : 0.0;
        activeAnchor.yawResidual = event.accepted ?
            std::abs(event.yaw_residual) : 0.0;
        state = CorrectionState::WAITING_FOR_EPOCH;

        RCLCPP_INFO(
            get_logger(),
            "TFUSION_ANCHOR stamp=%.9f epoch=%llu instance=%s generation=%llu "
            "arrival_delay=%.6f state=%s",
            stamp2Sec(event.header.stamp),
            static_cast<unsigned long long>(activeAnchor.epoch),
            activeAnchor.instanceId.c_str(),
            static_cast<unsigned long long>(activeAnchor.generation),
            event.arrival_delay_sec, stateName(state));

        mappingAnchorsByStamp.erase(anchorIt);
        scanEventsByStamp.erase(eventIt);
        while (mappingAnchorsByStamp.size() > 1000)
            mappingAnchorsByStamp.erase(mappingAnchorsByStamp.begin());
        while (scanEventsByStamp.size() > 1000)
            scanEventsByStamp.erase(scanEventsByStamp.begin());
    }

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr message)
    {
        std::lock_guard<std::mutex> lock(mtx);
        const int64_t stamp = stampToNanoseconds(message->header.stamp);
        mappingAnchorsByStamp[stamp] = *message;
        tryActivateAnchor(stamp);
    }

    void correctionEventHandler(
        const multi_map_navigation_msgs::msg::LioCorrectionEvent::SharedPtr message)
    {
        std::lock_guard<std::mutex> lock(mtx);
        RCLCPP_INFO(
            get_logger(),
            "TFUSION_CORRECTION stamp=%.9f epoch=%llu source=%u "
            "translation_residual=%.6f yaw_residual=%.6f accepted=%s reason=%s",
            stamp2Sec(message->header.stamp),
            static_cast<unsigned long long>(message->correction_epoch),
            static_cast<unsigned int>(message->source),
            message->translation_residual, message->yaw_residual,
            message->accepted ? "true" : "false", message->reason.c_str());

        if (message->source ==
            multi_map_navigation_msgs::msg::LioCorrectionEvent::SOURCE_RESET)
        {
            if (hasLastPublishedPose)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "LIO_PREINTEGRATION_RESET_RECOVERING epoch=%llu reason=%s; "
                    "holding the last legal odom pose until a matching epoch recovers",
                    static_cast<unsigned long long>(message->correction_epoch),
                    message->reason.c_str());
            }
            activeAnchor = Anchor();
            outputEpoch = 0;
            continuityOffset = Eigen::Isometry3d::Identity();
            continuityOffsetReleasePending = false;
            // Preserve the final legal odom pose across an in-process IMU
            // reset.  The first valid epoch after recovery will calculate a
            // fresh continuity offset against this pose.  A process restart
            // still starts with a new instance/generation and empty state.
            state = CorrectionState::WAITING_FOR_EPOCH;
            mappingAnchorsByStamp.clear();
            scanEventsByStamp.clear();
            return;
        }

        if (message->accepted && activeAnchor.valid &&
            message->instance_id == activeAnchor.instanceId &&
            message->generation == activeAnchor.generation &&
            message->correction_epoch == activeAnchor.epoch)
        {
            activeAnchor.translationResidual = std::max(
                activeAnchor.translationResidual,
                std::abs(message->translation_residual));
            activeAnchor.yawResidual = std::max(
                activeAnchor.yawResidual, std::abs(message->yaw_residual));
        }

        if (message->source !=
            multi_map_navigation_msgs::msg::LioCorrectionEvent::SOURCE_SCAN_TO_MAP)
            return;
        const int64_t stamp = stampToNanoseconds(message->header.stamp);
        scanEventsByStamp[stamp] = *message;
        tryActivateAnchor(stamp);
    }

    void releaseSmallOffset(const Eigen::Isometry3d & rawCurrent, double dt)
    {
        if (lio_sam::releaseContinuityOffset(rawCurrent, dt, continuityOffset))
        {
            state = CorrectionState::STABLE;
            continuityOffsetReleasePending = false;
        }
    }

    void beginEpoch(
        const multi_map_navigation_msgs::msg::EpochOdometry & message,
        const Eigen::Isometry3d & rawCurrent)
    {
        outputEpoch = message.correction_epoch;
        if (!hasLastPublishedPose)
        {
            continuityOffset = Eigen::Isometry3d::Identity();
            continuityOffsetReleasePending = false;
            state = CorrectionState::STABLE;
            return;
        }

        continuityOffset = lastPublishedPose * rawCurrent.inverse();
        // A delayed-output clamp may still have a small remainder when a new
        // scan epoch arrives. Preserve that release obligation even if this
        // epoch's scan correction itself was rejected.
        // Classify the correction itself (prediction -> optimization), not
        // the distance between a continuous odom pose and the globally
        // optimized pose.  Once frames are split, that latter distance is
        // expected to accumulate and is not an instantaneous correction.
        const double translationResidual = activeAnchor.translationResidual;
        const double yawResidual = activeAnchor.yawResidual;

        // mapOptimization and TransformFusion share the same parameterized
        // hard limits.  Keep this defensive check in the fusion path without
        // maintaining a second, potentially inconsistent set of constants.
        if (translationResidual > scanToMapMaxCorrectionTranslation ||
            yawResidual > scanToMapMaxCorrectionRotation)
        {
            state = CorrectionState::REJECTED_LARGE;
            publishFaultOnce(
                "LIO_CORRECTION_REJECTED_LARGE: epoch=" +
                std::to_string(outputEpoch) + "; translation=" +
                std::to_string(translationResidual) + "; yaw=" +
                std::to_string(yawResidual));
        }
        else if (translationResidual > kSmallCorrectionTranslation ||
                 yawResidual > kSmallCorrectionYaw)
        {
            state = CorrectionState::RELEASING_SMALL;
            RCLCPP_WARN(
                get_logger(),
                "LIO_CORRECTION_RATE_LIMITED epoch=%llu "
                "translation=%.6f yaw=%.6f",
                static_cast<unsigned long long>(outputEpoch),
                translationResidual, yawResidual);
        }
        else
        {
            state = CorrectionState::RELEASING_SMALL;
        }
    }

    void epochOdometryHandler(
        const multi_map_navigation_msgs::msg::EpochOdometry::SharedPtr message)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!sameIdentity(message->instance_id, message->generation, activeAnchor) ||
            message->correction_epoch != activeAnchor.epoch)
        {
            state = CorrectionState::WAITING_FOR_EPOCH;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "TFUSION_WAITING sample_epoch=%llu anchor_epoch=%llu "
                "sample_instance=%s anchor_instance=%s",
                static_cast<unsigned long long>(message->correction_epoch),
                static_cast<unsigned long long>(activeAnchor.epoch),
                message->instance_id.c_str(), activeAnchor.instanceId.c_str());
            return;
        }

        const Eigen::Isometry3d incrementalCurrent =
            odomToAffine(message->odometry);
        const Eigen::Isometry3d rawCurrent = activeAnchor.optimizedPose *
            activeAnchor.correctedIncrementalPose.inverse() * incrementalCurrent;
        if (outputEpoch != message->correction_epoch)
            beginEpoch(*message, rawCurrent);

        double dt = 0.01;
        const int64_t stamp = stampToNanoseconds(message->header.stamp);
        if (lastPublishedStamp > 0 && stamp > lastPublishedStamp)
            dt = static_cast<double>(stamp - lastPublishedStamp) * 1e-9;
        if (state == CorrectionState::RELEASING_SMALL &&
            (activeAnchor.accepted || continuityOffsetReleasePending))
            releaseSmallOffset(rawCurrent, dt);

        Eigen::Isometry3d candidate = continuityOffset * rawCurrent;
        double outputTranslationStep = 0.0;
        double outputPlanarStep = 0.0;
        double outputVerticalStep = 0.0;
        double outputYawStep = 0.0;
        double outputTranslationLimit =
            continuityStepLimits.minimum_translation_m;
        double outputVerticalLimit =
            continuityStepLimits.maximum_vertical_step_m;
        double outputYawLimit = continuityStepLimits.minimum_yaw_rad;
        if (hasLastPublishedPose)
        {
            const Eigen::Isometry3d delta = lastPublishedPose.inverse() * candidate;
            outputTranslationStep = delta.translation().norm();
            outputPlanarStep = delta.translation().head<2>().norm();
            outputVerticalStep = std::abs(delta.translation().z());
            outputYawStep = std::abs(yawOf(delta));
            const auto & twist = message->odometry.twist.twist;
            const double linearSpeed = std::hypot(
                twist.linear.x, twist.linear.y);
            const auto decision = lio_sam::evaluateContinuityStep(
                outputPlanarStep, outputVerticalStep, outputYawStep, dt,
                linearSpeed, twist.angular.z, continuityStepLimits);
            outputTranslationLimit = decision.translation_limit_m;
            outputVerticalLimit = decision.vertical_limit_m;
            outputYawLimit = decision.yaw_limit_rad;
            if (!decision.accepted)
            {
                state = CorrectionState::CONTINUITY_FAULT;
                publishFaultOnce(
                    "LIO_ODOM_CONTINUITY_FAULT: epoch=" +
                    std::to_string(outputEpoch) + "; step=" +
                    std::to_string(outputTranslationStep) + "; planar_step=" +
                    std::to_string(outputPlanarStep) + "; vertical_step=" +
                    std::to_string(outputVerticalStep) + "; yaw_step=" +
                    std::to_string(outputYawStep) + "; dt=" +
                    std::to_string(dt) + "; speed=" +
                    std::to_string(linearSpeed) + "; yaw_rate=" +
                    std::to_string(std::abs(twist.angular.z)) +
                    "; translation_limit=" +
                    std::to_string(outputTranslationLimit) +
                    "; vertical_limit=" +
                    std::to_string(outputVerticalLimit) +
                    "; yaw_limit=" + std::to_string(outputYawLimit));
                return;
            }

            if (outputTranslationStep >
                    continuityStepLimits.minimum_translation_m ||
                outputYawStep > continuityStepLimits.minimum_yaw_rad)
            {
                candidate = lio_sam::limitContinuityPoseStep(
                    lastPublishedPose, candidate,
                    continuityStepLimits.minimum_translation_m,
                    continuityStepLimits.minimum_yaw_rad);
                continuityOffset = candidate * rawCurrent.inverse();
                continuityOffsetReleasePending = true;
                state = CorrectionState::RELEASING_SMALL;

                const Eigen::Isometry3d limitedDelta =
                    lastPublishedPose.inverse() * candidate;
                outputTranslationStep = limitedDelta.translation().norm();
                outputPlanarStep =
                    limitedDelta.translation().head<2>().norm();
                outputVerticalStep =
                    std::abs(limitedDelta.translation().z());
                outputYawStep = std::abs(yawOf(limitedDelta));
            }
        }

        const Eigen::Isometry3d lioMapToOdomTarget =
            rawCurrent * candidate.inverse();
        const auto correctionTransform =
            tf2::eigenToTransform(lioMapToOdomTarget);
        multi_map_navigation_msgs::msg::LioMapOdomCorrection correction;
        correction.header = message->header;
        correction.header.frame_id = "lio_map";
        correction.instance_id = message->instance_id;
        correction.generation = message->generation;
        correction.correction_epoch = message->correction_epoch;
        correction.lio_map_to_odom_target = correctionTransform.transform;
        correction.translation_residual =
            activeAnchor.translationResidual;
        correction.yaw_residual = activeAnchor.yawResidual;
        correction.accepted =
            state == CorrectionState::STABLE ||
            state == CorrectionState::RELEASING_SMALL;
        if (state == CorrectionState::REJECTED_LARGE)
            correction.state = correction.STATE_LARGE_REJECTED;
        else if (state == CorrectionState::CONTINUITY_FAULT)
            correction.state = correction.STATE_CONTINUITY_FAULT;
        else
            correction.state = correction.STATE_SMALL_ACCEPTED;
        correction.reason = stateName(state);
        pubMapOdomCorrection->publish(correction);

        auto transform = tf2::eigenToTransform(candidate);
        nav_msgs::msg::Odometry fusedOdometry = message->odometry;
        fusedOdometry.header.frame_id = odometryFrame;
        fusedOdometry.child_frame_id = lidarFrame;
        fusedOdometry.pose.pose.position.x = transform.transform.translation.x;
        fusedOdometry.pose.pose.position.y = transform.transform.translation.y;
        fusedOdometry.pose.pose.position.z = transform.transform.translation.z;
        fusedOdometry.pose.pose.orientation = transform.transform.rotation;
        pubImuOdometry->publish(fusedOdometry);

        tf2::Stamped<tf2::Transform> currentTransform;
        tf2::convert(transform, currentTransform);
        if (lidarFrame != baselinkFrame)
        {
            try
            {
                tf2::fromMsg(tfBuffer->lookupTransform(
                    lidarFrame, baselinkFrame, rclcpp::Time(0)), lidar2Baselink);
                currentTransform = tf2::Stamped<tf2::Transform>(
                    currentTransform * lidar2Baselink,
                    tf2_ros::fromMsg(message->header.stamp), odometryFrame);
            }
            catch (const tf2::TransformException & error)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000, "%s", error.what());
                return;
            }
        }
        geometry_msgs::msg::TransformStamped tfMessage;
        tf2::convert(currentTransform, tfMessage);
        tfMessage.header.stamp = message->header.stamp;
        tfMessage.header.frame_id = odometryFrame;
        tfMessage.child_frame_id = baselinkFrame;
        tfBroadcaster->sendTransform(tfMessage);

        lastPublishedPose = candidate;
        lastPublishedStamp = stamp;
        hasLastPublishedPose = true;
        RCLCPP_DEBUG(
            get_logger(),
            "TFUSION_OUTPUT stamp=%.9f epoch=%llu state=%s "
            "translation_step=%.6f translation_limit=%.6f "
            "vertical_step=%.6f yaw_step=%.6f yaw_limit=%.6f dt=%.6f",
            stamp2Sec(message->header.stamp),
            static_cast<unsigned long long>(outputEpoch), stateName(state),
            outputTranslationStep, outputTranslationLimit,
            outputVerticalStep, outputYawStep, outputYawLimit, dt);

        const double imuTime = stamp2Sec(message->header.stamp);
        if (imuTime - lastPathTime > 0.1)
        {
            lastPathTime = imuTime;
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header = fusedOdometry.header;
            poseStamped.pose = fusedOdometry.pose.pose;
            imuPath.poses.push_back(poseStamped);
            while (!imuPath.poses.empty() &&
                stamp2Sec(imuPath.poses.front().header.stamp) < imuTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header = fusedOdometry.header;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex pendingMutex_;
    std::condition_variable pendingCv_;
    std::deque<sensor_msgs::msg::Imu> pendingImu_;
    std::deque<nav_msgs::msg::Odometry> pendingOdometry_;
    std::thread workerThread_;
    bool workerRunning_ = true;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Subscription<yunle_msgs::msg::VehicleStatus>::SharedPtr
        subVehicleStatus;
    rclcpp::Subscription<multi_map_navigation_msgs::msg::LioCorrectionEvent>::SharedPtr
        subRawCorrectionEvent;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<multi_map_navigation_msgs::msg::EpochOdometry>::SharedPtr
        pubEpochOdometry;
    rclcpp::Publisher<multi_map_navigation_msgs::msg::LioCorrectionEvent>::SharedPtr
        pubCorrectionEvent;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubReady;
    rclcpp::Publisher<multi_map_navigation_msgs::msg::LioReadiness>::SharedPtr
        pubReadiness;
    std::string readinessInstanceId_;
    uint64_t readinessGeneration_ = 0;
    std::atomic<uint64_t> correctionEpoch_{1};
    std::mutex correctionDiagnosticMutex_;
    std::multimap<int64_t, multi_map_navigation_msgs::msg::LioCorrectionEvent>
        pendingCorrectionEvents_;
    std::map<int64_t, uint64_t> epochByCorrectionStamp_;
    std::map<int64_t, geometry_msgs::msg::Pose>
        correctedIncrementalPoseByStamp_;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::noiseModel::Diagonal::shared_ptr zeroVelocityNoise;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastCorrectionTime_ = -1;
    double lastInputImuTime_ = -1;
    double minCorrectionTimeAfterReset_ = -1;
    size_t successfulOptimizations_ = 0;
    bool ready_ = false;
    double maxContinuousImuGap_ = 0.2;
    bool useVehicleZeroVelocityUpdate_ = true;
    double vehicleStationarySpeedThresholdKmh_ = 0.10;
    double vehicleStatusTimeoutSec_ = 0.25;
    int vehicleStationaryMinSamples_ = 5;
    double zeroVelocitySigma_ = 0.05;
    lio_sam::VehicleStationaryHistory vehicleStationaryHistory_;

    std::atomic_bool fatalOptimizationFailure_{false};

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imu_preintegration", options)
    {
        declare_parameter("imuMaxGapSec", 0.2);
        get_parameter("imuMaxGapSec", maxContinuousImuGap_);
        if (maxContinuousImuGap_ < 0.0)
            maxContinuousImuGap_ = 0.0;
        declare_parameter("useVehicleZeroVelocityUpdate", true);
        get_parameter(
            "useVehicleZeroVelocityUpdate", useVehicleZeroVelocityUpdate_);
        declare_parameter("vehicleStationarySpeedThresholdKmh", 0.10);
        get_parameter(
            "vehicleStationarySpeedThresholdKmh",
            vehicleStationarySpeedThresholdKmh_);
        declare_parameter("vehicleStatusTimeoutSec", 0.25);
        get_parameter("vehicleStatusTimeoutSec", vehicleStatusTimeoutSec_);
        declare_parameter("vehicleStationaryMinSamples", 5);
        get_parameter(
            "vehicleStationaryMinSamples", vehicleStationaryMinSamples_);
        declare_parameter("zeroVelocitySigma", 0.05);
        get_parameter("zeroVelocitySigma", zeroVelocitySigma_);
        vehicleStationarySpeedThresholdKmh_ = std::max(
            0.0, vehicleStationarySpeedThresholdKmh_);
        vehicleStatusTimeoutSec_ = std::max(0.05, vehicleStatusTimeoutSec_);
        vehicleStationaryMinSamples_ = std::max(
            1, vehicleStationaryMinSamples_);
        zeroVelocitySigma_ = std::max(1e-3, zeroVelocitySigma_);

        callbackGroupImu = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1),
            imuOpt); // 订阅imu话题
        subOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry_incremental", qos,
            std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1),
            odomOpt); // 订阅激光里程计话题
        subVehicleStatus =
            create_subscription<yunle_msgs::msg::VehicleStatus>(
                "/vehicle_status", rclcpp::QoS(20),
                std::bind(
                    &IMUPreintegration::vehicleStatusHandler, this,
                    std::placeholders::_1));
        subRawCorrectionEvent =
            create_subscription<multi_map_navigation_msgs::msg::LioCorrectionEvent>(
                "lio_sam/mapping/correction_event", rclcpp::QoS(100).reliable(),
                std::bind(
                    &IMUPreintegration::rawCorrectionEventHandler, this,
                    std::placeholders::_1));

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic+"_incremental", qos_imu); // 发布Imu里程计
        pubEpochOdometry =
            create_publisher<multi_map_navigation_msgs::msg::EpochOdometry>(
                "lio_sam/epoch_odometry", qos_imu);
        pubCorrectionEvent =
            create_publisher<multi_map_navigation_msgs::msg::LioCorrectionEvent>(
                "lio_sam/correction_event", rclcpp::QoS(100).reliable());
        auto legacyReadyQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        pubReady = create_publisher<std_msgs::msg::Bool>(
            "lio_sam/imu_preintegration_ready", legacyReadyQos);
        auto readinessQos = rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliable()
                                .transient_local();
        rclcpp::PublisherOptions readinessPublisherOptions;
        // This node enables intra-process communication for its high-rate LIO
        // topics.  ROS 2 Humble only permits volatile publishers on that path,
        // while readiness must be transient-local so ProcessManager can receive
        // the latest state after discovery.  Keep only this low-rate publisher
        // on DDS to satisfy both requirements.
        readinessPublisherOptions.use_intra_process_comm =
            rclcpp::IntraProcessSetting::Disable;
        pubReadiness =
            create_publisher<multi_map_navigation_msgs::msg::LioReadiness>(
                "lio_sam/imu_preintegration_readiness", readinessQos,
                readinessPublisherOptions);

        const char * instanceId = std::getenv("LIO_SAM_INSTANCE_ID");
        const char * generation = std::getenv("LIO_SAM_GENERATION");
        if (instanceId != nullptr)
            readinessInstanceId_ = instanceId;
        if (generation != nullptr)
        {
            try
            {
                readinessGeneration_ = std::stoull(generation);
            }
            catch (const std::exception & error)
            {
                RCLCPP_ERROR(
                    get_logger(), "Invalid LIO_SAM_GENERATION '%s': %s",
                    generation, error.what());
            }
        }
        if (readinessInstanceId_.empty() || readinessGeneration_ == 0)
        {
            RCLCPP_WARN(
                get_logger(),
                "LIO readiness identity is unavailable; ProcessManager will "
                "not accept this manually started instance");
        }

        // 预积分的噪声协方差
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        // 噪声先验
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        zeroVelocityNoise = gtsam::noiseModel::Isotropic::Sigma(
            3, zeroVelocitySigma_);
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        // gtsam中的imu预积分器，Imu里程计
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        // gtsam中的imu预积分器，用于因子图优化
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization

        prevPose_ = gtsam::Pose3();
        prevVel_ = gtsam::Vector3::Zero();
        prevBias_ = prior_imu_bias;
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        publishReady(false);
        workerThread_ = std::thread(&IMUPreintegration::processingLoop, this);
    }

    void vehicleStatusHandler(
        const yunle_msgs::msg::VehicleStatus::SharedPtr message)
    {
        const double speedKmh = static_cast<double>(message->cur_speed);
        vehicleStationaryHistory_.add(
            stampToNanoseconds(message->header.stamp), speedKmh);
    }

    lio_sam::VehicleStationaryDecision vehicleStationaryDecision(
        int64_t correction_stamp_ns) const
    {
        if (!useVehicleZeroVelocityUpdate_) {
            return {};
        }
        return vehicleStationaryHistory_.evaluate(
            correction_stamp_ns,
            vehicleStatusTimeoutSec_,
            vehicleStationaryMinSamples_,
            vehicleStationarySpeedThresholdKmh_);
    }

    void logVehicleStationaryDecision(
        int64_t correction_stamp_ns,
        const lio_sam::VehicleStationaryDecision & decision) const
    {
        RCLCPP_DEBUG(
            get_logger(),
            "LIO_ZERO_VELOCITY_DECISION correction=%ld applied=%s "
            "sample=%ld speed_kmh=%.3f reason=%s",
            static_cast<long>(correction_stamp_ns),
            decision.stationary ? "true" : "false",
            static_cast<long>(decision.sample_stamp_ns),
            decision.speed_kmh,
            decision.reason);
    }

    ~IMUPreintegration() override
    {
        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            workerRunning_ = false;
        }
        pendingCv_.notify_all();
        if (workerThread_.joinable())
            workerThread_.join();
        delete imuIntegratorOpt_;
        delete imuIntegratorImu_;
    }

    void publishReady(bool ready)
    {
        if (ready_ == ready && ready)
            return;
        ready_ = ready;
        std_msgs::msg::Bool message;
        message.data = ready;
        pubReady->publish(message);

        multi_map_navigation_msgs::msg::LioReadiness readiness;
        readiness.header.stamp = now();
        readiness.instance_id = readinessInstanceId_;
        readiness.generation = readinessGeneration_;
        readiness.ready = ready;
        pubReadiness->publish(readiness);
    }

    static int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL +
               static_cast<int64_t>(stamp.nanosec);
    }

    void publishCorrectionEvent(
        multi_map_navigation_msgs::msg::LioCorrectionEvent event,
        uint64_t epoch)
    {
        event.instance_id = readinessInstanceId_;
        event.generation = readinessGeneration_;
        event.correction_epoch = epoch;
        event.arrival_delay_sec =
            (now() - rclcpp::Time(event.header.stamp)).seconds();
        pubCorrectionEvent->publish(event);
    }

    static geometry_msgs::msg::Pose gtsamPoseToMessage(
        const gtsam::Pose3 & pose)
    {
        geometry_msgs::msg::Pose message;
        message.position.x = pose.translation().x();
        message.position.y = pose.translation().y();
        message.position.z = pose.translation().z();
        const auto quaternion = pose.rotation().toQuaternion();
        message.orientation.x = quaternion.x();
        message.orientation.y = quaternion.y();
        message.orientation.z = quaternion.z();
        message.orientation.w = quaternion.w();
        return message;
    }

    void attachCorrectedIncrementalPose(
        multi_map_navigation_msgs::msg::LioCorrectionEvent & event,
        int64_t stamp)
    {
        const auto pose = correctedIncrementalPoseByStamp_.find(stamp);
        if (pose != correctedIncrementalPoseByStamp_.end())
            event.incremental_pose_at_stamp = pose->second;
    }

    void rawCorrectionEventHandler(
        const multi_map_navigation_msgs::msg::LioCorrectionEvent::SharedPtr message)
    {
        if ((!message->instance_id.empty() &&
             message->instance_id != readinessInstanceId_) ||
            (message->generation != 0 &&
             message->generation != readinessGeneration_))
        {
            RCLCPP_WARN(
                get_logger(),
                "Dropping correction diagnostic from another LIO instance: "
                "instance=%s generation=%llu",
                message->instance_id.c_str(),
                static_cast<unsigned long long>(message->generation));
            return;
        }

        const int64_t stamp = stampToNanoseconds(message->header.stamp);
        std::lock_guard<std::mutex> lock(correctionDiagnosticMutex_);
        // Every mapping incremental odometry sample is still a correction input
        // to IMU preintegration when scan-to-map was skipped. Keep those events
        // paired with the successful repropagation epoch; rejected LOOP/GPS
        // candidates do not create a new epoch and can be published immediately.
        if (!message->accepted &&
            message->source !=
                multi_map_navigation_msgs::msg::LioCorrectionEvent::SOURCE_SCAN_TO_MAP)
        {
            publishCorrectionEvent(*message, correctionEpoch_.load());
            return;
        }

        const auto epoch = epochByCorrectionStamp_.find(stamp);
        if (epoch != epochByCorrectionStamp_.end())
        {
            auto event = *message;
            attachCorrectedIncrementalPose(event, stamp);
            publishCorrectionEvent(event, epoch->second);
            return;
        }
        pendingCorrectionEvents_.emplace(stamp, *message);
    }

    void finalizeCorrectionEpoch(
        const builtin_interfaces::msg::Time & correctionStamp,
        bool incrementEpoch,
        const gtsam::Pose3 & correctedIncrementalPose)
    {
        const int64_t stamp = stampToNanoseconds(correctionStamp);
        std::lock_guard<std::mutex> lock(correctionDiagnosticMutex_);
        if (incrementEpoch)
            correctionEpoch_.fetch_add(1);
        const uint64_t epoch = correctionEpoch_.load();
        epochByCorrectionStamp_[stamp] = epoch;
        correctedIncrementalPoseByStamp_[stamp] =
            gtsamPoseToMessage(correctedIncrementalPose);

        const auto range = pendingCorrectionEvents_.equal_range(stamp);
        for (auto it = range.first; it != range.second; ++it)
        {
            auto event = it->second;
            attachCorrectedIncrementalPose(event, stamp);
            publishCorrectionEvent(event, epoch);
        }
        pendingCorrectionEvents_.erase(range.first, range.second);

        while (epochByCorrectionStamp_.size() > 1000)
            epochByCorrectionStamp_.erase(epochByCorrectionStamp_.begin());
        while (correctedIncrementalPoseByStamp_.size() > 1000)
            correctedIncrementalPoseByStamp_.erase(
                correctedIncrementalPoseByStamp_.begin());

        const int64_t retentionCutoff = stamp - 10000000000LL;
        while (!pendingCorrectionEvents_.empty() &&
               pendingCorrectionEvents_.begin()->first < retentionCutoff)
        {
            auto stale = pendingCorrectionEvents_.begin();
            stale->second.accepted = false;
            stale->second.reason +=
                "; diagnostic arrived without a matching successful correction";
            publishCorrectionEvent(stale->second, epoch);
            pendingCorrectionEvents_.erase(stale);
        }
    }

    void publishResetEvent(const std::string & reason)
    {
        multi_map_navigation_msgs::msg::LioCorrectionEvent event;
        event.header.stamp = now();
        event.header.frame_id = mappingFrame;
        event.source =
            multi_map_navigation_msgs::msg::LioCorrectionEvent::SOURCE_RESET;
        event.predicted_pose_at_stamp.orientation.w = 1.0;
        event.optimized_pose_at_stamp.orientation.w = 1.0;
        event.incremental_pose_at_stamp.orientation.w = 1.0;
        event.accepted = true;
        event.reason = reason;
        publishCorrectionEvent(event, correctionEpoch_.load());
    }

    void processingLoop()
    {
        while (true)
        {
            sensor_msgs::msg::Imu imu;
            nav_msgs::msg::Odometry odometry;
            bool processImuSample = false;
            bool processOdometrySample = false;

            {
                std::unique_lock<std::mutex> lock(pendingMutex_);
                pendingCv_.wait(lock, [this]() {
                    return !workerRunning_ || !pendingImu_.empty() ||
                           !pendingOdometry_.empty();
                });

                if (!workerRunning_)
                    break;

                if (!pendingImu_.empty())
                {
                    imu = std::move(pendingImu_.front());
                    pendingImu_.pop_front();
                    processImuSample = true;
                }
                else if (!pendingOdometry_.empty())
                {
                    const double correctionTime =
                        stamp2Sec(pendingOdometry_.front().header.stamp);
                    if (lastInputImuTime_ < 0 || lastInputImuTime_ < correctionTime)
                    {
                        pendingCv_.wait(lock, [this]() {
                            return !workerRunning_ || !pendingImu_.empty();
                        });
                        continue;
                    }
                    odometry = std::move(pendingOdometry_.front());
                    pendingOdometry_.pop_front();
                    processOdometrySample = true;
                }
            }

            if (processImuSample)
                processImu(imu);
            else if (processOdometrySample)
                processOdometry(odometry);
        }
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    bool isValidImuDt(double dt) const
    {
        return isPositiveImuDt(dt) &&
               (maxContinuousImuGap_ <= 0.0 || dt <= maxContinuousImuGap_);
    }

    void resetPreintegrationState(
        bool clearImuQueues, double minCorrectionTimeAfterReset = -1)
    {
        publishReady(false);
        const uint64_t resetEpoch = correctionEpoch_.fetch_add(1) + 1;
        {
            std::lock_guard<std::mutex> lock(correctionDiagnosticMutex_);
            for (auto & entry : pendingCorrectionEvents_)
            {
                entry.second.accepted = false;
                entry.second.reason += "; discarded by IMU preintegration reset";
                publishCorrectionEvent(entry.second, resetEpoch);
            }
            pendingCorrectionEvents_.clear();
            epochByCorrectionStamp_.clear();
            correctedIncrementalPoseByStamp_.clear();
        }
        publishResetEvent(
            "IMU preintegration reset; clear_imu_queues=" +
            std::string(clearImuQueues ? "true" : "false") +
            "; min_correction_time=" +
            std::to_string(minCorrectionTimeAfterReset));
        resetOptimization();

        prevPose_ = gtsam::Pose3();
        prevVel_ = gtsam::Vector3::Zero();
        prevBias_ = gtsam::imuBias::ConstantBias();
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);

        lastImuT_imu = -1;
        lastImuT_opt = -1;
        lastCorrectionTime_ = -1;
        lastInputImuTime_ = -1;
        key = 1;
        doneFirstOpt = false;
        systemInitialized = false;
        successfulOptimizations_ = 0;
        minCorrectionTimeAfterReset_ = minCorrectionTimeAfterReset;

        if (clearImuQueues)
        {
            imuQueOpt.clear();
            imuQueImu.clear();

            size_t discardedCorrections = 0;
            {
                std::lock_guard<std::mutex> lock(pendingMutex_);
                discardedCorrections = pendingOdometry_.size();
                pendingOdometry_.clear();
            }
            if (discardedCorrections > 0)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Discarded %zu queued lidar corrections while resetting IMU "
                    "preintegration",
                    discardedCorrections);
            }
        }
    }

    void requestFatalPreintegrationShutdown(
        const char * reason,
        const char * stage,
        double correctionTime,
        size_t integratedSamples,
        const std::string & nearbyVariable = "unavailable",
        gtsam::Key nearbyKey = 0)
    {
        const double integratedTime = imuIntegratorOpt_->deltaTij();

        RCLCPP_FATAL(
            get_logger(),
            "Fatal IMU preintegration failure; shutting down process: "
            "stage=%s key=%d nearby_variable=%s(%llu) correction_time=%.9f "
            "last_correction_time=%.9f integrated_samples=%zu integrated_dt=%.9f "
            "imu_opt_queue=%zu imu_reprop_queue=%zu last_imu_opt=%.9f last_imu_realtime=%.9f; %s",
            stage,
            key,
            nearbyVariable.c_str(),
            static_cast<unsigned long long>(nearbyKey),
            correctionTime,
            lastCorrectionTime_,
            integratedSamples,
            integratedTime,
            imuQueOpt.size(),
            imuQueImu.size(),
            lastImuT_opt,
            lastImuT_imu,
            reason);

        // ISAM2 may be partially updated when update() throws. Discard every
        // optimizer, graph, integration and timing state before leaving.
        resetPreintegrationState(true);
        fatalOptimizationFailure_.store(true);
        rclcpp::shutdown();
    }

    void requestFatalOptimizationShutdown(
        const gtsam::IndeterminantLinearSystemException & error,
        const char * stage,
        double correctionTime,
        size_t integratedSamples)
    {
        requestFatalPreintegrationShutdown(
            error.what(), stage, correctionTime, integratedSamples,
            gtsam::DefaultKeyFormatter(error.nearbyVariable()),
            error.nearbyVariable());
    }

    bool hasFatalOptimizationFailure() const
    {
        return fatalOptimizationFailure_.load();
    }
    // 接收到激光里程计数据后自动执行回调
    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (fatalOptimizationFailure_.load())
            return;

        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingOdometry_.push_back(*odomMsg);
        }
        pendingCv_.notify_one();
    }

    void processOdometry(const nav_msgs::msg::Odometry & odomMessage)
    {
        const auto * odomMsg = &odomMessage;

        const int64_t correctionStampNs = stampToNanoseconds(odomMsg->header.stamp);
        double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);
        if (minCorrectionTimeAfterReset_ >= 0 &&
            currentCorrectionTime < minCorrectionTimeAfterReset_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping lidar correction from before the IMU recovery epoch: "
                "time=%.9f recovery_epoch=%.9f",
                currentCorrectionTime, minCorrectionTimeAfterReset_);
            return;
        }
        if (!std::isfinite(currentCorrectionTime) ||
            (lastCorrectionTime_ >= 0 && currentCorrectionTime <= lastCorrectionTime_))
        {
            RCLCPP_WARN(
                get_logger(),
                "Dropping non-monotonic lidar correction: time=%.9f last_time=%.9f",
                currentCorrectionTime, lastCorrectionTime_);
            return;
        }

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        double p_x = odomMsg->pose.pose.position.x;
        double p_y = odomMsg->pose.pose.position.y;
        double p_z = odomMsg->pose.pose.position.z;
        double r_x = odomMsg->pose.pose.orientation.x;
        double r_y = odomMsg->pose.pose.orientation.y;
        double r_z = odomMsg->pose.pose.orientation.z;
        double r_w = odomMsg->pose.pose.orientation.w;
        double quaternionNorm = std::sqrt(r_x * r_x + r_y * r_y + r_z * r_z + r_w * r_w);
        if (!std::isfinite(p_x) || !std::isfinite(p_y) || !std::isfinite(p_z) ||
            !std::isfinite(quaternionNorm) || quaternionNorm < 1e-6)
        {
            requestFatalPreintegrationShutdown(
                "invalid lidar correction pose or quaternion",
                "validate_lidar_correction_pose",
                currentCorrectionTime, 0);
            return;
        }
        r_x /= quaternionNorm;
        r_y /= quaternionNorm;
        r_z /= quaternionNorm;
        r_w /= quaternionNorm;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 系统初始化，仅首次执行 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (stamp2Sec(imuQueOpt.front().header.stamp) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = stamp2Sec(imuQueOpt.front().header.stamp);
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            const auto stationaryDecision = vehicleStationaryDecision(correctionStampNs);
            logVehicleStationaryDecision(correctionStampNs, stationaryDecision);
            const auto initialVelocityNoise =
                stationaryDecision.stationary ?
                    zeroVelocityNoise : priorVelNoise;
            gtsam::PriorFactor<gtsam::Vector3> priorVel(
                V(0), prevVel_, initialVelocityNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // Keep the prediction state consistent with the initial priors.
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevStateOdom = prevState_;
            prevBiasOdom = prevBias_;
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            try
            {
                optimizer.update(graphFactors, graphValues);
            }
            catch (const gtsam::IndeterminantLinearSystemException & error)
            {
                requestFatalOptimizationShutdown(
                    error, "initial_prior_update", currentCorrectionTime, 0);
                return;
            }
            catch (const std::exception & error)
            {
                requestFatalPreintegrationShutdown(
                    error.what(), "initial_prior_update", currentCorrectionTime, 0);
                return;
            }
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            lastCorrectionTime_ = currentCorrectionTime;
            systemInitialized = true;
            finalizeCorrectionEpoch(
                odomMsg->header.stamp, false,
                prevStateOdom.pose().compose(imu2Lidar));
            return;
        }


        size_t integratedSamples = 0;
        const char * optimizationStage = "incremental_update";

        try
        {
            // reset graph for speed
            if (key == 100)
            {
                optimizationStage = "marginal_covariance";
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizationStage = "graph_reset_update";
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

                key = 1;
            }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = stamp2Sec(thisImu->header.stamp);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? kDefaultImuDt : (imuTime - lastImuT_opt);
                if (!isFiniteImuMeasurement(*thisImu) || !isValidImuDt(dt))
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "IMU timeline broke during optimization; resetting preintegration: "
                        "imu_time=%.9f previous_time=%.9f dt=%.9f finite=%s",
                        imuTime, lastImuT_opt, dt,
                        isFiniteImuMeasurement(*thisImu) ? "true" : "false");
                    resetPreintegrationState(true, imuTime);
                    return;
                }
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
                ++integratedSamples;
            }
            else
                break;
        }
        const double integratedTime = imuIntegratorOpt_->deltaTij();
        if (!std::isfinite(integratedTime))
        {
            requestFatalPreintegrationShutdown(
                "non-finite accumulated IMU integration time",
                "validate_integrated_time",
                currentCorrectionTime, integratedSamples);
            return;
        }
        if (integratedSamples == 0 && integratedTime >= 0.0 && integratedTime <= kMinImuDt)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Waiting for IMU samples before optimization: correction_time=%.9f "
                "last_imu_time=%.9f samples=%zu integrated_dt=%.9f",
                currentCorrectionTime, lastImuT_opt, integratedSamples, integratedTime);
            return;
        }
        if (integratedSamples == 0 || integratedTime <= kMinImuDt)
        {
            requestFatalPreintegrationShutdown(
                "IMU sample count and accumulated integration time are inconsistent",
                "validate_integrated_samples",
                currentCorrectionTime, integratedSamples);
            return;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(std::sqrt(integratedTime) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        const auto stationaryDecision = vehicleStationaryDecision(correctionStampNs);
        logVehicleStationaryDecision(correctionStampNs, stationaryDecision);
        const bool zeroVelocityApplied = stationaryDecision.stationary;
        if (zeroVelocityApplied)
        {
            graphFactors.add(gtsam::PriorFactor<gtsam::Vector3>(
                V(key), gtsam::Vector3::Zero(), zeroVelocityNoise));
        }
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizationStage = "incremental_update";
        optimizer.update(graphFactors, graphValues);
        optimizationStage = "incremental_relinearize";
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        optimizationStage = "calculate_estimate";
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        if (zeroVelocityApplied)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LIO_ZERO_VELOCITY_UPDATE speed_kmh=%.3f "
                "velocity_before_constraint=%.6f velocity_after=%.6f "
                "accel_bias_norm=%.6f",
                stationaryDecision.speed_kmh, propState_.v().norm(),
                prevVel_.norm(), prevBias_.accelerometer().norm());
        }
        }
        catch (const gtsam::IndeterminantLinearSystemException & error)
        {
            requestFatalOptimizationShutdown(
                error, optimizationStage, currentCorrectionTime, integratedSamples);
            return;
        }
        catch (const std::exception & error)
        {
            requestFatalPreintegrationShutdown(
                error.what(), optimizationStage, currentCorrectionTime, integratedSamples);
            return;
        }
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevPose_, prevVel_, prevBias_))
        {
            const std::string reason =
                "optimized state failed finite/range checks: velocity_norm=" +
                std::to_string(prevVel_.norm()) +
                " accel_bias_norm=" + std::to_string(prevBias_.accelerometer().norm()) +
                " gyro_bias_norm=" + std::to_string(prevBias_.gyroscope().norm());
            requestFatalPreintegrationShutdown(
                reason.c_str(), "validate_optimized_state",
                currentCorrectionTime, integratedSamples);
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && stamp2Sec(imuQueImu.front().header.stamp) < currentCorrectionTime - delta_t)
        {
            lastImuQT = stamp2Sec(imuQueImu.front().header.stamp);
            imuQueImu.pop_front();
        }
        try
        {
            // Always reset the realtime integrator to the newly optimized state.
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            lastImuT_imu = -1;
            // repropogate
            if (!imuQueImu.empty())
            {
                // integrate imu message from the beginning of this optimization
                for (int i = 0; i < (int)imuQueImu.size(); ++i)
                {
                    sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                    double imuTime = stamp2Sec(thisImu->header.stamp);
                    double dt = (lastImuQT < 0) ? kDefaultImuDt : (imuTime - lastImuQT);
                    if (!isFiniteImuMeasurement(*thisImu) || !isValidImuDt(dt))
                    {
                        RCLCPP_ERROR(
                            get_logger(),
                            "IMU timeline broke during repropagation; resetting preintegration: "
                            "imu_time=%.9f previous_time=%.9f dt=%.9f finite=%s",
                            imuTime, lastImuQT, dt,
                            isFiniteImuMeasurement(*thisImu) ? "true" : "false");
                        resetPreintegrationState(true, imuTime);
                        return;
                    }

                    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                            gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                    lastImuQT = imuTime;
                    lastImuT_imu = imuTime;
                }
            }
        }
        catch (const gtsam::IndeterminantLinearSystemException & error)
        {
            requestFatalOptimizationShutdown(
                error, "repropagate_realtime_imu", currentCorrectionTime, integratedSamples);
            return;
        }
        catch (const std::exception & error)
        {
            requestFatalPreintegrationShutdown(
                error.what(), "repropagate_realtime_imu",
                currentCorrectionTime, integratedSamples);
            return;
        }

        lastCorrectionTime_ = currentCorrectionTime;
        // A new epoch becomes visible only after optimization and realtime IMU
        // repropagation have both completed successfully.
        finalizeCorrectionEpoch(
            odomMsg->header.stamp, true,
            prevStateOdom.pose().compose(imu2Lidar));
        ++key;
        doneFirstOpt = true;
        ++successfulOptimizations_;
        if (successfulOptimizations_ >= kReadyOptimizationCount)
            publishReady(true);
    }

    bool failureDetection(
        const gtsam::Pose3 & poseCur,
        const gtsam::Vector3 & velCur,
        const gtsam::imuBias::ConstantBias & biasCur)
    {
        if (!poseCur.matrix().allFinite())
        {
            RCLCPP_WARN(get_logger(), "Non-finite pose, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (!vel.allFinite())
        {
            RCLCPP_WARN(get_logger(), "Non-finite velocity, reset IMU-preintegration!");
            return true;
        }
        if (vel.norm() > 30)
        {
            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (!ba.allFinite() || !bg.allFinite())
        {
            RCLCPP_WARN(get_logger(), "Non-finite bias, reset IMU-preintegration!");
            return true;
        }
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    // 接收到imu数据时自动执行回调
    /*
        1.imu预积分累加，预估当前位姿
        2.坐标系转换，发布imu里程计数据
        注意：没有使用imu数据的orientation(位姿)字段，只用imu进行预积分计算运动增量(相对位姿)
    */
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw)
    {
        if (fatalOptimizationFailure_.load())
            return;

        {
            std::lock_guard<std::mutex> lock(pendingMutex_);
            pendingImu_.push_back(*imu_raw);
        }
        pendingCv_.notify_one();
    }

    void processImu(const sensor_msgs::msg::Imu & imuMessage)
    {
        auto imu_raw = std::make_shared<sensor_msgs::msg::Imu>(imuMessage);

        // 修改源码的部分
        // imu_raw->header.frame_id="imu_link";
        // std::cout << "x_before: " << imu_raw->linear_acceleration.x 
        //      << " y_before: " << imu_raw->linear_acceleration.y
        //      << " z_before: " << imu_raw->linear_acceleration.z 
        //      << std::endl; 

        // /livox/imu输出单位是g，需要乘以9.8转换为m/s²
        // imu_raw->linear_acceleration.x*=9.80511;
        // imu_raw->linear_acceleration.y*=9.80511;
        // imu_raw->linear_acceleration.z*=9.80511;

        // std::cout << " x_after: " << imu_raw->linear_acceleration.x 
        //      << " y_after: " << imu_raw->linear_acceleration.y
        //      << " z_after: " << imu_raw->linear_acceleration.z 
        //      << std::endl; 

        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw); // 将imu数据从imu坐标系转为雷达坐标系(6轴没有orientation字段)
        const double inputImuTime = stamp2Sec(thisImu.header.stamp);
        if (!std::isfinite(inputImuTime) || !isFiniteImuMeasurement(thisImu))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Dropping invalid IMU sample: time=%.9f finite=%s",
                inputImuTime, isFiniteImuMeasurement(thisImu) ? "true" : "false");
            return;
        }

        if (lastInputImuTime_ >= 0 && inputImuTime <= lastInputImuTime_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Dropping non-monotonic IMU sample: time=%.9f last_time=%.9f",
                inputImuTime, lastInputImuTime_);
            return;
        }

        if (lastInputImuTime_ >= 0 && maxContinuousImuGap_ > 0.0 &&
            inputImuTime - lastInputImuTime_ > maxContinuousImuGap_)
        {
            RCLCPP_ERROR(
                get_logger(),
                "IMU gap %.3f s exceeds %.3f s; resetting preintegration without "
                "stopping LIO-SAM",
                inputImuTime - lastInputImuTime_, maxContinuousImuGap_);
            resetPreintegrationState(true, inputImuTime);
        }
        lastInputImuTime_ = inputImuTime;
        
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        // 等待激光里程计提供初始偏差估计
        if (doneFirstOpt == false)
            return;
        
        // 计算两帧imu时间差
        double imuTime = inputImuTime;
        double dt = (lastImuT_imu < 0) ? kDefaultImuDt : (imuTime - lastImuT_imu);
        if (!isValidImuDt(dt))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Invalid realtime IMU dt %.9f; resetting preintegration without "
                "stopping LIO-SAM",
                dt);
            resetPreintegrationState(true, imuTime);
            return;
        }
        lastImuT_imu = imuTime;

        gtsam::NavState currentState;
        try
        {
            // 将当前Imu数据累计到预积分器中 integrate this single imu message
            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                    gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

            // 预测位姿 predict odometry
            currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
            if (!currentState.pose().matrix().allFinite() || !currentState.v().allFinite())
            {
                requestFatalPreintegrationShutdown(
                    "realtime IMU prediction produced a non-finite state",
                    "validate_realtime_prediction", lastCorrectionTime_, 0);
                return;
            }
        }
        catch (const gtsam::IndeterminantLinearSystemException & error)
        {
            requestFatalOptimizationShutdown(
                error, "predict_realtime_imu", lastCorrectionTime_, 0);
            return;
        }
        catch (const std::exception & error)
        {
            requestFatalPreintegrationShutdown(
                error.what(), "predict_realtime_imu", lastCorrectionTime_, 0);
            return;
        }

        // publish odometry
        auto odometry = nav_msgs::msg::Odometry();
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = mappingFrame;
        odometry.child_frame_id = "lio_map_imu";

        // 坐标变换至Lidar坐标系 transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        multi_map_navigation_msgs::msg::EpochOdometry epochOdometry;
        epochOdometry.header = odometry.header;
        epochOdometry.instance_id = readinessInstanceId_;
        epochOdometry.generation = readinessGeneration_;
        epochOdometry.correction_epoch = correctionEpoch_.load();
        epochOdometry.odometry = odometry;
        pubEpochOdometry->publish(epochOdometry);
        pubImuOdometry->publish(odometry);
    }
};


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    e.add_node(ImuP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> IMU Preintegration Started.\033[0m");

    e.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    return ImuP->hasFatalOptimizationFailure() ? EXIT_FAILURE : EXIT_SUCCESS;
}
