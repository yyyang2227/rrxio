/*
 * Copyright (c) 2021 Christopher Doer
 * Copyright (c) 2014, Autonomous Systems Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef ROVIO_ROVIONODE_HPP_
#define ROVIO_ROVIONODE_HPP_

#include <algorithm>
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <map>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cmath>
#include <Eigen/Eigenvalues>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <radar_ego_velocity_estimator/radar_body_velocity_estimator.h>

#include <rovio/SrvResetToPose.h>
#include "rovio/CoordinateTransform/RovioOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutput.hpp"
#include "rovio/CoordinateTransform/FeatureOutputReadable.hpp"
#include "rovio/CoordinateTransform/YprOutput.hpp"
#include "rovio/CoordinateTransform/LandmarkOutput.hpp"

#include "rrxio/RRxIOFilter.hpp"

namespace rovio
{
/** \brief Class, defining the Rovio Node
 *
 *  @tparam FILTER  - \ref rovio::RovioFilter
 */
template <typename FILTER>
// TODO REMOVE
// typedef rovio::RovioFilter<rovio::FilterState<15, 4, 6, 1, 0>> FILTER;

class RovioNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Filter Stuff
  typedef FILTER mtFilter;
  std::shared_ptr<mtFilter> mpFilter_;
  typedef typename mtFilter::mtFilterState mtFilterState;
  typedef typename mtFilterState::mtState mtState;
  typedef typename mtFilter::mtPrediction::mtMeas mtPredictionMeas;
  mtPredictionMeas predictionMeas_;
  typedef typename std::tuple_element<0, typename mtFilter::mtUpdates>::type mtImgUpdate;
  typedef typename mtImgUpdate::mtMeas mtImgMeas;
  mtImgMeas imgUpdateMeas_;
  mtImgUpdate* mpImgUpdate_;
  typedef typename std::tuple_element<1, typename mtFilter::mtUpdates>::type mtPoseUpdate;
  typedef typename mtPoseUpdate::mtMeas mtPoseMeas;
  mtPoseMeas poseUpdateMeas_;
  mtPoseUpdate* mpPoseUpdate_;
  typedef typename std::tuple_element<2, typename mtFilter::mtUpdates>::type mtVelocityUpdate;
  typedef typename mtVelocityUpdate::mtMeas mtVelocityMeas;
  mtVelocityMeas velocityUpdateMeas_;
  mtVelocityUpdate* mpVelocityUpdate_;

  struct FilterInitializationState
  {
    FilterInitializationState() : WrWM_(V3D::Zero()), state_(State::WaitForInitUsingAccel) {}

    enum class State
    {
      // Initialize the filter using accelerometer measurement on the next
      // opportunity.
      WaitForInitUsingAccel,
      // Initialize the filter using an external pose on the next opportunity.
      WaitForInitExternalPose,
      // The filter is initialized.
      Initialized
    } state_;

    // Buffer to hold the initial pose that should be set during initialization
    // with the state WaitForInitExternalPose.
    V3D WrWM_;
    QPD qMW_;

    explicit operator bool() const { return isInitialized(); }

    bool isInitialized() const { return (state_ == State::Initialized); }
  };
  FilterInitializationState init_state_;

  bool forceOdometryPublishing_;
  bool forcePoseWithCovariancePublishing_;
  bool forceTransformPublishing_;
  bool forceExtrinsicsPublishing_;
  bool forceImuBiasPublishing_;
  bool forcePclPublishing_;
  bool forceMarkersPublishing_;
  bool forcePatchPublishing_;
  bool gotFirstMessages_;
  std::mutex m_filter_;

  double timeshift_cam_imu_ = 0;

  // radar body velocity
  std::vector<sensor_msgs::Imu> most_recent_imus_;
  sensor_msgs::PointCloud2 most_recent_radar_scan_;
  Eigen::Vector3d w_b_radar_trigger_;
  std::shared_ptr<reve::RadarBodyVelocityEstimator> radar_body_estimator_;
  uint64_t radar_scan_callback_count_ = 0;

  struct RadarCovRecalibConfig
  {
    std::string cov_mode = "base";
    double fixed_scale   = 2.0;

    double w_cond      = 0.7;
    double w_inlier    = 1.0;
    double w_sparse    = 0.6;
    double n_targets_ref = 40.0;
    double k_alpha       = 1.5;
    double alpha_r_max   = 5.0;
    double sigma_min2    = 1.0e-4;
    double max_r_cond_ref = 1000.0;
  };
  RadarCovRecalibConfig radar_cov_recalib_cfg_;

  struct RadarSkConfig
  {
    bool enable      = false;
    bool apply_gate_enable = true;
    double tau_obs   = 8.0;
    double c_obs     = 1.5;
    double s_max     = 3.0;
    double eps_lambda = 1.0e-6;
    double lambda3_apply_max = 2.0;
    double d_r_apply_min = 0.55;
    double n_targets_apply_max = 40.0;
    double obs_trace_apply_max = 1.0e12;
    double obs_aniso_apply_min = 0.0;
  };
  RadarSkConfig radar_sk_cfg_;

  struct RadarCovDiagFields
  {
    double d_r = std::numeric_limits<double>::quiet_NaN();
    double alpha_r = std::numeric_limits<double>::quiet_NaN();
    int s_k_valid = 0;
    double lambda1_obs = std::numeric_limits<double>::quiet_NaN();
    double lambda2_obs = std::numeric_limits<double>::quiet_NaN();
    double lambda3_obs = std::numeric_limits<double>::quiet_NaN();
    double obs_trace = std::numeric_limits<double>::quiet_NaN();
    double obs_aniso = std::numeric_limits<double>::quiet_NaN();
    double s1 = std::numeric_limits<double>::quiet_NaN();
    double s2 = std::numeric_limits<double>::quiet_NaN();
    double s3 = std::numeric_limits<double>::quiet_NaN();
    double trace_R_after_alpha = std::numeric_limits<double>::quiet_NaN();
    double trace_R_after_sk = std::numeric_limits<double>::quiet_NaN();
    int s_k_applied = 0;
    std::string s_k_skip_reason = "not_applicable";
    int sk_gate_lambda3_pass = 0;
    int sk_gate_d_r_pass = 0;
    int sk_gate_ntargets_pass = 0;
    int sk_gate_obs_trace_pass = 0;
    int sk_gate_obs_aniso_pass = 0;
  };

  static constexpr double kChi2_3_95_ = 7.8147279;

  // DVC diagnostic logging (W3-W4 minimal closure)
  bool dvc_diag_enabled_ = false;
  std::string dvc_diag_output_dir_;
  std::string dvc_run_id_;
  std::ofstream dvc_diag_stream_;
  uint64_t dvc_diag_rows_ = 0;

  enum class SchedulerMode
  {
    Legacy,
    EventStage1,
    EventStage2
  };

  enum class EventType
  {
    Imu,
    Image0,
    Image1,
    GroundtruthPose,
    GroundtruthOdometry,
    Velocity,
    RadarTrigger,
    RadarScan,
    Reset,
    ResetToPose
  };

  struct SchedulerConfig
  {
    std::string mode             = "legacy";
    int max_events               = 2048;
    double watermark_margin_s    = 0.002;
    double watermark_max_wait_s  = 0.200;
    double radar_imu_window_s    = 0.020;
    bool enable_diag             = false;
    bool diag_log_all_events     = false;
    std::string diag_output_dir;
    bool imu_fast_path = true;
  };

  struct PerfConfig
  {
    std::string mode = "normal";
    int pub_decimation = 1;
    int tf_decimation = 1;
    int diag_flush_every_n = 1;
  };

  struct EventQueueStats
  {
    uint64_t enqueued = 0;
    uint64_t processed = 0;
    uint64_t dropped_total = 0;
    uint64_t watermark_block_count = 0;
    uint64_t radar_starved_count = 0;
    std::map<EventType, uint64_t> dropped_by_type;
  };

  struct SensorEvent
  {
    EventType type = EventType::Imu;
    double timestamp = 0.0;
    uint64_t seq = 0;
    double enqueue_wall_time = 0.0;
    double loader_backpressure_wait_ms = std::numeric_limits<double>::quiet_NaN();
    std::string loader_backpressure_reason = "none";
    uint64_t loader_backpressure_queue_depth = 0;
    uint64_t loader_backpressure_imu_depth = 0;
    int loader_backpressure_timeout = 0;

    sensor_msgs::Imu::ConstPtr imu_msg;
    sensor_msgs::ImageConstPtr img_msg;
    geometry_msgs::TransformStamped::ConstPtr gt_msg;
    nav_msgs::Odometry::ConstPtr gt_odom_msg;
    geometry_msgs::TwistStamped::ConstPtr vel_msg;
    std_msgs::HeaderConstPtr trigger_msg;
    sensor_msgs::PointCloud2ConstPtr radar_scan_msg;
    V3D reset_WrWM = V3D::Zero();
    QPD reset_qMW;
  };

  SchedulerConfig scheduler_cfg_;
  PerfConfig perf_cfg_;
  SchedulerMode scheduler_mode_ = SchedulerMode::Legacy;
  std::thread estimator_worker_;
  std::mutex scheduler_mutex_;
  std::condition_variable scheduler_cv_;
  std::deque<SensorEvent> event_queue_;
  std::deque<sensor_msgs::Imu::ConstPtr> imu_fast_queue_;
  std::atomic<bool> worker_running_{ false };
  std::atomic<bool> worker_processing_{ false };
  std::atomic<bool> scheduler_drain_requested_{ false };
  std::thread::id worker_thread_id_;
  uint64_t event_seq_ = 0;
  double latest_imu_time_ = -std::numeric_limits<double>::infinity();
  double oldest_pending_radar_timestamp_ = std::numeric_limits<double>::infinity();
  EventQueueStats event_queue_stats_;
  std::map<uint64_t, double> watermark_block_start_sec_;
  std::deque<sensor_msgs::Imu> imu_history_;
  std::ofstream dvc_sched_diag_stream_;
  std::string dvc_sched_diag_path_;
  uint64_t dvc_sched_diag_rows_ = 0;
  std::mutex image_sync_mutex_;
  double active_radar_enqueue_wall_time_ = std::numeric_limits<double>::quiet_NaN();
  double last_radar_enqueue_to_commit_ms_ = std::numeric_limits<double>::quiet_NaN();
  int last_radar_starved_ = 0;
  std::string last_drop_type_ = "none";
  double current_mfilter_lock_wait_us_ = 0.0;
  double current_mfilter_lock_hold_us_ = 0.0;
  double last_reve_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_backend_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_publish_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_diag_io_ms_ = std::numeric_limits<double>::quiet_NaN();
  bool loader_backpressure_pending_ = false;
  double pending_loader_backpressure_wait_ms_ = std::numeric_limits<double>::quiet_NaN();
  std::string pending_loader_backpressure_reason_ = "none";
  uint64_t pending_loader_backpressure_queue_depth_ = 0;
  uint64_t pending_loader_backpressure_imu_depth_ = 0;
  int pending_loader_backpressure_timeout_ = 0;
  uint64_t publish_cycle_count_ = 0;

  // Nodes, Subscriber, Publishers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber subImu_;
  ros::Subscriber subImg0_;
  ros::Subscriber subImg1_;
  ros::Subscriber subGroundtruth_;
  ros::Subscriber subGroundtruthOdometry_;
  ros::Subscriber subVelocity_;
  ros::Subscriber subRadarTrigger_;
  ros::Subscriber subRadar_;
  ros::ServiceServer srvResetFilter_;
  ros::ServiceServer srvResetToPoseFilter_;
  ros::Publisher pubOdometry_;
  ros::Publisher pubTransform_;
  ros::Publisher pubPoseWithCovStamped_;
  ros::Publisher pub_T_J_W_transform;
  tf::TransformBroadcaster tb_;
  ros::Publisher pubPcl_;     /**<Publisher: Ros point cloud, visualizing the landmarks.*/
  ros::Publisher pubPatch_;   /**<Publisher: Patch data.*/
  ros::Publisher pubMarkers_; /**<Publisher: Ros line marker, indicating the depth uncertainty of a landmark.*/
  ros::Publisher pubExtrinsics_[mtState::nCam_];
  ros::Publisher pubImuBias_;
  image_transport::Publisher image_publisher_;

  // Ros Messages
  geometry_msgs::TransformStamped transformMsg_;
  geometry_msgs::TransformStamped T_J_W_Msg_;
  nav_msgs::Odometry odometryMsg_;
  geometry_msgs::PoseWithCovarianceStamped estimatedPoseWithCovarianceStampedMsg_;
  geometry_msgs::PoseWithCovarianceStamped extrinsicsMsg_[mtState::nCam_];
  sensor_msgs::PointCloud2 pclMsg_;
  sensor_msgs::PointCloud2 patchMsg_;
  visualization_msgs::Marker markerMsg_;
  sensor_msgs::Imu imuBiasMsg_;
  int msgSeq_;

  // Rovio outputs and coordinate transformations
  typedef StandardOutput mtOutput;
  mtOutput cameraOutput_;
  MXD cameraOutputCov_;
  mtOutput imuOutput_;
  MXD imuOutputCov_;
  CameraOutputCT<mtState> cameraOutputCT_;
  ImuOutputCT<mtState> imuOutputCT_;
  rovio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
  rovio::LandmarkOutputImuCT<mtState> landmarkOutputImuCT_;
  rovio::FeatureOutput featureOutput_;
  rovio::LandmarkOutput landmarkOutput_;
  MXD featureOutputCov_;
  MXD landmarkOutputCov_;
  rovio::FeatureOutputReadableCT featureOutputReadableCT_;
  rovio::FeatureOutputReadable featureOutputReadable_;
  MXD featureOutputReadableCov_;

  // ROS names for output tf frames.
  std::string map_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string imu_frame_;

  /** \brief Constructor
   */
  RovioNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::shared_ptr<mtFilter> mpFilter) :
    nh_(nh),
    nh_private_(nh_private),
    mpFilter_(mpFilter),
    transformFeatureOutputCT_(&mpFilter->multiCamera_),
    landmarkOutputImuCT_(&mpFilter->multiCamera_),
    cameraOutputCov_((int)(mtOutput::D_), (int)(mtOutput::D_)),
    featureOutputCov_((int)(FeatureOutput::D_), (int)(FeatureOutput::D_)),
    landmarkOutputCov_(3, 3),
    featureOutputReadableCov_((int)(FeatureOutputReadable::D_), (int)(FeatureOutputReadable::D_)),
    w_b_radar_trigger_(0, 0, 0)
  {
#ifndef NDEBUG
    ROS_WARN("====================== Debug Mode ======================");
#endif
    mpImgUpdate_  = &std::get<0>(mpFilter_->mUpdates_);
    mpPoseUpdate_ = &std::get<1>(mpFilter_->mUpdates_);
    mpVelocityUpdate_ = &std::get<2>(mpFilter_->mUpdates_);
    forceOdometryPublishing_           = false;
    forcePoseWithCovariancePublishing_ = false;
    forceTransformPublishing_          = false;
    forceExtrinsicsPublishing_         = false;
    forceImuBiasPublishing_            = false;
    forcePclPublishing_                = false;
    forceMarkersPublishing_            = false;
    forcePatchPublishing_              = false;
    gotFirstMessages_                  = false;

    // radar body velocity estimation
    radar_body_estimator_.reset(new reve::RadarBodyVelocityEstimator(nh_private, true));

    // Subscribe topics
    std::string topic_imu = "/sensor_platform/imu/imu";
    nh_private_.param("topic_imu", topic_imu, topic_imu);
    std::string topic_cam = "/sensor_platform/camera/img";
    nh_private_.param("topic_cam", topic_cam, topic_cam);
    std::string topic_vel = "/refiner_ego_velocity_node/twist";
    nh_private_.param("topic_vel", topic_vel, topic_vel);
    std::string topic_radar_trigger = "";

    nh_private.param("topic_radar_trigger", topic_radar_trigger, topic_radar_trigger);
    std::string topic_radar_scan = "";
    nh_private.param("topic_radar_scan", topic_radar_scan, topic_radar_scan);
    double timeshift_cam_imu = 0.0;
    nh_private.param("timeshift_cam_imu", timeshift_cam_imu, timeshift_cam_imu);
    timeshift_cam_imu_ = timeshift_cam_imu;
    nh_private_.param("dvc_diag_enabled", dvc_diag_enabled_, dvc_diag_enabled_);
    nh_private_.param("dvc_diag_output_dir", dvc_diag_output_dir_, dvc_diag_output_dir_);
    nh_private_.param("dvc_run_id", dvc_run_id_, dvc_run_id_);
    nh_private_.param("dvc_rrxio/scheduler/mode", scheduler_cfg_.mode, scheduler_cfg_.mode);
    nh_private_.param("dvc_rrxio/scheduler/max_events", scheduler_cfg_.max_events, scheduler_cfg_.max_events);
    nh_private_.param(
        "dvc_rrxio/scheduler/watermark_margin_s", scheduler_cfg_.watermark_margin_s, scheduler_cfg_.watermark_margin_s);
    nh_private_.param("dvc_rrxio/scheduler/watermark_max_wait_s",
                      scheduler_cfg_.watermark_max_wait_s,
                      scheduler_cfg_.watermark_max_wait_s);
    nh_private_.param(
        "dvc_rrxio/scheduler/radar_imu_window_s", scheduler_cfg_.radar_imu_window_s, scheduler_cfg_.radar_imu_window_s);
    nh_private_.param("dvc_rrxio/scheduler/enable_diag", scheduler_cfg_.enable_diag, scheduler_cfg_.enable_diag);
    nh_private_.param("dvc_rrxio/scheduler/diag_log_all_events",
                      scheduler_cfg_.diag_log_all_events,
                      scheduler_cfg_.diag_log_all_events);
    nh_private_.param(
        "dvc_rrxio/scheduler/diag_output_dir", scheduler_cfg_.diag_output_dir, scheduler_cfg_.diag_output_dir);
    nh_private_.param("dvc_rrxio/scheduler/imu_fast_path", scheduler_cfg_.imu_fast_path, scheduler_cfg_.imu_fast_path);
    nh_private_.param("dvc_rrxio/perf_mode", perf_cfg_.mode, perf_cfg_.mode);
    nh_private_.param("dvc_rrxio/perf/pub_decimation", perf_cfg_.pub_decimation, perf_cfg_.pub_decimation);
    nh_private_.param("dvc_rrxio/perf/tf_decimation", perf_cfg_.tf_decimation, perf_cfg_.tf_decimation);
    nh_private_.param(
        "dvc_rrxio/perf/diag_flush_every_n", perf_cfg_.diag_flush_every_n, perf_cfg_.diag_flush_every_n);
    nh_private_.param("dvc_rrxio/cov_mode", radar_cov_recalib_cfg_.cov_mode, radar_cov_recalib_cfg_.cov_mode);
    nh_private_.param("dvc_rrxio/fixed_scale", radar_cov_recalib_cfg_.fixed_scale, radar_cov_recalib_cfg_.fixed_scale);
    nh_private_.param("dvc_rrxio/alpha_r/w_cond", radar_cov_recalib_cfg_.w_cond, radar_cov_recalib_cfg_.w_cond);
    nh_private_.param("dvc_rrxio/alpha_r/w_inlier", radar_cov_recalib_cfg_.w_inlier, radar_cov_recalib_cfg_.w_inlier);
    nh_private_.param("dvc_rrxio/alpha_r/w_sparse", radar_cov_recalib_cfg_.w_sparse, radar_cov_recalib_cfg_.w_sparse);
    nh_private_.param(
        "dvc_rrxio/alpha_r/n_targets_ref", radar_cov_recalib_cfg_.n_targets_ref, radar_cov_recalib_cfg_.n_targets_ref);
    nh_private_.param("dvc_rrxio/alpha_r/k_alpha", radar_cov_recalib_cfg_.k_alpha, radar_cov_recalib_cfg_.k_alpha);
    nh_private_.param(
        "dvc_rrxio/alpha_r/alpha_r_max", radar_cov_recalib_cfg_.alpha_r_max, radar_cov_recalib_cfg_.alpha_r_max);
    nh_private_.param("dvc_rrxio/alpha_r/sigma_min2", radar_cov_recalib_cfg_.sigma_min2, radar_cov_recalib_cfg_.sigma_min2);
    nh_private_.param("dvc_rrxio/s_k/enable", radar_sk_cfg_.enable, radar_sk_cfg_.enable);
    nh_private_.param(
        "dvc_rrxio/s_k/apply_gate_enable", radar_sk_cfg_.apply_gate_enable, radar_sk_cfg_.apply_gate_enable);
    nh_private_.param("dvc_rrxio/s_k/tau_obs", radar_sk_cfg_.tau_obs, radar_sk_cfg_.tau_obs);
    nh_private_.param("dvc_rrxio/s_k/c_obs", radar_sk_cfg_.c_obs, radar_sk_cfg_.c_obs);
    nh_private_.param("dvc_rrxio/s_k/s_max", radar_sk_cfg_.s_max, radar_sk_cfg_.s_max);
    nh_private_.param("dvc_rrxio/s_k/eps_lambda", radar_sk_cfg_.eps_lambda, radar_sk_cfg_.eps_lambda);
    nh_private_.param(
        "dvc_rrxio/s_k/lambda3_apply_max", radar_sk_cfg_.lambda3_apply_max, radar_sk_cfg_.lambda3_apply_max);
    nh_private_.param("dvc_rrxio/s_k/d_r_apply_min", radar_sk_cfg_.d_r_apply_min, radar_sk_cfg_.d_r_apply_min);
    nh_private_.param(
        "dvc_rrxio/s_k/n_targets_apply_max", radar_sk_cfg_.n_targets_apply_max, radar_sk_cfg_.n_targets_apply_max);
    nh_private_.param(
        "dvc_rrxio/s_k/obs_trace_apply_max", radar_sk_cfg_.obs_trace_apply_max, radar_sk_cfg_.obs_trace_apply_max);
    nh_private_.param(
        "dvc_rrxio/s_k/obs_aniso_apply_min", radar_sk_cfg_.obs_aniso_apply_min, radar_sk_cfg_.obs_aniso_apply_min);
    nh_private_.param("max_r_cond", radar_cov_recalib_cfg_.max_r_cond_ref, radar_cov_recalib_cfg_.max_r_cond_ref);

    std::transform(radar_cov_recalib_cfg_.cov_mode.begin(),
                   radar_cov_recalib_cfg_.cov_mode.end(),
                   radar_cov_recalib_cfg_.cov_mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(scheduler_cfg_.mode.begin(),
                   scheduler_cfg_.mode.end(),
                   scheduler_cfg_.mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(perf_cfg_.mode.begin(),
                   perf_cfg_.mode.end(),
                   perf_cfg_.mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (scheduler_cfg_.mode == "legacy")
      scheduler_mode_ = SchedulerMode::Legacy;
    else if (scheduler_cfg_.mode == "event_stage1")
      scheduler_mode_ = SchedulerMode::EventStage1;
    else if (scheduler_cfg_.mode == "event_stage2")
      scheduler_mode_ = SchedulerMode::EventStage2;
    else
      throw std::runtime_error("Invalid dvc_rrxio/scheduler/mode. Supported: legacy, event_stage1, event_stage2.");

    if (radar_cov_recalib_cfg_.cov_mode != "base" && radar_cov_recalib_cfg_.cov_mode != "fixed" &&
        radar_cov_recalib_cfg_.cov_mode != "alpha_r" && radar_cov_recalib_cfg_.cov_mode != "alpha_r_sk")
    {
      throw std::runtime_error("Invalid dvc_rrxio/cov_mode. Supported modes: base, fixed, alpha_r, alpha_r_sk.");
    }
    if (radar_cov_recalib_cfg_.fixed_scale <= 0.0)
    {
      throw std::runtime_error("dvc_rrxio/fixed_scale must be > 0.");
    }
    if (radar_cov_recalib_cfg_.sigma_min2 <= 0.0)
    {
      throw std::runtime_error("dvc_rrxio/alpha_r/sigma_min2 must be > 0.");
    }
    if (radar_cov_recalib_cfg_.n_targets_ref <= 0.0)
    {
      throw std::runtime_error("dvc_rrxio/alpha_r/n_targets_ref must be > 0.");
    }
    if (radar_cov_recalib_cfg_.k_alpha < 0.0)
    {
      throw std::runtime_error("dvc_rrxio/alpha_r/k_alpha must be >= 0.");
    }
    if (radar_cov_recalib_cfg_.alpha_r_max < 1.0)
    {
      throw std::runtime_error("dvc_rrxio/alpha_r/alpha_r_max must be >= 1.");
    }
    if (radar_sk_cfg_.tau_obs <= 0.0)
    {
      throw std::runtime_error("dvc_rrxio/s_k/tau_obs must be > 0.");
    }
    if (radar_sk_cfg_.c_obs < 0.0)
    {
      throw std::runtime_error("dvc_rrxio/s_k/c_obs must be >= 0.");
    }
    if (radar_sk_cfg_.s_max < 1.0)
    {
      throw std::runtime_error("dvc_rrxio/s_k/s_max must be >= 1.");
    }
    if (radar_sk_cfg_.eps_lambda <= 0.0)
    {
      throw std::runtime_error("dvc_rrxio/s_k/eps_lambda must be > 0.");
    }
    if (!(std::isfinite(radar_sk_cfg_.lambda3_apply_max) && radar_sk_cfg_.lambda3_apply_max > 0.0))
    {
      throw std::runtime_error("dvc_rrxio/s_k/lambda3_apply_max must be finite and > 0.");
    }
    if (!(std::isfinite(radar_sk_cfg_.d_r_apply_min) && radar_sk_cfg_.d_r_apply_min >= 0.0))
    {
      throw std::runtime_error("dvc_rrxio/s_k/d_r_apply_min must be finite and >= 0.");
    }
    if (!(std::isfinite(radar_sk_cfg_.n_targets_apply_max) && radar_sk_cfg_.n_targets_apply_max > 0.0))
    {
      throw std::runtime_error("dvc_rrxio/s_k/n_targets_apply_max must be finite and > 0.");
    }
    if (!(std::isfinite(radar_sk_cfg_.obs_trace_apply_max) && radar_sk_cfg_.obs_trace_apply_max > 0.0))
    {
      throw std::runtime_error("dvc_rrxio/s_k/obs_trace_apply_max must be finite and > 0.");
    }
    if (!(std::isfinite(radar_sk_cfg_.obs_aniso_apply_min) && radar_sk_cfg_.obs_aniso_apply_min >= 0.0))
    {
      throw std::runtime_error("dvc_rrxio/s_k/obs_aniso_apply_min must be finite and >= 0.");
    }
    if (radar_cov_recalib_cfg_.max_r_cond_ref <= 1.0)
    {
      throw std::runtime_error("max_r_cond must be > 1 for alpha_R condition-number normalization.");
    }
    if (scheduler_cfg_.max_events <= 0)
      throw std::runtime_error("dvc_rrxio/scheduler/max_events must be > 0.");
    if (scheduler_cfg_.watermark_margin_s < 0.0)
      throw std::runtime_error("dvc_rrxio/scheduler/watermark_margin_s must be >= 0.");
    if (scheduler_cfg_.watermark_max_wait_s <= 0.0)
      throw std::runtime_error("dvc_rrxio/scheduler/watermark_max_wait_s must be > 0.");
    if (scheduler_cfg_.radar_imu_window_s <= 0.0)
      throw std::runtime_error("dvc_rrxio/scheduler/radar_imu_window_s must be > 0.");
    if (perf_cfg_.mode != "normal" && perf_cfg_.mode != "perf")
      throw std::runtime_error("dvc_rrxio/perf_mode must be normal or perf.");
    if (perf_cfg_.pub_decimation <= 0)
      throw std::runtime_error("dvc_rrxio/perf/pub_decimation must be > 0.");
    if (perf_cfg_.tf_decimation <= 0)
      throw std::runtime_error("dvc_rrxio/perf/tf_decimation must be > 0.");
    if (perf_cfg_.diag_flush_every_n <= 0)
      throw std::runtime_error("dvc_rrxio/perf/diag_flush_every_n must be > 0.");
    if (scheduler_mode_ != SchedulerMode::EventStage2)
      scheduler_cfg_.imu_fast_path = false;
    initDvcDiagLogging();

    subImu_                 = nh_.subscribe(topic_imu, 2000, &RovioNode::imuCallback, this);
    subImg0_                = nh_.subscribe(topic_cam, 10, &RovioNode::imgCallback0, this);
    subImg1_                = nh_.subscribe("cam1/image_raw", 1000, &RovioNode::imgCallback1, this);
    subGroundtruth_         = nh_.subscribe("pose", 1000, &RovioNode::groundtruthCallback, this);
    subGroundtruthOdometry_ = nh_.subscribe("odometry", 1000, &RovioNode::groundtruthOdometryCallback, this);
    subVelocity_            = nh_.subscribe(topic_vel, 2, &RovioNode::velocityCallback, this);
    subRadarTrigger_        = nh_.subscribe(topic_radar_trigger, 10, &RovioNode::radarTriggerCallback, this);
    subRadar_               = nh_.subscribe(topic_radar_scan, 10, &RovioNode::radarScanCallback, this);

    // Initialize ROS service servers.
    srvResetFilter_ = nh_private_.advertiseService("rovio/reset", &RovioNode::resetServiceCallback, this);
    srvResetToPoseFilter_ =
        nh_private_.advertiseService("rovio/reset_to_pose", &RovioNode::resetToPoseServiceCallback, this);

    // Advertise topics
    pubTransform_ = nh_private_.advertise<geometry_msgs::TransformStamped>("rovio/transform", 1);
    pubOdometry_  = nh_private_.advertise<nav_msgs::Odometry>("rovio/odometry", 1);
    pubPoseWithCovStamped_ =
        nh_private_.advertise<geometry_msgs::PoseWithCovarianceStamped>("rovio/pose_with_covariance_stamped", 1);
    pubPcl_     = nh_private_.advertise<sensor_msgs::PointCloud2>("rovio/pcl", 1);
    pubPatch_   = nh_private_.advertise<sensor_msgs::PointCloud2>("rovio/patch", 1);
    pubMarkers_ = nh_private_.advertise<visualization_msgs::Marker>("rovio/markers", 1);

    image_transport::ImageTransport it(nh_private_);
    image_publisher_ = it.advertise("rovio/tracker", 1);

    pub_T_J_W_transform = nh_private_.advertise<geometry_msgs::TransformStamped>("rovio/T_G_W", 1);
    for (int camID = 0; camID < mtState::nCam_; camID++)
    {
      pubExtrinsics_[camID] = nh_private_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "rovio/extrinsics" + std::to_string(camID), 1);
    }
    pubImuBias_ = nh_private_.advertise<sensor_msgs::Imu>("rovio/imu_biases", 1);

    // Handle coordinate frame naming
    map_frame_    = "/map";
    world_frame_  = "/rovio";
    camera_frame_ = "/camera";
    imu_frame_    = "/imu";
    nh_private_.param("map_frame", map_frame_, map_frame_);
    nh_private_.param("world_frame", world_frame_, world_frame_);
    nh_private_.param("camera_frame", camera_frame_, camera_frame_);
    nh_private_.param("imu_frame", imu_frame_, imu_frame_);

    // Initialize messages
    transformMsg_.header.frame_id = world_frame_;
    transformMsg_.child_frame_id  = imu_frame_;

    T_J_W_Msg_.child_frame_id  = world_frame_;
    T_J_W_Msg_.header.frame_id = map_frame_;

    odometryMsg_.header.frame_id = world_frame_;
    odometryMsg_.child_frame_id  = imu_frame_;
    msgSeq_                      = 1;
    for (int camID = 0; camID < mtState::nCam_; camID++)
    {
      extrinsicsMsg_[camID].header.frame_id = imu_frame_;
    }
    imuBiasMsg_.header.frame_id = world_frame_;
    imuBiasMsg_.orientation.x   = 0;
    imuBiasMsg_.orientation.y   = 0;
    imuBiasMsg_.orientation.z   = 0;
    imuBiasMsg_.orientation.w   = 1;
    for (int i = 0; i < 9; i++)
    {
      imuBiasMsg_.orientation_covariance[i] = 0.0;
    }

    // PointCloud message.
    pclMsg_.header.frame_id         = imu_frame_;
    pclMsg_.height                  = 1;               // Unordered point cloud.
    pclMsg_.width                   = mtState::nMax_;  // Number of features/points.
    const int nFieldsPcl            = 18;
    std::string namePcl[nFieldsPcl] = {"id",
                                       "camId",
                                       "rgb",
                                       "status",
                                       "x",
                                       "y",
                                       "z",
                                       "b_x",
                                       "b_y",
                                       "b_z",
                                       "d",
                                       "c_00",
                                       "c_01",
                                       "c_02",
                                       "c_11",
                                       "c_12",
                                       "c_22",
                                       "c_d"};
    int sizePcl[nFieldsPcl]         = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
    int countPcl[nFieldsPcl]        = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    int datatypePcl[nFieldsPcl]     = {sensor_msgs::PointField::INT32,
                                   sensor_msgs::PointField::INT32,
                                   sensor_msgs::PointField::UINT32,
                                   sensor_msgs::PointField::UINT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32,
                                   sensor_msgs::PointField::FLOAT32};
    pclMsg_.fields.resize(nFieldsPcl);
    int byteCounter = 0;
    for (int i = 0; i < nFieldsPcl; i++)
    {
      pclMsg_.fields[i].name     = namePcl[i];
      pclMsg_.fields[i].offset   = byteCounter;
      pclMsg_.fields[i].count    = countPcl[i];
      pclMsg_.fields[i].datatype = datatypePcl[i];
      byteCounter += sizePcl[i] * countPcl[i];
    }
    pclMsg_.point_step = byteCounter;
    pclMsg_.row_step   = pclMsg_.point_step * pclMsg_.width;
    pclMsg_.data.resize(pclMsg_.row_step * pclMsg_.height);
    pclMsg_.is_dense = false;

    // PointCloud message.
    patchMsg_.header.frame_id           = "";
    patchMsg_.height                    = 1;               // Unordered point cloud.
    patchMsg_.width                     = mtState::nMax_;  // Number of features/points.
    const int nFieldsPatch              = 5;
    std::string namePatch[nFieldsPatch] = {"id", "patch", "dx", "dy", "error"};
    int sizePatch[nFieldsPatch]         = {4, 4, 4, 4, 4};
    int countPatch[nFieldsPatch]        = {1,
                                    mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
                                    mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
                                    mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_,
                                    mtState::nLevels_ * mtState::patchSize_ * mtState::patchSize_};
    int datatypePatch[nFieldsPatch]     = {sensor_msgs::PointField::INT32,
                                       sensor_msgs::PointField::FLOAT32,
                                       sensor_msgs::PointField::FLOAT32,
                                       sensor_msgs::PointField::FLOAT32,
                                       sensor_msgs::PointField::FLOAT32};
    patchMsg_.fields.resize(nFieldsPatch);
    byteCounter = 0;
    for (int i = 0; i < nFieldsPatch; i++)
    {
      patchMsg_.fields[i].name     = namePatch[i];
      patchMsg_.fields[i].offset   = byteCounter;
      patchMsg_.fields[i].count    = countPatch[i];
      patchMsg_.fields[i].datatype = datatypePatch[i];
      byteCounter += sizePatch[i] * countPatch[i];
    }
    patchMsg_.point_step = byteCounter;
    patchMsg_.row_step   = patchMsg_.point_step * patchMsg_.width;
    patchMsg_.data.resize(patchMsg_.row_step * patchMsg_.height);
    patchMsg_.is_dense = false;

    // Marker message (vizualization of uncertainty)
    markerMsg_.header.frame_id    = imu_frame_;
    markerMsg_.id                 = 0;
    markerMsg_.type               = visualization_msgs::Marker::LINE_LIST;
    markerMsg_.action             = visualization_msgs::Marker::ADD;
    markerMsg_.pose.position.x    = 0;
    markerMsg_.pose.position.y    = 0;
    markerMsg_.pose.position.z    = 0;
    markerMsg_.pose.orientation.x = 0.0;
    markerMsg_.pose.orientation.y = 0.0;
    markerMsg_.pose.orientation.z = 0.0;
    markerMsg_.pose.orientation.w = 1.0;
    markerMsg_.scale.x            = 0.04;  // Line width.
    markerMsg_.color.a            = 1.0;
    markerMsg_.color.r            = 0.0;
    markerMsg_.color.g            = 1.0;
    markerMsg_.color.b            = 0.0;

    if (scheduler_cfg_.enable_diag)
    {
      initSchedulerDiagLogging();
    }

    if (scheduler_mode_ != SchedulerMode::Legacy)
    {
      worker_running_.store(true);
      estimator_worker_ = std::thread(&RovioNode::estimatorWorkerLoop, this);
      ROS_INFO_STREAM("[scheduler] enabled mode=" << scheduler_cfg_.mode << ", max_events=" << scheduler_cfg_.max_events
                                                  << ", watermark_margin_s=" << scheduler_cfg_.watermark_margin_s
                                                  << ", watermark_max_wait_s=" << scheduler_cfg_.watermark_max_wait_s
                                                  << ", imu_fast_path=" << scheduler_cfg_.imu_fast_path);
    }
  }

  /** \brief Destructor
   */
  virtual ~RovioNode()
  {
    if (scheduler_mode_ != SchedulerMode::Legacy)
    {
      worker_running_.store(false);
      scheduler_cv_.notify_all();
      if (estimator_worker_.joinable())
      {
        estimator_worker_.join();
      }
    }
    if (dvc_diag_stream_.is_open())
      dvc_diag_stream_.close();
    if (dvc_sched_diag_stream_.is_open())
    {
      dvc_sched_diag_stream_.flush();
      dvc_sched_diag_stream_.close();
      sanitizeSchedulerDiagFileTail();
    }
  }

  static bool ensureDirectory(const std::string& directory)
  {
    if (directory.empty())
      return false;
    if (directory == "/")
      return true;

    std::string current = directory[0] == '/' ? "/" : "";
    std::stringstream ss(directory);
    std::string token;

    while (std::getline(ss, token, '/'))
    {
      if (token.empty())
        continue;
      if (!current.empty() && current.back() != '/')
        current += "/";
      current += token;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        ROS_WARN_STREAM("[dvc_diag] Failed creating directory: " << current << " errno=" << errno);
        return false;
      }
    }
    return true;
  }

  static double clampScalar(const double value, const double low, const double high)
  {
    return std::max(low, std::min(high, value));
  }

  bool isPerfMode() const { return perf_cfg_.mode == "perf"; }

  bool shouldRunOnCycle(const int decimation) const
  {
    if (decimation <= 1)
      return true;
    return (publish_cycle_count_ % static_cast<uint64_t>(decimation)) == 0;
  }

  bool regularizeCovariance(Eigen::Matrix3d& cov, const double min_eigenvalue_floor) const
  {
    if (!cov.allFinite())
      return false;
    cov = 0.5 * (cov + cov.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if (eig.info() != Eigen::Success)
      return false;

    Eigen::Vector3d evals = eig.eigenvalues();
    evals(0)              = std::max(evals(0), min_eigenvalue_floor);
    evals(1)              = std::max(evals(1), min_eigenvalue_floor);
    evals(2)              = std::max(evals(2), min_eigenvalue_floor);
    cov                   = eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();
    cov                   = 0.5 * (cov + cov.transpose());
    return cov.allFinite();
  }

  bool computeRadarCovariance(const Eigen::Matrix3d& cov_reve,
                              const reve::RadarEstimationDiag& diag,
                              Eigen::Matrix3d& cov_used,
                              RadarCovDiagFields& cov_diag) const
  {
    cov_used = cov_reve;
    cov_diag = RadarCovDiagFields();

    if (!cov_reve.allFinite())
      return false;

    const std::string& mode = radar_cov_recalib_cfg_.cov_mode;
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    if (mode == "base")
    {
      cov_diag.alpha_r = 1.0;
      cov_diag.s_k_skip_reason = "mode_no_sk";
      const bool ok = regularizeCovariance(cov_used, radar_cov_recalib_cfg_.sigma_min2);
      if (ok)
      {
        cov_diag.trace_R_after_alpha = cov_used.trace();
        cov_diag.trace_R_after_sk = cov_used.trace();
      }
      return ok;
    }

    if (mode == "fixed")
    {
      cov_diag.alpha_r = radar_cov_recalib_cfg_.fixed_scale;
      cov_diag.s_k_skip_reason = "mode_no_sk";
      cov_used = cov_diag.alpha_r * cov_reve + radar_cov_recalib_cfg_.sigma_min2 * I;
      const bool ok = regularizeCovariance(cov_used, radar_cov_recalib_cfg_.sigma_min2);
      if (ok)
      {
        cov_diag.trace_R_after_alpha = cov_used.trace();
        cov_diag.trace_R_after_sk = cov_used.trace();
      }
      return ok;
    }

    if (mode != "alpha_r" && mode != "alpha_r_sk")
      return false;

    const double denom = std::log10(radar_cov_recalib_cfg_.max_r_cond_ref);
    if (!(std::isfinite(denom) && denom > 0.0))
      return false;

    const double cond         = std::isfinite(diag.cond) ? std::fabs(diag.cond) : radar_cov_recalib_cfg_.max_r_cond_ref;
    const double inlier_ratio = std::isfinite(diag.inlier_ratio) ? diag.inlier_ratio : 0.0;
    const double n_targets    = static_cast<double>(diag.n_targets);

    const double f_cond = clampScalar(std::log10(std::max(cond, 1.0)) / denom, 0.0, 1.0);
    const double f_inlier = clampScalar(1.0 - inlier_ratio, 0.0, 1.0);
    const double f_sparse = clampScalar(
        (radar_cov_recalib_cfg_.n_targets_ref - n_targets) / radar_cov_recalib_cfg_.n_targets_ref, 0.0, 1.0);

    cov_diag.d_r = radar_cov_recalib_cfg_.w_cond * f_cond + radar_cov_recalib_cfg_.w_inlier * f_inlier +
                   radar_cov_recalib_cfg_.w_sparse * f_sparse;
    cov_diag.alpha_r = clampScalar(1.0 + radar_cov_recalib_cfg_.k_alpha * cov_diag.d_r, 1.0, radar_cov_recalib_cfg_.alpha_r_max);

    Eigen::Matrix3d cov_alpha = cov_diag.alpha_r * cov_reve + radar_cov_recalib_cfg_.sigma_min2 * I;
    if (!regularizeCovariance(cov_alpha, radar_cov_recalib_cfg_.sigma_min2))
      return false;
    cov_diag.trace_R_after_alpha = cov_alpha.trace();

    if (mode == "alpha_r")
    {
      cov_used = cov_alpha;
      cov_diag.trace_R_after_sk = cov_used.trace();
      cov_diag.s_k_skip_reason = "mode_alpha_r";
      return true;
    }

    cov_used = cov_alpha;
    if (!radar_sk_cfg_.enable || !diag.obs_valid || !diag.obs_ut_u.allFinite())
    {
      cov_diag.s_k_skip_reason = !radar_sk_cfg_.enable ? "sk_disabled" : "obs_invalid";
      cov_diag.trace_R_after_sk = cov_used.trace();
      return true;
    }

    Eigen::Matrix3d G = 0.5 * (diag.obs_ut_u + diag.obs_ut_u.transpose());
    G += radar_sk_cfg_.eps_lambda * I;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(G);
    if (es.info() != Eigen::Success || !es.eigenvalues().allFinite() || !es.eigenvectors().allFinite())
    {
      cov_diag.s_k_skip_reason = "obs_eig_fail";
      cov_diag.trace_R_after_sk = cov_used.trace();
      return true;
    }

    const Eigen::Vector3d evals = es.eigenvalues();
    cov_diag.lambda1_obs = evals(2);
    cov_diag.lambda2_obs = evals(1);
    cov_diag.lambda3_obs = evals(0);
    cov_diag.obs_trace = cov_diag.lambda1_obs + cov_diag.lambda2_obs + cov_diag.lambda3_obs;
    cov_diag.obs_aniso =
        (cov_diag.lambda1_obs - cov_diag.lambda3_obs) / (cov_diag.lambda1_obs + radar_sk_cfg_.eps_lambda);

    if (radar_sk_cfg_.apply_gate_enable)
    {
      cov_diag.sk_gate_lambda3_pass = cov_diag.lambda3_obs <= radar_sk_cfg_.lambda3_apply_max ? 1 : 0;
      cov_diag.sk_gate_d_r_pass = cov_diag.d_r >= radar_sk_cfg_.d_r_apply_min ? 1 : 0;
      cov_diag.sk_gate_ntargets_pass =
          static_cast<double>(diag.n_targets) <= radar_sk_cfg_.n_targets_apply_max ? 1 : 0;
      cov_diag.sk_gate_obs_trace_pass = cov_diag.obs_trace <= radar_sk_cfg_.obs_trace_apply_max ? 1 : 0;
      cov_diag.sk_gate_obs_aniso_pass = cov_diag.obs_aniso >= radar_sk_cfg_.obs_aniso_apply_min ? 1 : 0;
      if (cov_diag.sk_gate_lambda3_pass == 0 || cov_diag.sk_gate_d_r_pass == 0 || cov_diag.sk_gate_ntargets_pass == 0 ||
          cov_diag.sk_gate_obs_trace_pass == 0 || cov_diag.sk_gate_obs_aniso_pass == 0)
      {
        if (cov_diag.sk_gate_lambda3_pass == 0)
          cov_diag.s_k_skip_reason = "gate_lambda3";
        else if (cov_diag.sk_gate_d_r_pass == 0)
          cov_diag.s_k_skip_reason = "gate_d_r";
        else if (cov_diag.sk_gate_ntargets_pass == 0)
          cov_diag.s_k_skip_reason = "gate_n_targets";
        else if (cov_diag.sk_gate_obs_trace_pass == 0)
          cov_diag.s_k_skip_reason = "gate_obs_trace";
        else
          cov_diag.s_k_skip_reason = "gate_obs_aniso";
        cov_diag.trace_R_after_sk = cov_used.trace();
        return true;
      }
    }
    else
    {
      cov_diag.sk_gate_lambda3_pass = 1;
      cov_diag.sk_gate_d_r_pass = 1;
      cov_diag.sk_gate_ntargets_pass = 1;
      cov_diag.sk_gate_obs_trace_pass = 1;
      cov_diag.sk_gate_obs_aniso_pass = 1;
    }

    const Eigen::Matrix3d V = es.eigenvectors();
    Eigen::Vector3d s_values;
    for (int i = 0; i < 3; ++i)
    {
      const double ratio = std::max(0.0, (radar_sk_cfg_.tau_obs - evals(i)) / (radar_sk_cfg_.tau_obs + radar_sk_cfg_.eps_lambda));
      s_values(i) = clampScalar(1.0 + radar_sk_cfg_.c_obs * ratio, 1.0, radar_sk_cfg_.s_max);
    }

    const Eigen::Matrix3d S_k = V * s_values.asDiagonal() * V.transpose();
    Eigen::Matrix3d cov_sk = S_k * cov_alpha * S_k.transpose();
    if (!regularizeCovariance(cov_sk, radar_cov_recalib_cfg_.sigma_min2))
    {
      cov_diag.s_k_skip_reason = "sk_cov_regularize_fail";
      cov_diag.trace_R_after_sk = cov_used.trace();
      return true;
    }

    cov_diag.s_k_valid = 1;
    cov_diag.s_k_applied = 1;
    cov_diag.s_k_skip_reason = "applied";
    cov_diag.s1 = s_values(2);
    cov_diag.s2 = s_values(1);
    cov_diag.s3 = s_values(0);
    cov_diag.trace_R_after_sk = cov_sk.trace();
    cov_used = cov_sk;
    return true;
  }

  void initDvcDiagLogging()
  {
    if (!dvc_diag_enabled_)
      return;

    if (dvc_diag_output_dir_.empty())
    {
      ROS_WARN_STREAM("[dvc_diag] dvc_diag_enabled is true but dvc_diag_output_dir is empty. Disable diagnostic logging.");
      dvc_diag_enabled_ = false;
      return;
    }

    if (!ensureDirectory(dvc_diag_output_dir_))
    {
      ROS_WARN_STREAM("[dvc_diag] Cannot create output directory: " << dvc_diag_output_dir_);
      dvc_diag_enabled_ = false;
      return;
    }

    if (dvc_run_id_.empty())
    {
      std::ostringstream oss;
      oss << "run_" << ros::WallTime::now().toNSec();
      dvc_run_id_ = oss.str();
    }

    const std::string diag_file = dvc_diag_output_dir_ + "/dvc_diag_" + dvc_run_id_ + ".csv";
    dvc_diag_stream_.open(diag_file.c_str(), std::ios::out | std::ios::trunc);
    if (!dvc_diag_stream_.good())
    {
      ROS_WARN_STREAM("[dvc_diag] Failed to open: " << diag_file);
      dvc_diag_enabled_ = false;
      return;
    }

    dvc_diag_stream_ << "timestamp,cond,inlier_ratio,trace_R_used,minEig_R_used,use_radar_update,runtime_reve_ms,"
                        "runtime_backend_ms,radar_scan_callback_count,row_id,cov_mode,d_r,alpha_r,n_targets,n_inliers,"
                        "nis_vel,nis_valid,nis_exceed_95,radar_update_committed,s_k_valid,lambda1_obs,lambda2_obs,"
                        "lambda3_obs,obs_trace,obs_aniso,s1,s2,s3,trace_R_after_alpha,trace_R_after_sk,s_k_applied,"
                        "s_k_skip_reason,sk_gate_lambda3_pass,sk_gate_d_r_pass,sk_gate_ntargets_pass,"
                        "sk_gate_obs_trace_pass,sk_gate_obs_aniso_pass\n";
    dvc_diag_stream_.flush();
    ROS_INFO_STREAM("[dvc_diag] Logging enabled: " << diag_file);
  }

  void writeDvcDiagRow(const double timestamp,
                       const reve::RadarEstimationDiag& diag,
                       const Eigen::Matrix3d* cov_v_b_r,
                       const int use_radar_update,
                       const double runtime_reve_ms,
                       const double runtime_backend_ms,
                       const std::string& cov_mode,
                       const RadarCovDiagFields& cov_diag,
                       const double nis_vel,
                       const int nis_valid,
                       const int nis_exceed_95,
                       const int radar_update_committed)
  {
    if (!dvc_diag_enabled_ || !dvc_diag_stream_.good())
      return;

    double trace_R = std::numeric_limits<double>::quiet_NaN();
    double minEig  = std::numeric_limits<double>::quiet_NaN();

    if (cov_v_b_r != nullptr && cov_v_b_r->allFinite())
    {
      const Eigen::Matrix3d sym_cov = 0.5 * ((*cov_v_b_r) + cov_v_b_r->transpose());
      trace_R                       = sym_cov.trace();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(sym_cov);
      if (es.info() == Eigen::Success)
      {
        minEig = es.eigenvalues().minCoeff();
      }
    }

    dvc_diag_stream_ << std::fixed << std::setprecision(9) << timestamp << "," << diag.cond << "," << diag.inlier_ratio
                     << "," << trace_R << "," << minEig << "," << use_radar_update << "," << runtime_reve_ms << ","
                     << runtime_backend_ms << "," << radar_scan_callback_count_ << "," << dvc_diag_rows_++ << ","
                     << cov_mode << "," << cov_diag.d_r << "," << cov_diag.alpha_r << "," << diag.n_targets << ","
                     << diag.n_inliers << "," << nis_vel << "," << nis_valid << "," << nis_exceed_95 << ","
                     << radar_update_committed << "," << cov_diag.s_k_valid << "," << cov_diag.lambda1_obs << ","
                     << cov_diag.lambda2_obs << "," << cov_diag.lambda3_obs << "," << cov_diag.obs_trace << ","
                     << cov_diag.obs_aniso << "," << cov_diag.s1 << "," << cov_diag.s2 << "," << cov_diag.s3 << ","
                     << cov_diag.trace_R_after_alpha << "," << cov_diag.trace_R_after_sk << "," << cov_diag.s_k_applied
                     << "," << cov_diag.s_k_skip_reason << ","
                     << cov_diag.sk_gate_lambda3_pass << "," << cov_diag.sk_gate_d_r_pass << ","
                     << cov_diag.sk_gate_ntargets_pass << "," << cov_diag.sk_gate_obs_trace_pass << ","
                     << cov_diag.sk_gate_obs_aniso_pass
                     << "\n";
    dvc_diag_stream_.flush();
  }

  bool isEventMode() const { return scheduler_mode_ != SchedulerMode::Legacy; }
  bool isWorkerThread() const { return isEventMode() && std::this_thread::get_id() == worker_thread_id_; }
  bool isImuFastPathEnabled() const { return scheduler_mode_ == SchedulerMode::EventStage2 && scheduler_cfg_.imu_fast_path; }
  struct SchedulerSnapshot
  {
    double latest_imu_time = -std::numeric_limits<double>::infinity();
    double oldest_pending_radar_timestamp = std::numeric_limits<double>::infinity();
    size_t event_queue_depth = 0;
    size_t imu_fast_depth = 0;
    bool radar_needs_imu_catchup = false;
  };
  SchedulerSnapshot getSchedulerSnapshot()
  {
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    SchedulerSnapshot snapshot;
    snapshot.latest_imu_time = latest_imu_time_;
    snapshot.oldest_pending_radar_timestamp = oldest_pending_radar_timestamp_;
    snapshot.event_queue_depth = event_queue_.size();
    snapshot.imu_fast_depth = imu_fast_queue_.size();
    snapshot.radar_needs_imu_catchup = std::isfinite(oldest_pending_radar_timestamp_) &&
                                       latest_imu_time_ < oldest_pending_radar_timestamp_ + scheduler_cfg_.radar_imu_window_s;
    return snapshot;
  }
  bool radarNeedsImuCatchup()
  {
    const SchedulerSnapshot snapshot = getSchedulerSnapshot();
    return snapshot.radar_needs_imu_catchup;
  }
  size_t getSchedulerQueueDepth()
  {
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    return event_queue_.size();
  }
  size_t getImuFastQueueDepth()
  {
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    return imu_fast_queue_.size();
  }
  size_t getSchedulerMaxEvents() const { return static_cast<size_t>(scheduler_cfg_.max_events); }
  void noteLoaderBackpressureWait(const double wait_ms,
                                  const std::string& reason,
                                  const size_t queue_depth,
                                  const size_t imu_fast_depth,
                                  const bool timed_out)
  {
    if (!isEventMode() || !std::isfinite(wait_ms) || wait_ms < 0.0)
      return;
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    loader_backpressure_pending_ = true;
    pending_loader_backpressure_wait_ms_ = wait_ms;
    pending_loader_backpressure_reason_ = reason.empty() ? "none" : reason;
    pending_loader_backpressure_queue_depth_ = static_cast<uint64_t>(queue_depth);
    pending_loader_backpressure_imu_depth_ = static_cast<uint64_t>(imu_fast_depth);
    pending_loader_backpressure_timeout_ = timed_out ? 1 : 0;
  }
  bool waitUntilSchedulerQueueBelow(const size_t target_depth, const double timeout_s)
  {
    std::unique_lock<std::mutex> lock(scheduler_mutex_);
    const auto pred = [&]() {
      return !ros::ok() || event_queue_.size() <= target_depth || !worker_running_.load();
    };
    if (timeout_s <= 0.0)
    {
      scheduler_cv_.wait(lock, pred);
      return event_queue_.size() <= target_depth;
    }
    const bool ok = scheduler_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), pred);
    return ok && event_queue_.size() <= target_depth;
  }
  bool waitUntilImuFastQueueBelow(const size_t target_depth, const double timeout_s)
  {
    std::unique_lock<std::mutex> lock(scheduler_mutex_);
    const auto pred = [&]() {
      return !ros::ok() || imu_fast_queue_.size() <= target_depth || !worker_running_.load();
    };
    if (timeout_s <= 0.0)
    {
      scheduler_cv_.wait(lock, pred);
      return imu_fast_queue_.size() <= target_depth;
    }
    const bool ok = scheduler_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), pred);
    return ok && imu_fast_queue_.size() <= target_depth;
  }
  bool waitUntilSchedulerDrained(const double timeout_s)
  {
    scheduler_drain_requested_.store(true);
    scheduler_cv_.notify_all();

    std::unique_lock<std::mutex> lock(scheduler_mutex_);
    const auto pred = [&]() {
      return !ros::ok() || ((!worker_running_.load() || scheduler_mode_ == SchedulerMode::Legacy) && event_queue_.empty()) ||
             (event_queue_.empty() && !worker_processing_.load());
    };

    bool ok = false;
    if (timeout_s <= 0.0)
    {
      scheduler_cv_.wait(lock, pred);
      ok = event_queue_.empty() && !worker_processing_.load();
    }
    else
    {
      ok = scheduler_cv_.wait_for(lock, std::chrono::duration<double>(timeout_s), pred) && event_queue_.empty() &&
           !worker_processing_.load();
    }

    scheduler_drain_requested_.store(false);
    return ok;
  }

  int eventPriority(const EventType type) const
  {
    switch (type)
    {
      case EventType::Imu:
        return 0;
      case EventType::Image0:
      case EventType::Image1:
        return 1;
      case EventType::RadarTrigger:
      case EventType::RadarScan:
        return 2;
      case EventType::GroundtruthPose:
      case EventType::GroundtruthOdometry:
      case EventType::Velocity:
        return 3;
      case EventType::Reset:
      case EventType::ResetToPose:
        return 4;
      default:
        return 5;
    }
  }

  static const char* eventTypeName(const EventType type)
  {
    switch (type)
    {
      case EventType::Imu:
        return "imu";
      case EventType::Image0:
        return "image0";
      case EventType::Image1:
        return "image1";
      case EventType::GroundtruthPose:
        return "groundtruth_pose";
      case EventType::GroundtruthOdometry:
        return "groundtruth_odom";
      case EventType::Velocity:
        return "velocity";
      case EventType::RadarTrigger:
        return "radar_trigger";
      case EventType::RadarScan:
        return "radar_scan";
      case EventType::Reset:
        return "reset";
      case EventType::ResetToPose:
        return "reset_to_pose";
      default:
        return "unknown";
    }
  }

  void initSchedulerDiagLogging()
  {
    if (!scheduler_cfg_.enable_diag)
      return;

    std::string output_dir = scheduler_cfg_.diag_output_dir.empty() ? dvc_diag_output_dir_ : scheduler_cfg_.diag_output_dir;
    if (output_dir.empty())
    {
      ROS_WARN_STREAM("[scheduler] diagnostics enabled but no output directory set.");
      scheduler_cfg_.enable_diag = false;
      return;
    }
    if (!ensureDirectory(output_dir))
    {
      ROS_WARN_STREAM("[scheduler] failed creating diagnostics directory: " << output_dir);
      scheduler_cfg_.enable_diag = false;
      return;
    }

    if (dvc_run_id_.empty())
    {
      std::ostringstream oss;
      oss << "run_" << ros::WallTime::now().toNSec();
      dvc_run_id_ = oss.str();
    }

    dvc_sched_diag_path_ = output_dir + "/dvc_sched_diag_" + dvc_run_id_ + ".csv";
    dvc_sched_diag_stream_.open(dvc_sched_diag_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!dvc_sched_diag_stream_.good())
    {
      ROS_WARN_STREAM("[scheduler] failed opening diagnostics file: " << dvc_sched_diag_path_);
      scheduler_cfg_.enable_diag = false;
      dvc_sched_diag_path_.clear();
      return;
    }

    dvc_sched_diag_stream_
        << "row_id,event_timestamp,event_enqueue_wall_time,event_process_wall_time,scheduler_mode,event_type,"
        << "queue_depth,event_latency_ms,watermark_wait_ms,drop_total,drop_type,lock_wait_us,lock_hold_us,"
        << "radar_enqueue_to_commit_ms,radar_starved,queue_wait_us,select_ready_us,worker_dispatch_us,"
        << "mfilter_lock_wait_us,mfilter_lock_hold_us,reve_ms,backend_ms,publish_ms,diag_io_ms,"
        << "loader_backpressure_wait_ms,loader_backpressure_reason,loader_backpressure_queue_depth,"
        << "loader_backpressure_imu_depth,loader_backpressure_timeout,event_queue_wait_ms,worker_dispatch_ms,"
        << "update_publish_ms,diag_write_ms\n";
    dvc_sched_diag_stream_.flush();
  }

  static size_t csvColumnCount(const std::string& line)
  {
    return std::count(line.begin(), line.end(), ',') + 1;
  }

  void sanitizeSchedulerDiagFileTail()
  {
    if (dvc_sched_diag_path_.empty())
      return;

    std::ifstream in(dvc_sched_diag_path_.c_str(), std::ios::in);
    if (!in.good())
      return;

    std::vector<std::string> lines;
    lines.reserve(dvc_sched_diag_rows_ + 1);
    std::string line;
    while (std::getline(in, line))
      lines.push_back(line);
    in.close();

    if (lines.size() <= 1)
      return;

    const size_t expected_cols = csvColumnCount(lines.front());
    size_t removed_tail_rows = 0;
    while (lines.size() > 1 && csvColumnCount(lines.back()) != expected_cols)
    {
      lines.pop_back();
      removed_tail_rows++;
    }

    if (removed_tail_rows == 0)
      return;

    std::ofstream out(dvc_sched_diag_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!out.good())
    {
      ROS_WARN_STREAM("[scheduler] failed rewriting diagnostics file after tail cleanup: " << dvc_sched_diag_path_);
      return;
    }
    for (const auto& row : lines)
      out << row << "\n";
    out.flush();
    ROS_WARN_STREAM("[scheduler] removed " << removed_tail_rows
                                           << " truncated tail row(s) from diagnostics file: "
                                           << dvc_sched_diag_path_);
  }

  void writeSchedulerDiagRow(const SensorEvent& event,
                             const size_t queue_depth,
                             const double event_latency_ms,
                             const double watermark_wait_ms,
                             const double lock_wait_us,
                             const double lock_hold_us,
                             const double radar_enqueue_to_commit_ms,
                             const int radar_starved,
                             const double queue_wait_us = std::numeric_limits<double>::quiet_NaN(),
                             const double select_ready_us = std::numeric_limits<double>::quiet_NaN(),
                             const double worker_dispatch_us = std::numeric_limits<double>::quiet_NaN(),
                             const double mfilter_lock_wait_us = std::numeric_limits<double>::quiet_NaN(),
                             const double mfilter_lock_hold_us = std::numeric_limits<double>::quiet_NaN(),
                             const double reve_ms = std::numeric_limits<double>::quiet_NaN(),
                             const double backend_ms = std::numeric_limits<double>::quiet_NaN(),
                             const double publish_ms = std::numeric_limits<double>::quiet_NaN(),
                             const bool force_log = false)
  {
    if (!scheduler_cfg_.enable_diag || !dvc_sched_diag_stream_.good())
      return;
    if (!ros::ok() || ros::isShuttingDown())
      return;
    if (!force_log && scheduler_mode_ != SchedulerMode::Legacy && !scheduler_cfg_.diag_log_all_events &&
        event.type != EventType::RadarScan && event.type != EventType::RadarTrigger)
    {
      return;
    }

    const double event_queue_wait_ms =
        std::isfinite(queue_wait_us) ? (queue_wait_us * 1.0e-3) : std::numeric_limits<double>::quiet_NaN();
    const double worker_dispatch_ms =
        std::isfinite(worker_dispatch_us) ? (worker_dispatch_us * 1.0e-3) : std::numeric_limits<double>::quiet_NaN();
    const double update_publish_ms =
        (std::isfinite(backend_ms) && std::isfinite(publish_ms)) ? (backend_ms + publish_ms) : std::numeric_limits<double>::quiet_NaN();
    const double diag_write_ms = last_diag_io_ms_;

    std::ostringstream row;
    row << dvc_sched_diag_rows_++ << "," << std::fixed << std::setprecision(9) << event.timestamp << ","
        << event.enqueue_wall_time << "," << ros::WallTime::now().toSec() << "," << scheduler_cfg_.mode << ","
        << eventTypeName(event.type) << "," << queue_depth << "," << event_latency_ms << "," << watermark_wait_ms
        << "," << event_queue_stats_.dropped_total << "," << last_drop_type_ << "," << lock_wait_us << ","
        << lock_hold_us << "," << radar_enqueue_to_commit_ms << "," << radar_starved << "," << queue_wait_us << ","
        << select_ready_us << "," << worker_dispatch_us << "," << mfilter_lock_wait_us << ","
        << mfilter_lock_hold_us << "," << reve_ms << "," << backend_ms << "," << publish_ms << ","
        << last_diag_io_ms_ << "," << event.loader_backpressure_wait_ms << "," << event.loader_backpressure_reason
        << "," << event.loader_backpressure_queue_depth << "," << event.loader_backpressure_imu_depth << ","
        << event.loader_backpressure_timeout << "," << event_queue_wait_ms << "," << worker_dispatch_ms << ","
        << update_publish_ms << "," << diag_write_ms << "\n";

    const std::string row_str = row.str();
    const double t_diag_write_begin = ros::WallTime::now().toSec();
    dvc_sched_diag_stream_.write(row_str.c_str(), static_cast<std::streamsize>(row_str.size()));

    if (!isPerfMode() || (dvc_sched_diag_rows_ % static_cast<uint64_t>(std::max(1, perf_cfg_.diag_flush_every_n))) == 0 ||
        force_log)
    {
      dvc_sched_diag_stream_.flush();
    }
    const double t_diag_write_end = ros::WallTime::now().toSec();
    last_diag_io_ms_ = std::max(0.0, (t_diag_write_end - t_diag_write_begin) * 1000.0);
  }

  void enqueueEvent(SensorEvent event)
  {
    if (!isEventMode())
      return;

    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      event.seq               = ++event_seq_;
      event.enqueue_wall_time = ros::WallTime::now().toSec();
      if (loader_backpressure_pending_ && event.type != EventType::Imu)
      {
        event.loader_backpressure_wait_ms = pending_loader_backpressure_wait_ms_;
        event.loader_backpressure_reason = pending_loader_backpressure_reason_;
        event.loader_backpressure_queue_depth = pending_loader_backpressure_queue_depth_;
        event.loader_backpressure_imu_depth = pending_loader_backpressure_imu_depth_;
        event.loader_backpressure_timeout = pending_loader_backpressure_timeout_;
        loader_backpressure_pending_ = false;
      }

      if (event_queue_.size() >= static_cast<size_t>(scheduler_cfg_.max_events))
      {
        const SensorEvent dropped = event_queue_.front();
        event_queue_.pop_front();
        watermark_block_start_sec_.erase(dropped.seq);
        if (dropped.type == EventType::RadarScan &&
            std::fabs(dropped.timestamp - oldest_pending_radar_timestamp_) < 1.0e-9)
        {
          recomputeOldestPendingRadarTimestampUnlocked();
        }
        event_queue_stats_.dropped_total++;
        event_queue_stats_.dropped_by_type[dropped.type]++;
        last_drop_type_ = eventTypeName(dropped.type);
      }

      event_queue_.push_back(event);
      if (event.type == EventType::RadarScan)
      {
        oldest_pending_radar_timestamp_ = std::min(oldest_pending_radar_timestamp_, event.timestamp);
      }
      event_queue_stats_.enqueued++;
    }
    scheduler_cv_.notify_all();
  }

  void recomputeOldestPendingRadarTimestampUnlocked()
  {
    oldest_pending_radar_timestamp_ = std::numeric_limits<double>::infinity();
    for (const auto& pending : event_queue_)
    {
      if (pending.type == EventType::RadarScan)
      {
        oldest_pending_radar_timestamp_ = std::min(oldest_pending_radar_timestamp_, pending.timestamp);
      }
    }
  }

  bool isEventEligible(const SensorEvent& event) const
  {
    if (event.type == EventType::Imu || event.type == EventType::Reset || event.type == EventType::ResetToPose)
      return true;
    if (scheduler_mode_ == SchedulerMode::EventStage2 && scheduler_cfg_.imu_fast_path &&
        event.type == EventType::RadarScan)
    {
      const double required_imu_horizon = std::max(scheduler_cfg_.watermark_margin_s, scheduler_cfg_.radar_imu_window_s);
      return latest_imu_time_ >= event.timestamp + required_imu_horizon;
    }
    return latest_imu_time_ >= event.timestamp + scheduler_cfg_.watermark_margin_s;
  }

  bool popNextReadyEvent(SensorEvent& event,
                         double& watermark_wait_ms,
                         size_t& queue_depth_after_pop,
                         double& queue_wait_us,
                         double& select_ready_us)
  {
    queue_wait_us = 0.0;
    select_ready_us = 0.0;
    std::unique_lock<std::mutex> lock(scheduler_mutex_);
    while (worker_running_.load())
    {
      if (event_queue_.empty())
      {
        const auto t_wait_begin = ros::WallTime::now().toSec();
        scheduler_cv_.wait(lock, [&]() { return !worker_running_.load() || !event_queue_.empty(); });
        const auto t_wait_end = ros::WallTime::now().toSec();
        queue_wait_us += std::max(0.0, (t_wait_end - t_wait_begin) * 1.0e6);
        continue;
      }

      const auto t_select_begin = ros::WallTime::now().toSec();
      size_t best_idx      = event_queue_.size();
      double best_ts       = std::numeric_limits<double>::infinity();
      int best_priority    = std::numeric_limits<int>::max();
      uint64_t best_seq    = std::numeric_limits<uint64_t>::max();
      bool any_waiting_non_imu = false;
      const double now_wall = ros::WallTime::now().toSec();
      double longest_wait_s = 0.0;

      for (size_t i = 0; i < event_queue_.size(); ++i)
      {
        const SensorEvent& candidate = event_queue_[i];
        if (!isEventEligible(candidate))
        {
          if (candidate.type != EventType::Imu)
          {
            any_waiting_non_imu = true;
            if (watermark_block_start_sec_.find(candidate.seq) == watermark_block_start_sec_.end())
            {
              watermark_block_start_sec_[candidate.seq] = now_wall;
            }
            const auto it_block = watermark_block_start_sec_.find(candidate.seq);
            if (it_block != watermark_block_start_sec_.end())
            {
              const double wait_s = now_wall - it_block->second;
              if (wait_s > longest_wait_s)
                longest_wait_s = wait_s;
            }
          }
          continue;
        }

        const int p = eventPriority(candidate.type);
        if (candidate.timestamp < best_ts ||
            (candidate.timestamp == best_ts && (p < best_priority || (p == best_priority && candidate.seq < best_seq))))
        {
          best_idx      = i;
          best_ts       = candidate.timestamp;
          best_priority = p;
          best_seq      = candidate.seq;
        }
      }
      const auto t_select_end = ros::WallTime::now().toSec();
      select_ready_us += std::max(0.0, (t_select_end - t_select_begin) * 1.0e6);

      if (best_idx == event_queue_.size())
      {
        if (any_waiting_non_imu)
        {
          event_queue_stats_.watermark_block_count++;
          size_t drop_idx = event_queue_.size();
          for (size_t i = 0; i < event_queue_.size(); ++i)
          {
            const SensorEvent& candidate = event_queue_[i];
            if (candidate.type == EventType::Imu || candidate.type == EventType::Reset ||
                candidate.type == EventType::ResetToPose)
              continue;
            const auto it_block = watermark_block_start_sec_.find(candidate.seq);
            if (it_block == watermark_block_start_sec_.end())
              continue;
            const double wait_s = now_wall - it_block->second;
            if (wait_s > longest_wait_s)
            {
              longest_wait_s = wait_s;
              drop_idx = i;
            }
          }

          if (drop_idx < event_queue_.size() && longest_wait_s >= scheduler_cfg_.watermark_max_wait_s)
          {
            const SensorEvent dropped = event_queue_[drop_idx];
            event_queue_.erase(event_queue_.begin() + drop_idx);
            watermark_block_start_sec_.erase(dropped.seq);
            if (dropped.type == EventType::RadarScan &&
                std::fabs(dropped.timestamp - oldest_pending_radar_timestamp_) < 1.0e-9)
            {
              recomputeOldestPendingRadarTimestampUnlocked();
            }
            event_queue_stats_.dropped_total++;
            event_queue_stats_.dropped_by_type[dropped.type]++;
            last_drop_type_ = eventTypeName(dropped.type);
            const double event_latency_ms = (now_wall - dropped.enqueue_wall_time) * 1000.0;
            const double watermark_wait_ms = longest_wait_s * 1000.0;
            writeSchedulerDiagRow(dropped,
                                  event_queue_.size(),
                                  event_latency_ms,
                                  watermark_wait_ms,
                                  0.0,
                                  0.0,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  0,
                                  queue_wait_us,
                                  select_ready_us,
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  true);
            continue;
          }
        }
        const double remaining_wait_s = std::max(0.0, scheduler_cfg_.watermark_max_wait_s - longest_wait_s);
        const double wait_quantum_s = std::max(0.001, std::min(remaining_wait_s, 0.01));
        const uint64_t enqueued_before_wait = event_queue_stats_.enqueued;
        const double latest_imu_before_wait = latest_imu_time_;
        const auto t_wait_begin = ros::WallTime::now().toSec();
        scheduler_cv_.wait_for(lock, std::chrono::duration<double>(wait_quantum_s), [&]() {
          return !worker_running_.load() || scheduler_drain_requested_.load() || event_queue_.empty() ||
                 event_queue_stats_.enqueued != enqueued_before_wait || latest_imu_time_ > latest_imu_before_wait;
        });
        const auto t_wait_end = ros::WallTime::now().toSec();
        queue_wait_us += std::max(0.0, (t_wait_end - t_wait_begin) * 1.0e6);
        continue;
      }

      event = event_queue_[best_idx];
      event_queue_.erase(event_queue_.begin() + best_idx);
      if (event.type == EventType::RadarScan &&
          std::fabs(event.timestamp - oldest_pending_radar_timestamp_) < 1.0e-9)
      {
        recomputeOldestPendingRadarTimestampUnlocked();
      }
      queue_depth_after_pop = event_queue_.size();
      watermark_wait_ms = 0.0;
      const auto it_block = watermark_block_start_sec_.find(event.seq);
      if (it_block != watermark_block_start_sec_.end())
      {
        watermark_wait_ms = std::max(0.0, (ros::WallTime::now().toSec() - it_block->second) * 1000.0);
        watermark_block_start_sec_.erase(it_block);
      }
      scheduler_cv_.notify_all();
      return true;
    }
    return false;
  }

  bool computeRadarOmegaFromHistory(const double scan_timestamp, Eigen::Vector3d& omega_mean)
  {
    omega_mean.setZero();
    size_t count = 0;
    const double end_time = scan_timestamp + scheduler_cfg_.radar_imu_window_s;
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    for (const auto& imu : imu_history_)
    {
      const double t = imu.header.stamp.toSec();
      if (t >= scan_timestamp && t <= end_time)
      {
        omega_mean += Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
        count++;
      }
    }
    if (count == 0)
      return false;
    omega_mean /= static_cast<double>(count);
    return true;
  }

  void updateImuHistoryUnlocked(const sensor_msgs::Imu& imu_msg)
  {
    imu_history_.push_back(imu_msg);
    double cutoff = imu_msg.header.stamp.toSec() - 1.0;
    if (scheduler_mode_ == SchedulerMode::EventStage2 && scheduler_cfg_.imu_fast_path &&
        std::isfinite(oldest_pending_radar_timestamp_))
    {
      // Preserve IMU support for the oldest pending radar event to avoid
      // fast-path timestamp starvation under rosbag burst playback.
      cutoff = std::min(cutoff, oldest_pending_radar_timestamp_ - scheduler_cfg_.radar_imu_window_s);
    }
    while (!imu_history_.empty() && imu_history_.front().header.stamp.toSec() < cutoff)
      imu_history_.pop_front();
  }

  void updateImuHistory(const sensor_msgs::Imu& imu_msg)
  {
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    updateImuHistoryUnlocked(imu_msg);
  }

  void updateImuHistory(const sensor_msgs::Imu::ConstPtr& imu_msg) { updateImuHistory(*imu_msg); }

  void drainImuFastQueueUpTo(const double max_timestamp, std::vector<sensor_msgs::Imu::ConstPtr>& imu_msgs)
  {
    imu_msgs.clear();
    std::lock_guard<std::mutex> lock(scheduler_mutex_);
    while (!imu_fast_queue_.empty() && imu_fast_queue_.front()->header.stamp.toSec() <= max_timestamp)
    {
      imu_msgs.push_back(imu_fast_queue_.front());
      imu_fast_queue_.pop_front();
    }
    if (!imu_msgs.empty())
      scheduler_cv_.notify_all();
  }

  void estimatorWorkerLoop()
  {
    worker_thread_id_ = std::this_thread::get_id();
    while (worker_running_.load())
    {
      SensorEvent event;
      double watermark_wait_ms = 0.0;
      size_t queue_depth_after_pop = 0;
      double queue_wait_us = 0.0;
      double select_ready_us = 0.0;
      if (!popNextReadyEvent(event, watermark_wait_ms, queue_depth_after_pop, queue_wait_us, select_ready_us))
        continue;

      const double process_start = ros::WallTime::now().toSec();
      const double event_latency_ms = (process_start - event.enqueue_wall_time) * 1000.0;

      worker_processing_.store(true);
      scheduler_cv_.notify_all();
      last_radar_enqueue_to_commit_ms_ = std::numeric_limits<double>::quiet_NaN();
      last_radar_starved_              = 0;
      active_radar_enqueue_wall_time_  = std::numeric_limits<double>::quiet_NaN();
      current_mfilter_lock_wait_us_    = 0.0;
      current_mfilter_lock_hold_us_    = 0.0;
      last_reve_ms_                    = std::numeric_limits<double>::quiet_NaN();
      last_backend_ms_                 = std::numeric_limits<double>::quiet_NaN();
      last_publish_ms_                 = std::numeric_limits<double>::quiet_NaN();

      if (scheduler_cfg_.imu_fast_path && event.type != EventType::Imu)
      {
        std::vector<sensor_msgs::Imu::ConstPtr> imu_batch;
        double imu_drain_horizon = event.timestamp + scheduler_cfg_.watermark_margin_s;
        if (scheduler_mode_ == SchedulerMode::EventStage2 && event.type == EventType::RadarScan)
        {
          imu_drain_horizon = event.timestamp + scheduler_cfg_.radar_imu_window_s;
        }
        drainImuFastQueueUpTo(imu_drain_horizon, imu_batch);
        for (const auto& imu_msg : imu_batch)
        {
          imuCallback(imu_msg);
        }
      }

      const double dispatch_begin = ros::WallTime::now().toSec();
      switch (event.type)
      {
        case EventType::Imu:
          latest_imu_time_ = std::max(latest_imu_time_, event.timestamp);
          updateImuHistory(event.imu_msg);
          imuCallback(event.imu_msg);
          break;
        case EventType::Image0:
          imgCallback0(event.img_msg);
          break;
        case EventType::Image1:
          imgCallback1(event.img_msg);
          break;
        case EventType::GroundtruthPose:
          groundtruthCallback(event.gt_msg);
          break;
        case EventType::GroundtruthOdometry:
          groundtruthOdometryCallback(event.gt_odom_msg);
          break;
        case EventType::Velocity:
          velocityCallback(event.vel_msg);
          break;
        case EventType::RadarTrigger:
          radarTriggerCallback(event.trigger_msg);
          break;
        case EventType::RadarScan:
          active_radar_enqueue_wall_time_ = event.enqueue_wall_time;
          radarScanCallback(event.radar_scan_msg);
          break;
        case EventType::Reset:
          requestReset();
          break;
        case EventType::ResetToPose:
          requestResetToPose(event.reset_WrWM, event.reset_qMW);
          break;
        default:
          break;
      }
      const double dispatch_end = ros::WallTime::now().toSec();
      const double worker_dispatch_us = std::max(0.0, (dispatch_end - dispatch_begin) * 1.0e6);

      active_radar_enqueue_wall_time_ = std::numeric_limits<double>::quiet_NaN();

      writeSchedulerDiagRow(event,
                            queue_depth_after_pop,
                            event_latency_ms,
                            watermark_wait_ms,
                            0.0,
                            0.0,
                            last_radar_enqueue_to_commit_ms_,
                            last_radar_starved_,
                            queue_wait_us,
                            select_ready_us,
                            worker_dispatch_us,
                            current_mfilter_lock_wait_us_,
                            current_mfilter_lock_hold_us_,
                            last_reve_ms_,
                            last_backend_ms_,
                            last_publish_ms_);
      event_queue_stats_.processed++;
      worker_processing_.store(false);
      scheduler_cv_.notify_all();
    }
  }

  /** \brief Tests the functionality of the rovio node.
   *
   *  @todo debug with   doVECalibration = false and depthType = 0
   */
  void makeTest()
  {
    mtFilterState* mpTestFilterState = new mtFilterState();
    *mpTestFilterState               = mpFilter_->init_;
    mpTestFilterState->setCamera(&mpFilter_->multiCamera_);
    mtState& testState = mpTestFilterState->state_;
    unsigned int s     = 2;
    testState.setRandom(s);
    predictionMeas_.setRandom(s);
    imgUpdateMeas_.setRandom(s);

    LWF::NormalVectorElement tempNor;
    for (int i = 0; i < mtState::nMax_; i++)
    {
      testState.CfP(i).camID_ = 0;
      tempNor.setRandom(s);
      if (tempNor.getVec()(2) < 0)
      {
        tempNor.boxPlus(Eigen::Vector2d(3.14, 0), tempNor);
      }
      testState.CfP(i).set_nor(tempNor);
      testState.CfP(i).trackWarping_ = false;
      tempNor.setRandom(s);
      if (tempNor.getVec()(2) < 0)
      {
        tempNor.boxPlus(Eigen::Vector2d(3.14, 0), tempNor);
      }
      testState.aux().feaCoorMeas_[i].set_nor(tempNor, true);
      testState.aux().feaCoorMeas_[i].mpCamera_ = &mpFilter_->multiCamera_.cameras_[0];
      testState.aux().feaCoorMeas_[i].camID_    = 0;
    }
    testState.CfP(0).camID_ = mtState::nCam_ - 1;
    mpTestFilterState->fsm_.setAllCameraPointers();

    // Prediction
    std::cout << "Testing Prediction" << std::endl;
    mpFilter_->mPrediction_.testPredictionJacs(testState, predictionMeas_, 1e-8, 1e-6, 0.1);

    // Update
    if (!mpImgUpdate_->useDirectMethod_)
    {
      std::cout << "Testing Update (can sometimes exhibit large absolut errors due to the float precision)"
                << std::endl;
      for (int i = 0; i < (std::min((int)mtState::nMax_, 2)); i++)
      {
        testState.aux().activeFeature_       = i;
        testState.aux().activeCameraCounter_ = 0;
        mpImgUpdate_->testUpdateJacs(testState, imgUpdateMeas_, 1e-4, 1e-5);
        testState.aux().activeCameraCounter_ = mtState::nCam_ - 1;
        mpImgUpdate_->testUpdateJacs(testState, imgUpdateMeas_, 1e-4, 1e-5);
      }
    }

    // Testing CameraOutputCF and CameraOutputCF
    std::cout << "Testing cameraOutputCF" << std::endl;
    cameraOutputCT_.testTransformJac(testState, 1e-8, 1e-6);
    std::cout << "Testing imuOutputCF" << std::endl;
    imuOutputCT_.testTransformJac(testState, 1e-8, 1e-6);
    std::cout << "Testing attitudeToYprCF" << std::endl;
    rovio::AttitudeToYprCT attitudeToYprCF;
    attitudeToYprCF.testTransformJac(1e-8, 1e-6);

    // Testing TransformFeatureOutputCT
    std::cout << "Testing transformFeatureOutputCT" << std::endl;
    transformFeatureOutputCT_.setFeatureID(0);
    if (mtState::nCam_ > 1)
    {
      transformFeatureOutputCT_.setOutputCameraID(1);
      transformFeatureOutputCT_.testTransformJac(testState, 1e-8, 1e-5);
    }
    transformFeatureOutputCT_.setOutputCameraID(0);
    transformFeatureOutputCT_.testTransformJac(testState, 1e-8, 1e-5);

    // Testing LandmarkOutputImuCT
    std::cout << "Testing LandmarkOutputImuCT" << std::endl;
    landmarkOutputImuCT_.setFeatureID(0);
    landmarkOutputImuCT_.testTransformJac(testState, 1e-8, 1e-5);

    // Getting featureOutput for next tests
    transformFeatureOutputCT_.transformState(testState, featureOutput_);
    if (!featureOutput_.c().isInFront())
    {
      featureOutput_.c().set_nor(featureOutput_.c().get_nor().rotated(QPD(0.0, 1.0, 0.0, 0.0)), false);
    }

    // Testing FeatureOutputReadableCT
    std::cout << "Testing FeatureOutputReadableCT" << std::endl;
    featureOutputReadableCT_.testTransformJac(featureOutput_, 1e-8, 1e-5);

    // Testing pixelOutputCT
    rovio::PixelOutputCT pixelOutputCT;
    std::cout << "Testing pixelOutputCT (can sometimes exhibit large absolut errors due to the float precision)"
              << std::endl;
    pixelOutputCT.testTransformJac(featureOutput_, 1e-4, 1.0);  // Reduces accuracy due to float and strong camera
                                                                // distortion

    // Testing ZeroVelocityUpdate_
    std::cout << "Testing zero velocity update" << std::endl;
    mpImgUpdate_->zeroVelocityUpdate_.testJacs();

    // Testing PoseUpdate
    if (!mpPoseUpdate_->noFeedbackToRovio_)
    {
      std::cout << "Testing pose update" << std::endl;
      mpPoseUpdate_->testUpdateJacs(1e-8, 1e-5);
    }

    delete mpTestFilterState;
  }

  /** \brief Callback for IMU-Messages. Adds IMU measurements (as prediction measurements) to the filter.
   */
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    if (isEventMode() && !isWorkerThread() && scheduler_cfg_.imu_fast_path)
    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      latest_imu_time_ = std::max(latest_imu_time_, imu_msg->header.stamp.toSec());
      imu_fast_queue_.push_back(imu_msg);
      updateImuHistoryUnlocked(*imu_msg);
      scheduler_cv_.notify_all();
      return;
    }

    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::Imu;
      event.timestamp = imu_msg->header.stamp.toSec();
      event.imu_msg   = imu_msg;
      enqueueEvent(event);
      return;
    }

    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    const double lock_wait_begin = ros::WallTime::now().toSec();
    double lock_wait_us = 0.0;
    double lock_hold_begin = 0.0;
    if (!isEventMode() || !isWorkerThread())
    {
      lock.lock();
      lock_wait_us = std::max(0.0, (ros::WallTime::now().toSec() - lock_wait_begin) * 1.0e6);
      lock_hold_begin = ros::WallTime::now().toSec();
    }

    if (most_recent_imus_.size() < 20)
      most_recent_imus_.emplace_back(*imu_msg);

    // check for radar scan processing wait for radar scan to be completed --> 20ms
    // TODO make parameter
    if (scheduler_mode_ != SchedulerMode::EventStage2 && most_recent_radar_scan_.header.stamp > ros::TIME_MIN &&
        most_recent_radar_scan_.header.stamp.toSec() + scheduler_cfg_.radar_imu_window_s < imu_msg->header.stamp.toSec())
    {
      // calc mean omega
      Eigen::Vector3d w(0, 0, 0);
      for (const auto& imu : most_recent_imus_)
        w += Eigen::Vector3d(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
      w = 1.0 / most_recent_imus_.size() * w;

      processRadarScan(w - mpFilter_->safe_.state_.gyb());
    }
    predictionMeas_.template get<mtPredictionMeas::_acc>() =
        Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    predictionMeas_.template get<mtPredictionMeas::_gyr>() =
        Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    if (init_state_.isInitialized())
    {
      mpFilter_->addPredictionMeas(predictionMeas_, imu_msg->header.stamp.toSec());
      // Event-queue modes defer costly updateSafe/publish calls to non-IMU events.
      if (scheduler_mode_ == SchedulerMode::Legacy)
      {
        updateAndPublish();
      }
    }
    else
    {
      switch (init_state_.state_)
      {
        case FilterInitializationState::State::WaitForInitExternalPose:
        {
          std::cout << "-- Filter: Initializing using external pose ..." << std::endl;
          mpFilter_->resetWithPose(init_state_.WrWM_, init_state_.qMW_, imu_msg->header.stamp.toSec());
          break;
        }
        case FilterInitializationState::State::WaitForInitUsingAccel:
        {
          std::cout << "-- Filter: Initializing using accel. measurement ..." << std::endl;
          mpFilter_->resetWithAccelerometer(predictionMeas_.template get<mtPredictionMeas::_acc>(),
                                            imu_msg->header.stamp.toSec());
          break;
        }
        default:
        {
          std::cout << "Unhandeld initialization type." << std::endl;
          abort();
          break;
        }
      }

      std::cout << std::setprecision(12);
      std::cout << "-- Filter: Initialized at t = " << imu_msg->header.stamp.toSec() << std::endl;
      init_state_.state_ = FilterInitializationState::State::Initialized;
    }
    if (!isEventMode() || !isWorkerThread())
    {
      current_mfilter_lock_wait_us_ = lock_wait_us;
      current_mfilter_lock_hold_us_ = std::max(0.0, (ros::WallTime::now().toSec() - lock_hold_begin) * 1.0e6);
    }
  }

  /** \brief Image callback for the camera with ID 0
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback0(const sensor_msgs::ImageConstPtr& img)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::Image0;
      event.timestamp = img->header.stamp.toSec();
      event.img_msg   = img;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    imgCallback(img, 0);
  }

  /** \brief Image callback for the camera with ID 1
   *
   * @param img - Image message.
   * @todo generalize
   */
  void imgCallback1(const sensor_msgs::ImageConstPtr& img)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::Image1;
      event.timestamp = img->header.stamp.toSec();
      event.img_msg   = img;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (mtState::nCam_ > 1)
      imgCallback(img, 1);
  }

  /** \brief Image callback. Adds images (as update measurements) to the filter.
   *
   *   @param img   - Image message.
   *   @param camID - Camera ID.
   */
  void imgCallback(const sensor_msgs::ImageConstPtr& img, const int camID = 0)
  {
    // Get image from msg
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_8UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat cv_img;
    cv_ptr->image.copyTo(cv_img);
    if (init_state_.isInitialized() && !cv_img.empty())
    {
      double msgTime = img->header.stamp.toSec();
      if (msgTime != imgUpdateMeas_.template get<mtImgMeas::_aux>().imgTime_)
      {
        for (int i = 0; i < mtState::nCam_; i++)
        {
          if (imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[i])
          {
            std::cout << "    \033[31mFailed Synchronization of Camera Frames, t = " << msgTime << "\033[0m"
                      << std::endl;
          }
        }
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
      }
      imgUpdateMeas_.template get<mtImgMeas::_aux>().pyr_[camID].computeFromImage(cv_img, true);
      imgUpdateMeas_.template get<mtImgMeas::_aux>().isValidPyr_[camID] = true;

      if (imgUpdateMeas_.template get<mtImgMeas::_aux>().areAllValid())
      {
        mpFilter_->template addUpdateMeas<0>(imgUpdateMeas_, msgTime);
        imgUpdateMeas_.template get<mtImgMeas::_aux>().reset(msgTime);
        updateAndPublish();
      }
    }
  }

  /** \brief Callback for external groundtruth as TransformStamped
   *
   *  @param transform - Groundtruth message.
   */
  void groundtruthCallback(const geometry_msgs::TransformStamped::ConstPtr& transform)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::GroundtruthPose;
      event.timestamp = transform->header.stamp.toSec();
      event.gt_msg    = transform;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (init_state_.isInitialized())
    {
      Eigen::Vector3d JrJV(
          transform->transform.translation.x, transform->transform.translation.y, transform->transform.translation.z);
      poseUpdateMeas_.pos() = JrJV;
      QPD qJV(transform->transform.rotation.w,
              transform->transform.rotation.x,
              transform->transform.rotation.y,
              transform->transform.rotation.z);
      poseUpdateMeas_.att() = qJV.inverted();
      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,
                                           transform->header.stamp.toSec() + mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external groundtruth as Odometry
   *
   * @param odometry - Groundtruth message.
   */
  void groundtruthOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::GroundtruthOdometry;
      event.timestamp = odometry->header.stamp.toSec();
      event.gt_odom_msg = odometry;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (init_state_.isInitialized())
    {
      Eigen::Vector3d JrJV(
          odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z);
      poseUpdateMeas_.pos() = JrJV;

      QPD qJV(odometry->pose.pose.orientation.w,
              odometry->pose.pose.orientation.x,
              odometry->pose.pose.orientation.y,
              odometry->pose.pose.orientation.z);
      poseUpdateMeas_.att() = qJV.inverted();

      const Eigen::Matrix<double, 6, 6> measuredCov =
          Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(odometry->pose.covariance.data());
      poseUpdateMeas_.measuredCov() = measuredCov;

      mpFilter_->template addUpdateMeas<1>(poseUpdateMeas_,
                                           odometry->header.stamp.toSec() + mpPoseUpdate_->timeOffset_);
      updateAndPublish();
    }
  }

  /** \brief Callback for external velocity measurements
   *
   *  @param transform - Groundtruth message.
   */
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::Velocity;
      event.timestamp = velocity->header.stamp.toSec();
      event.vel_msg   = velocity;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (init_state_.isInitialized())
    {
      Eigen::Vector3d AvM(velocity->twist.linear.x, velocity->twist.linear.y, velocity->twist.linear.z);
      velocityUpdateMeas_.vel() = AvM;
      mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_, velocity->header.stamp.toSec());
      updateAndPublish();
    }
  }

  /**
   * @brief radarTriggerCallback
   * @param header
   */
  void radarTriggerCallback(const std_msgs::HeaderConstPtr& header)
  {
    if (isEventMode() && !isWorkerThread())
    {
      // Trigger callback does not feed estimator state. Keep it out of the
      // event queue to avoid artificial backlog and latency inflation.
      return;
    }

    //    if (most_recent_imu_.header.stamp.toSec() + 5.0e-3 < header->stamp.toSec())
    //    {
    //      ROS_WARN("[radarTriggerCallback]: Most recent IMU %0.3f is older than radar trigger %0.3f!",
    //               most_recent_imu_.header.stamp.toSec(),
    //               header->stamp.toSec());
    //    }

    //    if ((most_recent_imu_.header.stamp - header->stamp).toSec() > 5.0e-3)
    //    {
    //      ROS_WARN_STREAM("[radarTriggerCallback]: Large time diff radar trigger and IMU: "
    //                      << (most_recent_imu_.header.stamp - header->stamp).toSec());
    //    }

    //    w_b_radar_trigger_ = Eigen::Vector3d(
    //        most_recent_imu_.angular_velocity.x, most_recent_imu_.angular_velocity.y,
    //        most_recent_imu_.angular_velocity.z);

    //    most_recent_imu_.clear();
  }

  /**
   * @brief radarScanCallback
   * @param radar_scan
   */
  void radarScanCallback(const sensor_msgs::PointCloud2ConstPtr& radar_scan)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::RadarScan;
      event.timestamp = radar_scan->header.stamp.toSec();
      event.radar_scan_msg = radar_scan;
      enqueueEvent(event);
      return;
    }

    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    const double lock_wait_begin = ros::WallTime::now().toSec();
    double lock_wait_us          = 0.0;
    double lock_hold_us          = 0.0;
    last_radar_enqueue_to_commit_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_radar_starved_              = 0;
    if (!isEventMode() || !isWorkerThread())
    {
      lock.lock();
      lock_wait_us = std::max(0.0, (ros::WallTime::now().toSec() - lock_wait_begin) * 1.0e6);
    }
    const double lock_hold_begin = ros::WallTime::now().toSec();
    const auto finalize_lock_metrics = [&]() {
      if (!isEventMode() || !isWorkerThread())
      {
        current_mfilter_lock_wait_us_ = lock_wait_us;
        current_mfilter_lock_hold_us_ = std::max(0.0, (ros::WallTime::now().toSec() - lock_hold_begin) * 1.0e6);
      }
    };
    radar_scan_callback_count_++;
    if (scheduler_mode_ == SchedulerMode::EventStage2)
    {
      most_recent_radar_scan_ = *radar_scan;
      Eigen::Vector3d w(0, 0, 0);
      if (!computeRadarOmegaFromHistory(radar_scan->header.stamp.toSec(), w))
      {
        last_radar_starved_ = 1;
        event_queue_stats_.radar_starved_count++;
        const reve::RadarEstimationDiag diag;
        const RadarCovDiagFields cov_diag;
        writeDvcDiagRow(radar_scan->header.stamp.toSec(),
                        diag,
                        nullptr,
                        0,
                        std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN(),
                        radar_cov_recalib_cfg_.cov_mode,
                        cov_diag,
                        std::numeric_limits<double>::quiet_NaN(),
                        0,
                        0,
                        0);
        most_recent_radar_scan_.header.stamp = ros::TIME_MIN;
        finalize_lock_metrics();
        return;
      }
      processRadarScan(w - mpFilter_->safe_.state_.gyb());
      finalize_lock_metrics();
      return;
    }

    // clear imu buffer --> collect imu measurements during radar scan for improved omega
    most_recent_imus_.clear();
    most_recent_radar_scan_ = *radar_scan;
    active_radar_enqueue_wall_time_ = ros::WallTime::now().toSec();
    (void)lock_wait_us;
    (void)lock_hold_us;
    (void)lock_hold_begin;
    finalize_lock_metrics();
  }

  void processRadarScan(const Eigen::Vector3d& w)
  {
    Eigen::Vector3d v_b_r;
    Eigen::Matrix3d cov_v_b_r_reve;
    Eigen::Matrix3d cov_v_b_r_used;
    reve::RadarEstimationDiag radar_diag;
    RadarCovDiagFields cov_diag;
    double runtime_reve_ms    = std::numeric_limits<double>::quiet_NaN();
    double runtime_backend_ms = std::numeric_limits<double>::quiet_NaN();
    double nis_vel            = std::numeric_limits<double>::quiet_NaN();
    int nis_valid             = 0;
    int nis_exceed_95         = 0;
    int radar_update_committed = 0;
    last_radar_enqueue_to_commit_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_reve_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_backend_ms_ = std::numeric_limits<double>::quiet_NaN();

    const double t_reve_start = ros::WallTime::now().toSec();

    if (radar_body_estimator_->estimate(most_recent_radar_scan_, w, v_b_r, cov_v_b_r_reve, &radar_diag))
    {
      runtime_reve_ms = (ros::WallTime::now().toSec() - t_reve_start) * 1000.0;
      last_reve_ms_ = runtime_reve_ms;
      if (init_state_.isInitialized())
      {
        if (!computeRadarCovariance(cov_v_b_r_reve, radar_diag, cov_v_b_r_used, cov_diag))
        {
          ROS_WARN_STREAM("[radarScanCallback] Invalid radar covariance after recalibration. Skip radar update.");
          writeDvcDiagRow(most_recent_radar_scan_.header.stamp.toSec(),
                          radar_diag,
                          &cov_v_b_r_reve,
                          0,
                          runtime_reve_ms,
                          runtime_backend_ms,
                          radar_cov_recalib_cfg_.cov_mode,
                          cov_diag,
                          nis_vel,
                          nis_valid,
                          nis_exceed_95,
                          radar_update_committed);
          most_recent_radar_scan_.header.stamp = ros::TIME_MIN;
          return;
        }

        const double t_backend_start = ros::WallTime::now().toSec();
        mpVelocityUpdate_->setMeasurementNoise(cov_v_b_r_used);

        velocityUpdateMeas_.vel() = v_b_r;

        // TODO make parameter
        mpFilter_->template addUpdateMeas<2>(velocityUpdateMeas_,
                                             most_recent_radar_scan_.header.stamp.toSec() + 10.0e-3);
        updateAndPublish();
        runtime_backend_ms = (ros::WallTime::now().toSec() - t_backend_start) * 1000.0;
        last_backend_ms_ = runtime_backend_ms;

        const VelocityUpdateDiag& vel_diag = mpVelocityUpdate_->getLastDiag();
        nis_vel = vel_diag.mahalanobis_distance;
        nis_valid = std::isfinite(nis_vel) ? 1 : 0;
        radar_update_committed = (nis_valid == 1 && !vel_diag.is_outlier) ? 1 : 0;
        nis_exceed_95          = (nis_valid == 1 && std::isfinite(nis_vel) && nis_vel > kChi2_3_95_) ? 1 : 0;
        if (radar_update_committed == 1 && std::isfinite(active_radar_enqueue_wall_time_))
        {
          last_radar_enqueue_to_commit_ms_ = (ros::WallTime::now().toSec() - active_radar_enqueue_wall_time_) * 1000.0;
        }

        writeDvcDiagRow(most_recent_radar_scan_.header.stamp.toSec(),
                        radar_diag,
                        &cov_v_b_r_used,
                        1,
                        runtime_reve_ms,
                        runtime_backend_ms,
                        radar_cov_recalib_cfg_.cov_mode,
                        cov_diag,
                        nis_vel,
                        nis_valid,
                        nis_exceed_95,
                        radar_update_committed);
      }
      else
      {
        // estimation succeeded but update not used because filter not initialized yet
        if (computeRadarCovariance(cov_v_b_r_reve, radar_diag, cov_v_b_r_used, cov_diag))
        {
          writeDvcDiagRow(most_recent_radar_scan_.header.stamp.toSec(),
                          radar_diag,
                          &cov_v_b_r_used,
                          0,
                          runtime_reve_ms,
                          runtime_backend_ms,
                          radar_cov_recalib_cfg_.cov_mode,
                          cov_diag,
                          nis_vel,
                          nis_valid,
                          nis_exceed_95,
                          radar_update_committed);
        }
        else
        {
          writeDvcDiagRow(most_recent_radar_scan_.header.stamp.toSec(),
                          radar_diag,
                          &cov_v_b_r_reve,
                          0,
                          runtime_reve_ms,
                          runtime_backend_ms,
                          radar_cov_recalib_cfg_.cov_mode,
                          cov_diag,
                          nis_vel,
                          nis_valid,
                          nis_exceed_95,
                          radar_update_committed);
        }
      }
    }
    else
    {
      runtime_reve_ms = (ros::WallTime::now().toSec() - t_reve_start) * 1000.0;
      last_reve_ms_ = runtime_reve_ms;
      ROS_INFO_STREAM("[radarScanCallback]: Ego velocity failed");
      writeDvcDiagRow(most_recent_radar_scan_.header.stamp.toSec(),
                      radar_diag,
                      nullptr,
                      0,
                      runtime_reve_ms,
                      runtime_backend_ms,
                      radar_cov_recalib_cfg_.cov_mode,
                      cov_diag,
                      nis_vel,
                      nis_valid,
                      nis_exceed_95,
                      radar_update_committed);
    }
    if (scheduler_mode_ == SchedulerMode::Legacy && scheduler_cfg_.enable_diag)
    {
      SensorEvent diag_event;
      diag_event.type              = EventType::RadarScan;
      diag_event.timestamp         = most_recent_radar_scan_.header.stamp.toSec();
      diag_event.enqueue_wall_time = active_radar_enqueue_wall_time_;
      const double latency_ms =
          std::isfinite(active_radar_enqueue_wall_time_) ? (ros::WallTime::now().toSec() - active_radar_enqueue_wall_time_) * 1000.0
                                                         : std::numeric_limits<double>::quiet_NaN();
      writeSchedulerDiagRow(diag_event,
                            0,
                            latency_ms,
                            0.0,
                            0.0,
                            0.0,
                            last_radar_enqueue_to_commit_ms_,
                            last_radar_starved_);
    }
    most_recent_radar_scan_.header.stamp = ros::TIME_MIN;
    active_radar_enqueue_wall_time_      = std::numeric_limits<double>::quiet_NaN();
  }

  /** \brief ROS service handler for resetting the filter.
   */
  bool resetServiceCallback(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/)
  {
    requestReset();
    return true;
  }

  /** \brief ROS service handler for resetting the filter to a given pose.
   */
  bool resetToPoseServiceCallback(rovio::SrvResetToPose::Request& request,
                                  rovio::SrvResetToPose::Response& /*response*/)
  {
    V3D WrWM(request.T_WM.position.x, request.T_WM.position.y, request.T_WM.position.z);
    QPD qWM(
        request.T_WM.orientation.w, request.T_WM.orientation.x, request.T_WM.orientation.y, request.T_WM.orientation.z);
    requestResetToPose(WrWM, qWM.inverted());
    return true;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The orientaetion is initialized using an accel. measurement.
   */
  void requestReset()
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::Reset;
      event.timestamp = ros::Time::now().toSec();
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (!init_state_.isInitialized())
    {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.state_ = FilterInitializationState::State::WaitForInitUsingAccel;
  }

  /** \brief Reset the filter when the next IMU measurement is received.
   *         The pose is initialized to the passed pose.
   *  @param WrWM - Position Vector, pointing from the World-Frame to the IMU-Frame, expressed in World-Coordinates.
   *  @param qMW  - Quaternion, expressing World-Frame in IMU-Coordinates (World Coordinates->IMU Coordinates)
   */
  void requestResetToPose(const V3D& WrWM, const QPD& qMW)
  {
    if (isEventMode() && !isWorkerThread())
    {
      SensorEvent event;
      event.type      = EventType::ResetToPose;
      event.timestamp = ros::Time::now().toSec();
      event.reset_WrWM = WrWM;
      event.reset_qMW  = qMW;
      enqueueEvent(event);
      return;
    }
    std::unique_lock<std::mutex> lock(m_filter_, std::defer_lock);
    if (!isEventMode() || !isWorkerThread())
      lock.lock();
    if (!init_state_.isInitialized())
    {
      std::cout << "Reinitialization already triggered. Ignoring request...";
      return;
    }

    init_state_.WrWM_  = WrWM;
    init_state_.qMW_   = qMW;
    init_state_.state_ = FilterInitializationState::State::WaitForInitExternalPose;
  }

  /** \brief Executes the update step of the filter and publishes the updated data.
   */
  void updateAndPublish()
  {
    if (init_state_.isInitialized())
    {
      // Execute the filter update.
      const double t1          = (double)cv::getTickCount();
      static double timing_T   = 0;
      static int timing_C      = 0;
      const double oldSafeTime = mpFilter_->safe_.t_;
      int c1                   = 0;
      int c2                   = 0;

      if (scheduler_mode_ == SchedulerMode::Legacy)
      {
        c1 = std::get<ROVIO_UPDATE_SOURCE>(mpFilter_->updateTimelineTuple_).measMap_.size();
        double lastImageTime;
        // select queue to trigger updates 0: img, 2: radar velocity
        if (std::get<ROVIO_UPDATE_SOURCE>(mpFilter_->updateTimelineTuple_).getLastTime(lastImageTime))
        {
          mpFilter_->updateSafe(&lastImageTime);
        }
        c2 = std::get<ROVIO_UPDATE_SOURCE>(mpFilter_->updateTimelineTuple_).measMap_.size();
      }
      else
      {
        mpFilter_->updateSafe();
      }

      const double t2 = (double)cv::getTickCount();
      timing_T += (t2 - t1) / cv::getTickFrequency() * 1000;
      timing_C += std::max(1, c1 - c2);
      bool plotTiming = false;
      if (plotTiming)
      {
        ROS_INFO_STREAM(" == Filter Update: " << (t2 - t1) / cv::getTickFrequency() * 1000 << " ms for processing "
                                              << c1 - c2 << " images, average: " << timing_T / timing_C);
      }
      if (mpFilter_->safe_.t_ > oldSafeTime)
      {  // Publish only if something changed
        publish_cycle_count_++;
        const bool run_pub_cycle = !isPerfMode() || shouldRunOnCycle(perf_cfg_.pub_decimation);
        const bool run_tf_cycle  = !isPerfMode() || shouldRunOnCycle(perf_cfg_.tf_decimation);
        const double t_publish_begin = ros::WallTime::now().toSec();
        for (int i = 0; i < mtState::nCam_; i++)
        {
          if (run_pub_cycle && !mpFilter_->safe_.img_[i].empty() && mpImgUpdate_->doFrameVisualisation_)
          {
            sensor_msgs::ImagePtr msg;
            std_msgs::Header header;
            header.stamp = ros::Time(mpFilter_->safe_.t_);
            msg          = cv_bridge::CvImage(header, "bgr8", mpFilter_->safe_.img_[i]).toImageMsg();
            image_publisher_.publish(*msg);

            //            cv::imshow("Tracker" + std::to_string(i), mpFilter_->safe_.img_[i]);
            //          cv::waitKey(3);
          }
        }
        if (run_pub_cycle && !mpFilter_->safe_.patchDrawing_.empty() && mpImgUpdate_->visualizePatches_)
        {
          cv::imshow("Patches", mpFilter_->safe_.patchDrawing_);
          cv::waitKey(3);
        }

        // Obtain the save filter state.
        mtFilterState& filterState = mpFilter_->safe_;
        mtState& state             = mpFilter_->safe_.state_;
        state.updateMultiCameraExtrinsics(&mpFilter_->multiCamera_);
        MXD& cov = mpFilter_->safe_.cov_;
        imuOutputCT_.transformState(state, imuOutput_);

        // Cout verbose for pose measurements
        if (mpImgUpdate_->verbose_)
        {
          if (mpPoseUpdate_->inertialPoseIndex_ >= 0)
          {
            std::cout << "Transformation between inertial frames, IrIW, qWI: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->inertialPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->inertialPoseIndex_) << std::endl;
          }
          if (mpPoseUpdate_->bodyPoseIndex_ >= 0)
          {
            std::cout << "Transformation between body frames, MrMV, qVM: " << std::endl;
            std::cout << "  " << state.poseLin(mpPoseUpdate_->bodyPoseIndex_).transpose() << std::endl;
            std::cout << "  " << state.poseRot(mpPoseUpdate_->bodyPoseIndex_) << std::endl;
          }
        }

        // Send Map (Pose Sensor, I) to World (rovio-intern, W) transformation
        if (run_tf_cycle && mpPoseUpdate_->inertialPoseIndex_ >= 0)
        {
          Eigen::Vector3d IrIW = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
          QPD qWI              = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
          tf::StampedTransform tf_transform_WI;
          tf_transform_WI.frame_id_       = map_frame_;
          tf_transform_WI.child_frame_id_ = world_frame_;
          tf_transform_WI.stamp_          = ros::Time(mpFilter_->safe_.t_);
          tf_transform_WI.setOrigin(tf::Vector3(IrIW(0), IrIW(1), IrIW(2)));
          tf_transform_WI.setRotation(tf::Quaternion(qWI.x(), qWI.y(), qWI.z(), -qWI.w()));
          tb_.sendTransform(tf_transform_WI);
        }

        // Send IMU pose.
        if (run_tf_cycle)
        {
          tf::StampedTransform tf_transform_MW;
          tf_transform_MW.frame_id_       = world_frame_;
          tf_transform_MW.child_frame_id_ = imu_frame_;
          tf_transform_MW.stamp_          = ros::Time(mpFilter_->safe_.t_);
          tf_transform_MW.setOrigin(tf::Vector3(imuOutput_.WrWB()(0), imuOutput_.WrWB()(1), imuOutput_.WrWB()(2)));
          tf_transform_MW.setRotation(
              tf::Quaternion(imuOutput_.qBW().x(), imuOutput_.qBW().y(), imuOutput_.qBW().z(), -imuOutput_.qBW().w()));
          tb_.sendTransform(tf_transform_MW);
        }

        // Send camera pose.
        if (run_tf_cycle)
        {
          for (int camID = 0; camID < mtState::nCam_; camID++)
          {
            tf::StampedTransform tf_transform_CM;
            tf_transform_CM.frame_id_       = imu_frame_;
            tf_transform_CM.child_frame_id_ = camera_frame_ + std::to_string(camID);
            tf_transform_CM.stamp_          = ros::Time(mpFilter_->safe_.t_);
            tf_transform_CM.setOrigin(tf::Vector3(state.MrMC(camID)(0), state.MrMC(camID)(1), state.MrMC(camID)(2)));
            tf_transform_CM.setRotation(tf::Quaternion(
                state.qCM(camID).x(), state.qCM(camID).y(), state.qCM(camID).z(), -state.qCM(camID).w()));
            tb_.sendTransform(tf_transform_CM);
          }
        }

        // Publish Odometry
        if (pubOdometry_.getNumSubscribers() > 0 || forceOdometryPublishing_)
        {
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state, cov, imuOutputCov_);

          odometryMsg_.header.seq              = msgSeq_;
          odometryMsg_.header.stamp            = ros::Time(mpFilter_->safe_.t_);
          odometryMsg_.pose.pose.position.x    = imuOutput_.WrWB()(0);
          odometryMsg_.pose.pose.position.y    = imuOutput_.WrWB()(1);
          odometryMsg_.pose.pose.position.z    = imuOutput_.WrWB()(2);
          odometryMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          odometryMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          odometryMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          odometryMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();
          for (unsigned int i = 0; i < 6; i++)
          {
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>() + i;
            if (i >= 3)
              ind1 = mtOutput::template getId<mtOutput::_att>() + i - 3;
            for (unsigned int j = 0; j < 6; j++)
            {
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>() + j;
              if (j >= 3)
                ind2 = mtOutput::template getId<mtOutput::_att>() + j - 3;
              odometryMsg_.pose.covariance[j + 6 * i] = imuOutputCov_(ind1, ind2);
            }
          }
          odometryMsg_.twist.twist.linear.x  = imuOutput_.BvB()(0);
          odometryMsg_.twist.twist.linear.y  = imuOutput_.BvB()(1);
          odometryMsg_.twist.twist.linear.z  = imuOutput_.BvB()(2);
          odometryMsg_.twist.twist.angular.x = imuOutput_.BwWB()(0);
          odometryMsg_.twist.twist.angular.y = imuOutput_.BwWB()(1);
          odometryMsg_.twist.twist.angular.z = imuOutput_.BwWB()(2);
          for (unsigned int i = 0; i < 6; i++)
          {
            unsigned int ind1 = mtOutput::template getId<mtOutput::_vel>() + i;
            if (i >= 3)
              ind1 = mtOutput::template getId<mtOutput::_ror>() + i - 3;
            for (unsigned int j = 0; j < 6; j++)
            {
              unsigned int ind2 = mtOutput::template getId<mtOutput::_vel>() + j;
              if (j >= 3)
                ind2 = mtOutput::template getId<mtOutput::_ror>() + j - 3;
              odometryMsg_.twist.covariance[j + 6 * i] = imuOutputCov_(ind1, ind2);
            }
          }
          pubOdometry_.publish(odometryMsg_);
        }

        if (pubPoseWithCovStamped_.getNumSubscribers() > 0 || forcePoseWithCovariancePublishing_)
        {
          // Compute covariance of output
          imuOutputCT_.transformCovMat(state, cov, imuOutputCov_);

          estimatedPoseWithCovarianceStampedMsg_.header.seq              = msgSeq_;
          estimatedPoseWithCovarianceStampedMsg_.header.stamp            = ros::Time(mpFilter_->safe_.t_);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.x    = imuOutput_.WrWB()(0);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.y    = imuOutput_.WrWB()(1);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.position.z    = imuOutput_.WrWB()(2);
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.w = -imuOutput_.qBW().w();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.x = imuOutput_.qBW().x();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.y = imuOutput_.qBW().y();
          estimatedPoseWithCovarianceStampedMsg_.pose.pose.orientation.z = imuOutput_.qBW().z();

          for (unsigned int i = 0; i < 6; i++)
          {
            unsigned int ind1 = mtOutput::template getId<mtOutput::_pos>() + i;
            if (i >= 3)
              ind1 = mtOutput::template getId<mtOutput::_att>() + i - 3;
            for (unsigned int j = 0; j < 6; j++)
            {
              unsigned int ind2 = mtOutput::template getId<mtOutput::_pos>() + j;
              if (j >= 3)
                ind2 = mtOutput::template getId<mtOutput::_att>() + j - 3;
              estimatedPoseWithCovarianceStampedMsg_.pose.covariance[j + 6 * i] = imuOutputCov_(ind1, ind2);
            }
          }

          pubPoseWithCovStamped_.publish(estimatedPoseWithCovarianceStampedMsg_);
        }

        // Send IMU pose message.
        if (pubTransform_.getNumSubscribers() > 0 || forceTransformPublishing_)
        {
          transformMsg_.header.seq              = msgSeq_;
          transformMsg_.header.stamp            = ros::Time(mpFilter_->safe_.t_);
          transformMsg_.transform.translation.x = imuOutput_.WrWB()(0);
          transformMsg_.transform.translation.y = imuOutput_.WrWB()(1);
          transformMsg_.transform.translation.z = imuOutput_.WrWB()(2);
          transformMsg_.transform.rotation.x    = imuOutput_.qBW().x();
          transformMsg_.transform.rotation.y    = imuOutput_.qBW().y();
          transformMsg_.transform.rotation.z    = imuOutput_.qBW().z();
          transformMsg_.transform.rotation.w    = -imuOutput_.qBW().w();
          pubTransform_.publish(transformMsg_);
        }

        if (pub_T_J_W_transform.getNumSubscribers() > 0 || forceTransformPublishing_)
        {
          if (mpPoseUpdate_->inertialPoseIndex_ >= 0)
          {
            Eigen::Vector3d IrIW               = state.poseLin(mpPoseUpdate_->inertialPoseIndex_);
            QPD qWI                            = state.poseRot(mpPoseUpdate_->inertialPoseIndex_);
            T_J_W_Msg_.header.seq              = msgSeq_;
            T_J_W_Msg_.header.stamp            = ros::Time(mpFilter_->safe_.t_);
            T_J_W_Msg_.transform.translation.x = IrIW(0);
            T_J_W_Msg_.transform.translation.y = IrIW(1);
            T_J_W_Msg_.transform.translation.z = IrIW(2);
            T_J_W_Msg_.transform.rotation.x    = qWI.x();
            T_J_W_Msg_.transform.rotation.y    = qWI.y();
            T_J_W_Msg_.transform.rotation.z    = qWI.z();
            T_J_W_Msg_.transform.rotation.w    = -qWI.w();
            pub_T_J_W_transform.publish(T_J_W_Msg_);
          }
        }

        // Publish Extrinsics
        for (int camID = 0; camID < mtState::nCam_; camID++)
        {
          if (pubExtrinsics_[camID].getNumSubscribers() > 0 || forceExtrinsicsPublishing_)
          {
            extrinsicsMsg_[camID].header.seq              = msgSeq_;
            extrinsicsMsg_[camID].header.stamp            = ros::Time(mpFilter_->safe_.t_);
            extrinsicsMsg_[camID].pose.pose.position.x    = state.MrMC(camID)(0);
            extrinsicsMsg_[camID].pose.pose.position.y    = state.MrMC(camID)(1);
            extrinsicsMsg_[camID].pose.pose.position.z    = state.MrMC(camID)(2);
            extrinsicsMsg_[camID].pose.pose.orientation.x = state.qCM(camID).x();
            extrinsicsMsg_[camID].pose.pose.orientation.y = state.qCM(camID).y();
            extrinsicsMsg_[camID].pose.pose.orientation.z = state.qCM(camID).z();
            extrinsicsMsg_[camID].pose.pose.orientation.w = -state.qCM(camID).w();
            for (unsigned int i = 0; i < 6; i++)
            {
              unsigned int ind1 = mtState::template getId<mtState::_vep>(camID) + i;
              if (i >= 3)
                ind1 = mtState::template getId<mtState::_vea>(camID) + i - 3;
              for (unsigned int j = 0; j < 6; j++)
              {
                unsigned int ind2 = mtState::template getId<mtState::_vep>(camID) + j;
                if (j >= 3)
                  ind2 = mtState::template getId<mtState::_vea>(camID) + j - 3;
                extrinsicsMsg_[camID].pose.covariance[j + 6 * i] = cov(ind1, ind2);
              }
            }
            pubExtrinsics_[camID].publish(extrinsicsMsg_[camID]);
          }
        }

        // Publish IMU biases
        if (pubImuBias_.getNumSubscribers() > 0 || forceImuBiasPublishing_)
        {
          imuBiasMsg_.header.seq            = msgSeq_;
          imuBiasMsg_.header.stamp          = ros::Time(mpFilter_->safe_.t_);
          imuBiasMsg_.angular_velocity.x    = state.gyb()(0);
          imuBiasMsg_.angular_velocity.y    = state.gyb()(1);
          imuBiasMsg_.angular_velocity.z    = state.gyb()(2);
          imuBiasMsg_.linear_acceleration.x = state.acb()(0);
          imuBiasMsg_.linear_acceleration.y = state.acb()(1);
          imuBiasMsg_.linear_acceleration.z = state.acb()(2);
          for (int i = 0; i < 3; i++)
          {
            for (int j = 0; j < 3; j++)
            {
              imuBiasMsg_.angular_velocity_covariance[3 * i + j] =
                  cov(mtState::template getId<mtState::_gyb>() + i, mtState::template getId<mtState::_gyb>() + j);
            }
          }
          for (int i = 0; i < 3; i++)
          {
            for (int j = 0; j < 3; j++)
            {
              imuBiasMsg_.linear_acceleration_covariance[3 * i + j] =
                  cov(mtState::template getId<mtState::_acb>() + i, mtState::template getId<mtState::_acb>() + j);
            }
          }
          pubImuBias_.publish(imuBiasMsg_);
        }

        // PointCloud message.
        if (run_pub_cycle &&
            (pubPcl_.getNumSubscribers() > 0 || pubMarkers_.getNumSubscribers() > 0 || forcePclPublishing_ ||
             forceMarkersPublishing_))
        {
          pclMsg_.header.seq      = msgSeq_;
          pclMsg_.header.stamp    = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.header.seq   = msgSeq_;
          markerMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          markerMsg_.points.clear();
          float badPoint = std::numeric_limits<float>::quiet_NaN();  // Invalid point.
          int offset     = 0;

          FeatureDistance distance;
          double d, d_minus, d_plus;
          const double stretchFactor = 3;
          for (unsigned int i = 0; i < mtState::nMax_; i++, offset += pclMsg_.point_step)
          {
            if (filterState.fsm_.isValid_[i])
            {
              // Get 3D feature coordinates.
              int camID          = filterState.fsm_.features_[i].mpCoordinates_->camID_;
              distance           = state.dep(i);
              d                  = distance.getDistance();
              const double sigma = sqrt(
                  cov(mtState::template getId<mtState::_fea>(i) + 2, mtState::template getId<mtState::_fea>(i) + 2));
              distance.p_ -= stretchFactor * sigma;
              d_minus = distance.getDistance();
              if (d_minus > 1000)
                d_minus = 1000;
              if (d_minus < 0)
                d_minus = 0;
              distance.p_ += 2 * stretchFactor * sigma;
              d_plus = distance.getDistance();
              if (d_plus > 1000)
                d_plus = 1000;
              if (d_plus < 0)
                d_plus = 0;
              Eigen::Vector3d bearingVector = filterState.state_.CfP(i).get_nor().getVec();
              const Eigen::Vector3d CrCPm   = bearingVector * d_minus;
              const Eigen::Vector3d CrCPp   = bearingVector * d_plus;
              const Eigen::Vector3f MrMPm =
                  V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPm))
                      .cast<float>();
              const Eigen::Vector3f MrMPp =
                  V3D(mpFilter_->multiCamera_.BrBC_[camID] + mpFilter_->multiCamera_.qCB_[camID].inverseRotate(CrCPp))
                      .cast<float>();

              // Get human readable output
              transformFeatureOutputCT_.setFeatureID(i);
              transformFeatureOutputCT_.setOutputCameraID(filterState.fsm_.features_[i].mpCoordinates_->camID_);
              transformFeatureOutputCT_.transformState(state, featureOutput_);
              transformFeatureOutputCT_.transformCovMat(state, cov, featureOutputCov_);
              featureOutputReadableCT_.transformState(featureOutput_, featureOutputReadable_);
              featureOutputReadableCT_.transformCovMat(featureOutput_, featureOutputCov_, featureOutputReadableCov_);

              // Get landmark output
              landmarkOutputImuCT_.setFeatureID(i);
              landmarkOutputImuCT_.transformState(state, landmarkOutput_);
              landmarkOutputImuCT_.transformCovMat(state, cov, landmarkOutputCov_);
              const Eigen::Vector3f MrMP = landmarkOutput_.get<LandmarkOutput::_lmk>().template cast<float>();

              // Write feature id, camera id, and rgb
              uint8_t gray    = 255;
              uint32_t rgb    = (gray << 16) | (gray << 8) | gray;
              uint32_t status = filterState.fsm_.features_[i].mpStatistics_->status_[0];
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset],
                     &filterState.fsm_.features_[i].idx_,
                     sizeof(int));                                                               // id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[1].offset], &camID, sizeof(int));     // cam id
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[2].offset], &rgb, sizeof(uint32_t));  // rgb
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[3].offset], &status, sizeof(int));    // status

              // Write coordinates to pcl message.
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[4].offset], &MrMP[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[5].offset], &MrMP[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[6].offset], &MrMP[2], sizeof(float));  // z

              // Add feature bearing vector and distance
              const Eigen::Vector3f bearing = featureOutputReadable_.bea().template cast<float>();
              const float distance          = static_cast<float>(featureOutputReadable_.dis());
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[7].offset], &bearing[0], sizeof(float));  // x
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[8].offset], &bearing[1], sizeof(float));  // y
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[9].offset], &bearing[2], sizeof(float));  // z
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[10].offset], &distance, sizeof(float));   // d

              // Add the corresponding covariance (upper triangular)
              Eigen::Matrix3f cov_MrMP = landmarkOutputCov_.cast<float>();
              int mCounter             = 11;
              for (int row = 0; row < 3; row++)
              {
                for (int col = row; col < 3; col++)
                {
                  memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &cov_MrMP(row, col), sizeof(float));
                  mCounter++;
                }
              }

              // Add distance uncertainty
              const float distance_cov = static_cast<float>(featureOutputReadableCov_(3, 3));
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[mCounter].offset], &distance_cov, sizeof(float));

              // Line markers (Uncertainty rays).
              geometry_msgs::Point point_near_msg;
              geometry_msgs::Point point_far_msg;
              point_near_msg.x = float(CrCPp[0]);
              point_near_msg.y = float(CrCPp[1]);
              point_near_msg.z = float(CrCPp[2]);
              point_far_msg.x  = float(CrCPm[0]);
              point_far_msg.y  = float(CrCPm[1]);
              point_far_msg.z  = float(CrCPm[2]);
              markerMsg_.points.push_back(point_near_msg);
              markerMsg_.points.push_back(point_far_msg);
            }
            else
            {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&pclMsg_.data[offset + pclMsg_.fields[0].offset], &id, sizeof(int));  // id
              for (int j = 1; j < pclMsg_.fields.size(); j++)
              {
                memcpy(&pclMsg_.data[offset + pclMsg_.fields[j].offset], &badPoint, sizeof(float));
              }
            }
          }
          pubPcl_.publish(pclMsg_);
          pubMarkers_.publish(markerMsg_);
        }
        if (run_pub_cycle && (pubPatch_.getNumSubscribers() > 0 || forcePatchPublishing_))
        {
          patchMsg_.header.seq   = msgSeq_;
          patchMsg_.header.stamp = ros::Time(mpFilter_->safe_.t_);
          int offset             = 0;
          for (unsigned int i = 0; i < mtState::nMax_; i++, offset += patchMsg_.point_step)
          {
            if (filterState.fsm_.isValid_[i])
            {
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset],
                     &filterState.fsm_.features_[i].idx_,
                     sizeof(int));  // id
              // Add patch data
              for (int l = 0; l < mtState::nLevels_; l++)
              {
                for (int y = 0; y < mtState::patchSize_; y++)
                {
                  for (int x = 0; x < mtState::patchSize_; x++)
                  {
                    memcpy(
                        &patchMsg_
                             .data[offset + patchMsg_.fields[1].offset +
                                   (l * mtState::patchSize_ * mtState::patchSize_ + y * mtState::patchSize_ + x) * 4],
                        &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].patch_[y * mtState::patchSize_ +
                                                                                              x],
                        sizeof(float));  // Patch
                    memcpy(
                        &patchMsg_
                             .data[offset + patchMsg_.fields[2].offset +
                                   (l * mtState::patchSize_ * mtState::patchSize_ + y * mtState::patchSize_ + x) * 4],
                        &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dx_[y * mtState::patchSize_ + x],
                        sizeof(float));  // dx
                    memcpy(
                        &patchMsg_
                             .data[offset + patchMsg_.fields[3].offset +
                                   (l * mtState::patchSize_ * mtState::patchSize_ + y * mtState::patchSize_ + x) * 4],
                        &filterState.fsm_.features_[i].mpMultilevelPatch_->patches_[l].dy_[y * mtState::patchSize_ + x],
                        sizeof(float));  // dy
                    memcpy(
                        &patchMsg_
                             .data[offset + patchMsg_.fields[4].offset +
                                   (l * mtState::patchSize_ * mtState::patchSize_ + y * mtState::patchSize_ + x) * 4],
                        &filterState.mlpErrorLog_[i].patches_[l].patch_[y * mtState::patchSize_ + x],
                        sizeof(float));  // error
                  }
                }
              }
            }
            else
            {
              // If current feature is not valid copy NaN
              int id = -1;
              memcpy(&patchMsg_.data[offset + patchMsg_.fields[0].offset], &id, sizeof(int));  // id
            }
          }

          pubPatch_.publish(patchMsg_);
        }
        const double t_publish_end = ros::WallTime::now().toSec();
        last_publish_ms_ = std::max(0.0, (t_publish_end - t_publish_begin) * 1000.0);
        gotFirstMessages_ = true;
      }
    }
  }
};

}  // namespace rovio

#endif /* ROVIO_ROVIONODE_HPP_ */
