#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
namespace
{
double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::string sdfString(const sdf::ElementPtr &sdf, const std::string &name, const std::string &fallback)
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}

double sdfDouble(const sdf::ElementPtr &sdf, const std::string &name, const double fallback)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}

int sdfInt(const sdf::ElementPtr &sdf, const std::string &name, const int fallback)
{
  return sdf->HasElement(name) ? sdf->Get<int>(name) : fallback;
}
}  // namespace

class ExampleTrackDrivePlugin : public ModelPlugin
{
  enum class TrackMotion
  {
    Straight,
    Arc
  };

public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    cmd_vel_topic_ = sdfString(sdf, "cmd_vel_topic", "/cmd_vel");
    sprocket_joint_name_ = sdfString(sdf, "sprocket_joint", "sprocket_axle");
    track_joint_prefix_ = sdfString(sdf, "track_joint_prefix", "track");
    sprocket_radius_ = sdfDouble(sdf, "sprocket_radius", 0.115);
    track_radius_ = sdfDouble(sdf, "track_radius", sprocket_radius_);
    track_pitch_ = sdfDouble(sdf, "track_pitch", 0.056);
    wheel_joint_count_ = sdfInt(sdf, "wheel_joint_count", 0);
    max_linear_velocity_ = sdfDouble(sdf, "max_linear_velocity", 0.6);
    max_angular_velocity_ = sdfDouble(sdf, "max_angular_velocity", 1.2);
    command_timeout_ = sdfDouble(sdf, "command_timeout", 0.5);

    if (sprocket_radius_ <= 0.0) {
      sprocket_radius_ = 0.115;
    }

    sprocket_joint_ = model_->GetJoint(sprocket_joint_name_);
    if (!sprocket_joint_) {
      gzwarn << "[ExampleTrackDrivePlugin] Joint [" << sprocket_joint_name_ << "] was not found.\n";
    }
    if (track_radius_ <= 0.0) {
      track_radius_ = sprocket_radius_;
    }
    if (track_pitch_ <= 0.0) {
      track_pitch_ = 0.056;
    }
    if (wheel_joint_count_ > 0) {
      for (int i = 0; i < wheel_joint_count_; ++i) {
        track_joints_.push_back(
          {model_->GetJoint(track_joint_prefix_ + "_wheel_joint" + std::to_string(i)), track_radius_,
           TrackMotion::Arc});
      }
    } else {
      track_joints_.push_back(
        {model_->GetJoint(track_joint_prefix_ + "_straight_segment_joint0"), 1.0, TrackMotion::Straight});
      track_joints_.push_back(
        {model_->GetJoint(track_joint_prefix_ + "_arc_segment_joint0"), track_radius_, TrackMotion::Arc});
      track_joints_.push_back(
        {model_->GetJoint(track_joint_prefix_ + "_straight_segment_joint1"), 1.0, TrackMotion::Straight});
      track_joints_.push_back(
        {model_->GetJoint(track_joint_prefix_ + "_arc_segment_joint1"), track_radius_, TrackMotion::Arc});
    }
    for (const auto &track_joint : track_joints_) {
      if (!track_joint.joint) {
        gzwarn << "[ExampleTrackDrivePlugin] Track joint with prefix [" << track_joint_prefix_
               << "] was not found; belt animation will be partial.\n";
      }
    }

    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
      owns_rclcpp_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>(model_->GetName() + "_example_track_drive");
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&ExampleTrackDrivePlugin::cmdVelCallback, this, std::placeholders::_1));
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ExampleTrackDrivePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[ExampleTrackDrivePlugin] Listening on [" << cmd_vel_topic_
          << "] for model [" << model_->GetName() << "]\n";
  }

  ~ExampleTrackDrivePlugin() override
  {
    update_connection_.reset();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (node_ && executor_) {
      executor_->remove_node(node_);
      node_.reset();
    }
    executor_.reset();
    if (owns_rclcpp_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    linear_velocity_ = clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    angular_velocity_ = clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    last_command_time_ = model_->GetWorld()->SimTime();
    received_command_ = true;
  }

  void onUpdate(const common::UpdateInfo &info)
  {
    if (!last_update_time_valid_) {
      last_update_time_ = info.simTime;
      last_update_time_valid_ = true;
    }
    const double dt = std::max(0.0, (info.simTime - last_update_time_).Double());
    last_update_time_ = info.simTime;

    double linear = 0.0;
    double angular = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (received_command_ && (info.simTime - last_command_time_).Double() <= command_timeout_) {
        linear = linear_velocity_;
        angular = angular_velocity_;
      }
    }

    if (sprocket_joint_) {
      const double sprocket_velocity = linear / sprocket_radius_;
      setJointVelocity(sprocket_joint_, sprocket_velocity);
    }

    (void)angular;
    belt_position_ += linear * dt;
    double wrapped_position = std::fmod(belt_position_, track_pitch_);
    if (wrapped_position < 0.0) {
      wrapped_position += track_pitch_;
    }
    wrapped_position -= 0.5 * track_pitch_;

    for (const auto &track_joint : track_joints_) {
      if (track_joint.joint) {
        double position = wrapped_position;
        if (track_joint.motion == TrackMotion::Arc) {
          constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
          position = std::fmod(belt_position_ / track_joint.joint_to_track, kTwoPi);
          if (position < 0.0) {
            position += kTwoPi;
          }
        }
        setJointVelocity(track_joint.joint, linear / track_joint.joint_to_track);
        setJointPosition(track_joint.joint, position);
      }
    }
  }

  void setJointPosition(const physics::JointPtr &joint, const double position)
  {
    joint->SetPosition(0, position, true);
  }

  void setJointVelocity(const physics::JointPtr &joint, const double velocity)
  {
    const double effort_limit = joint->GetEffortLimit(0);
    joint->SetParam(
      "fmax", 0, effort_limit > 0.0 ? effort_limit : std::numeric_limits<double>::max());
    joint->SetParam("vel", 0, velocity);
  }

  struct TrackJoint
  {
    physics::JointPtr joint;
    double joint_to_track;
    TrackMotion motion;
  };

  physics::ModelPtr model_;
  physics::JointPtr sprocket_joint_;
  std::vector<TrackJoint> track_joints_;
  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  bool owns_rclcpp_context_{false};

  std::mutex command_mutex_;
  common::Time last_command_time_{0, 0};
  common::Time last_update_time_{0, 0};
  bool last_update_time_valid_{false};
  bool received_command_{false};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  double belt_position_{0.0};

  std::string cmd_vel_topic_;
  std::string sprocket_joint_name_;
  std::string track_joint_prefix_;
  double sprocket_radius_{0.115};
  double track_radius_{0.115};
  double track_pitch_{0.056};
  int wheel_joint_count_{0};
  double max_linear_velocity_{0.6};
  double max_angular_velocity_{1.2};
  double command_timeout_{0.5};
};

GZ_REGISTER_MODEL_PLUGIN(ExampleTrackDrivePlugin)
}  // namespace gazebo
