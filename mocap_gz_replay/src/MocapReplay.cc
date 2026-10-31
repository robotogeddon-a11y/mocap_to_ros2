// Gazebo system that replays recorded motion exactly (kinematically).
//
// Every physics step it overwrites the joint positions and the model's world
// pose with the latest command, so the robot follows the mocap 1:1 and cannot
// fall over. Commands arrive on Gazebo topics (bridged from ROS by ros_gz_bridge):
//   <joint_topic> gz.msgs.Model : joint names + axis1.position (from sensor_msgs/JointState)
//   <pose_topic>  gz.msgs.Pose  : world pose of the model's root link (pelvis)
//
// Usage (world SDF):
//   <plugin filename="MocapReplay" name="mocap_gz_replay::MocapReplay">
//     <model_name>g1</model_name>
//     <joint_topic>/joint_states</joint_topic>
//     <pose_topic>/root_pose</pose_topic>
//   </plugin>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/model.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace mocap_gz_replay
{
class MocapReplay : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    model_name_ = sdf->Get<std::string>("model_name", "g1").first;
    const auto joint_topic = sdf->Get<std::string>("joint_topic", "/joint_states").first;
    const auto pose_topic = sdf->Get<std::string>("pose_topic", "/root_pose").first;
    node_.Subscribe(joint_topic, &MocapReplay::OnJoints, this);
    node_.Subscribe(pose_topic, &MocapReplay::OnPose, this);
    gzmsg << "MocapReplay: model [" << model_name_ << "], joints on [" << joint_topic
          << "], pose on [" << pose_topic << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    if (model_ == gz::sim::kNullEntity) {
      // The robot is spawned after the world loads, so look it up lazily.
      model_ = ecm.EntityByComponents(gz::sim::components::Model(),
                                      gz::sim::components::Name(model_name_));
      if (model_ == gz::sim::kNullEntity) {
        return;
      }
    }
    gz::sim::Model model(model_);

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [name, position] : joint_cmd_) {
      auto it = joints_.find(name);
      if (it == joints_.end()) {
        it = joints_.emplace(name, model.JointByName(ecm, name)).first;
        if (it->second == gz::sim::kNullEntity) {
          gzwarn << "MocapReplay: model has no joint [" << name << "]\n";
        }
      }
      if (it->second != gz::sim::kNullEntity) {
        gz::sim::Joint joint(it->second);
        joint.ResetPosition(ecm, {position});
        joint.ResetVelocity(ecm, {0.0});
      }
    }
    if (pose_cmd_) {
      model.SetWorldPoseCmd(ecm, *pose_cmd_);
    }
  }

private:
  void OnJoints(const gz::msgs::Model & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & joint : msg.joint()) {
      joint_cmd_[joint.name()] = joint.axis1().position();
    }
  }

  void OnPose(const gz::msgs::Pose & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_cmd_ = gz::msgs::Convert(msg);
  }

  gz::transport::Node node_;
  std::string model_name_;
  gz::sim::Entity model_{gz::sim::kNullEntity};
  std::map<std::string, gz::sim::Entity> joints_;
  std::mutex mutex_;
  std::map<std::string, double> joint_cmd_;
  std::optional<gz::math::Pose3d> pose_cmd_;
};
}  // namespace mocap_gz_replay

GZ_ADD_PLUGIN(mocap_gz_replay::MocapReplay, gz::sim::System,
              mocap_gz_replay::MocapReplay::ISystemConfigure,
              mocap_gz_replay::MocapReplay::ISystemPreUpdate)
