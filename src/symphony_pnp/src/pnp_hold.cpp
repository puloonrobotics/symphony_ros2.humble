#include <atomic>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/msgs/empty.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace pnp_hold {

class PnpHold
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity & entity,
                 const std::shared_ptr<const sdf::Element> & sdf,
                 ignition::gazebo::EntityComponentManager & ecm,
                 ignition::gazebo::EventManager &) override {
    model_ = ignition::gazebo::Model(entity);
    parent_name_ = sdf->Get<std::string>("parent_link");
    child_model_ = sdf->Get<std::string>("child_model");
    child_link_ = sdf->Get<std::string>("child_link");
    parent_ = model_.LinkByName(ecm, parent_name_);
    node_.Subscribe(sdf->Get<std::string>("attach_topic"), &PnpHold::OnAttach, this);
    node_.Subscribe(sdf->Get<std::string>("detach_topic"), &PnpHold::OnDetach, this);
    ignmsg << "pnp hold ready " << child_model_ << std::endl;
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &,
                 ignition::gazebo::EntityComponentManager & ecm) override {
    if (attach_.exchange(false) && joint_ == ignition::gazebo::kNullEntity) {
      ResolveChild(ecm);
      if (parent_ == ignition::gazebo::kNullEntity || child_ == ignition::gazebo::kNullEntity) {
        ignwarn << "pnp hold missed links for " << child_model_ << std::endl;
        return;
      }
      ParkCollisions(ecm, true);
      joint_ = ecm.CreateEntity();
      ignition::gazebo::components::DetachableJointInfo info;
      info.parentLink = parent_;
      info.childLink = child_;
      info.jointType = "fixed";
      ecm.CreateComponent(joint_, ignition::gazebo::components::DetachableJoint(info));
      ignmsg << "pnp hold attached " << child_model_ << std::endl;
    }
    if (detach_.exchange(false) && joint_ != ignition::gazebo::kNullEntity) {
      ecm.RequestRemoveEntity(joint_);
      joint_ = ignition::gazebo::kNullEntity;
      ParkCollisions(ecm, false);
      ZeroVelocity(ecm, child_);
      ignmsg << "pnp hold detached " << child_model_ << std::endl;
    }
  }

 private:
  void OnAttach(const ignition::msgs::Empty &) { attach_ = true; }
  void OnDetach(const ignition::msgs::Empty &) { detach_ = true; }

  void ResolveChild(ignition::gazebo::EntityComponentManager & ecm) {
    child_ = ignition::gazebo::kNullEntity;
    for (const auto entity : ignition::gazebo::entitiesFromScopedName(child_model_, ecm)) {
      if (!ecm.EntityHasComponentType(entity, ignition::gazebo::components::Model::typeId)) {
        continue;
      }
      const auto link = ecm.EntityByComponents(
        ignition::gazebo::components::Link(),
        ignition::gazebo::components::ParentEntity(entity),
        ignition::gazebo::components::Name(child_link_));
      if (link != ignition::gazebo::kNullEntity) {
        child_ = link;
        return;
      }
    }
  }

  void ParkCollisions(ignition::gazebo::EntityComponentManager & ecm, bool park) {
    if (park) {
      parked_.clear();
      ecm.Each<ignition::gazebo::components::Collision,
               ignition::gazebo::components::ParentEntity,
               ignition::gazebo::components::Pose>(
        [&](const ignition::gazebo::Entity & entity,
            const ignition::gazebo::components::Collision *,
            const ignition::gazebo::components::ParentEntity * parent,
            const ignition::gazebo::components::Pose * pose) {
          if (parent->Data() != child_) {
            return true;
          }
          parked_.push_back({entity, pose->Data()});
          ecm.Component<ignition::gazebo::components::Pose>(entity)->Data() =
            ignition::math::Pose3d(0, 0, -100, 0, 0, 0);
          ecm.SetChanged(entity, ignition::gazebo::components::Pose::typeId,
                         ignition::gazebo::ComponentState::OneTimeChange);
          return true;
        });
      return;
    }
    for (const auto & saved : parked_) {
      auto * pose = ecm.Component<ignition::gazebo::components::Pose>(saved.entity);
      if (pose != nullptr) {
        pose->Data() = saved.pose;
        ecm.SetChanged(saved.entity, ignition::gazebo::components::Pose::typeId,
                       ignition::gazebo::ComponentState::OneTimeChange);
      }
    }
    parked_.clear();
  }

  void ZeroVelocity(ignition::gazebo::EntityComponentManager & ecm,
                    ignition::gazebo::Entity link) {
    auto * linear = ecm.Component<ignition::gazebo::components::LinearVelocity>(link);
    if (linear != nullptr) {
      linear->Data() = ignition::math::Vector3d::Zero;
    }
    auto * angular = ecm.Component<ignition::gazebo::components::AngularVelocity>(link);
    if (angular != nullptr) {
      angular->Data() = ignition::math::Vector3d::Zero;
    }
  }

  ignition::gazebo::Model model_{ignition::gazebo::kNullEntity};
  ignition::transport::Node node_;
  std::string parent_name_;
  std::string child_model_;
  std::string child_link_;
  ignition::gazebo::Entity parent_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity child_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity joint_{ignition::gazebo::kNullEntity};
  std::atomic<bool> attach_{false};
  std::atomic<bool> detach_{false};
  struct SavedPose {
    ignition::gazebo::Entity entity;
    ignition::math::Pose3d pose;
  };
  std::vector<SavedPose> parked_;
};

}  // namespace pnp_hold

IGNITION_ADD_PLUGIN(
  pnp_hold::PnpHold,
  ignition::gazebo::System,
  pnp_hold::PnpHold::ISystemConfigure,
  pnp_hold::PnpHold::ISystemPreUpdate)
