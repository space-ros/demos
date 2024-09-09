// Copyright 2024 Element Robotics Pty Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <sdf/Light.hh>
#include <ignition/gazebo/components.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Actor.hh>
#include <ignition/transport/Node.hh>
#include <random>
#include <iostream>

#ifdef USE_IGNITION
namespace gazebo = ignition::gazebo;
#else
namespace gazebo = gz::sim;
#endif

class LunarSun : public gazebo::System,
         public gazebo::ISystemConfigure,
         public gazebo::ISystemUpdate {
  // Model entity
  gazebo::World world_{gazebo::kNullEntity};


  // Joint entities
  gazebo::Entity actorEntity, lightEntity;

  // Whether the system has been properly configured
  bool configured_{false};

public:
  void Configure(const gazebo::Entity& entity,
         const std::shared_ptr<const sdf::Element>& sdf,
         gazebo::EntityComponentManager& ecm,
         gazebo::EventManager& /*eventMgr*/) override {
  this->world_ = gazebo::World(entity);

  this->actorEntity =
    ecm.EntityByComponents(gazebo::components::Name("animated_sun"));

  LoadCSV("/home/spaceros-user/demos_ws/src/lunarsim_demo/lunar_sun_gz_plugin/horizons_az_el.csv");

  ignition::math::Pose3d startPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  // Position the light above the ground
  ecm.CreateComponent(this->actorEntity,
            gazebo::components::Pose(startPose));
  ecm.CreateComponent(this->actorEntity,
            gazebo::components::TrajectoryPose(
              this->trajectory[0]));
  configured_ = true;
  }

  void Update(const gazebo::UpdateInfo& info,
        gazebo::EntityComponentManager& ecm) override {
  // Get actor pose
  auto actor = gazebo::Actor(this->actorEntity);
  auto actorPosition = actor.WorldPose(ecm);


  // Get the current simulation time
  this->currentSimTime = info.simTime;

  // Calculate the time elapsed since the last update
  std::chrono::_V2::steady_clock::duration elapsedTime =
    this->currentSimTime - this->startWaypointTime;

  double secondElapsed =
    std::chrono::duration_cast<std::chrono::seconds>(elapsedTime).count();

  if (elapsedTime >= waypointDuration) {
    // Move to the next waypoint
    this->startWaypointTime = this->currentSimTime;
    this->trajectoryIndex++;
    if (this->trajectoryIndex >= this->trajectory.size()) {
    this->trajectoryIndex = 0;
    }
    // Set the actor's trajectory pose
    actor.SetTrajectoryPose(ecm, this->trajectory[this->trajectoryIndex]);
    ecm.SetChanged(this->actorEntity, gazebo::components::TrajectoryPose::typeId,
           gazebo::ComponentState::PeriodicChange);

    ignition::msgs::Light lightMsg;
    lightMsg.set_name("sunlight");
    lightMsg.set_type(ignition::msgs::Light::DIRECTIONAL); 

    // Set the pose in the light message
    // Directly set the position components
    ignition::msgs::Pose* poseLight2 = lightMsg.mutable_pose();
    poseLight2->mutable_position()->set_x(
      this->trajectory[this->trajectoryIndex].X());
    poseLight2->mutable_position()->set_y(
      this->trajectory[this->trajectoryIndex].Y());
    poseLight2->mutable_position()->set_z(
      this->trajectory[this->trajectoryIndex].Z());

    // Directly set the orientation components (quaternion)
    poseLight2->mutable_orientation()->set_w(1.0);  // No rotation
    poseLight2->mutable_orientation()->set_x(0.0);
    poseLight2->mutable_orientation()->set_y(0.0);
    poseLight2->mutable_orientation()->set_z(0.0);

    // Set light properties
    lightMsg.mutable_diffuse()->set_r(0.8);
    lightMsg.mutable_diffuse()->set_g(0.8);
    lightMsg.mutable_diffuse()->set_b(0.8);
    lightMsg.mutable_diffuse()->set_a(1.0);

    lightMsg.mutable_specular()->set_r(0.05);
    lightMsg.mutable_specular()->set_g(0.05);
    lightMsg.mutable_specular()->set_b(0.05);
    lightMsg.mutable_specular()->set_a(1.0);

    lightMsg.mutable_direction()->set_x(
      this->trajectory[this->trajectoryIndex].X());
    lightMsg.mutable_direction()->set_y(
      -this->trajectory[this->trajectoryIndex].Y());
    lightMsg.mutable_direction()->set_z(
      -this->trajectory[this->trajectoryIndex].Z());

    lightMsg.set_range(20);
    lightMsg.set_cast_shadows(true);
    lightMsg.set_intensity(200.0);

    // Publish the light message to the light config service
    bool result;
    ignition::msgs::Boolean res;
    ignition::transport::Node node;
    constexpr unsigned int timeout = 5000;
    bool executed = node.Request("/world/dem_heightmap/light_config", lightMsg,
                   timeout, res, result);

    ecm.SetChanged(this->lightEntity, gazebo::components::Pose::typeId,
           gazebo::ComponentState::PeriodicChange);
    ecm.SetChanged(this->lightEntity, gazebo::components::Light::typeId,
           gazebo::ComponentState::PeriodicChange);
  } else {
    return;
  }
  }

private:
  void LoadCSV(const std::string& filename) {
  double originx;
  double originy;
  double originz;
  std::ifstream file(filename);
  std::string line;

  if (std::getline(file, line)) {
    // Skip the first line
  }

  if (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string origTimeStr, origAzimuthStr, origElevationStr;
    std::getline(ss, origTimeStr, ',');
    std::getline(ss, origAzimuthStr, ',');
    std::getline(ss, origElevationStr, ',');
    double origAzimuth = std::stod(origAzimuthStr);
    double origElevation = std::stod(origElevationStr);
    originx = 10000 * cos(origAzimuth * M_PI / 180.0) *
        cos(origElevation * M_PI / 180.0);
    originy = 10000 * sin(origAzimuth * M_PI / 180.0) *
        cos(origElevation * M_PI / 180.0);
    originz = sin(origElevation * M_PI / 180.0);
  }

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string timeStr, azimuthStr, elevationStr;
    std::getline(ss, timeStr, ',');
    std::getline(ss, azimuthStr, ',');
    std::getline(ss, elevationStr, ',');
    double azimuth = std::stod(azimuthStr);
    double elevation = std::stod(elevationStr);

    // Convert azimuth and elevation to radians and then to a Pose
    ignition::math::Vector3d position(
      100000 * cos(azimuth * M_PI / 180.0) * cos(elevation * M_PI / 180.0),
      100000 * sin(azimuth * M_PI / 180.0) * cos(elevation * M_PI / 180.0),
      (100000 * sin(elevation * M_PI / 180.0)) - 10000);

    ignition::math::Pose3d pose(position, ignition::math::Quaterniond::Identity);
    this->trajectory.push_back(pose);
  }
  }

  std::vector<ignition::math::Pose3d> trajectory;
  int trajectoryIndex = 0;
  std::chrono::_V2::steady_clock::duration startWaypointTime =
    std::chrono::steady_clock::duration::zero();
  std::chrono::_V2::steady_clock::duration currentSimTime;
  std::chrono::_V2::steady_clock::duration waypointDuration =
    std::chrono::hours(1);
};

IGNITION_ADD_PLUGIN(LunarSun, gazebo::System, LunarSun::ISystemConfigure,
          LunarSun::ISystemUpdate)
