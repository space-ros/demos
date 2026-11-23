/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "helicopter_flight_control.hh"

#include <algorithm>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/msgs/wrench.pb.h>  // Include for wrench messages
#include <gz/msgs/double.pb.h>  // Include for double messages
#include <gz/transport/Node.hh> // Include for Ignition Transport

using namespace gz;
using namespace gz::sim;
using namespace systems;

// Define private data for HelicopterControl plugin
class ignition::gazebo::systems::HelicopterControlPrivate
{
public:
  void Load(const EntityComponentManager &_ecm,
            const sdf::ElementPtr &_sdf);

  /// \brief Update function to compute lift forces and apply control
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
public:
  void Update(EntityComponentManager &_ecm);

  /// \brief Model interface
public:
  Model model{kNullEntity};

  /// \brief PID parameters for altitude control
public:
  double Kp_alt = 0.1, Ki_alt = 0.01, Kd_alt = 0.05;

  /// \brief Initial desired altitude
public:
  double desired_altitude = 10.0;

  /// \brief Current altitude and velocity
public:
  double current_altitude = 0.0;
  double current_velocity_alt = 0.0;

  /// \brief PID state variables
public:
  double integral_alt = 0.0;
  double previous_error_alt = 0.0;

  /// \brief Link entity targeted by this plugin
public:
  Entity linkEntity;

  /// \brief Initialization flag
public:
  bool initialized{false};

  /// \brief Valid configuration flag
public:
  bool validConfig{false};

  /// \brief SDF configuration
public:
  sdf::ElementPtr sdfConfig;

  /// \brief Ignition Transport node for publishing rotor speed data
public:
  gz::transport::Node node;

  /// \brief Publisher for the angle of attack control
public:
  gz::transport::Node::Publisher angleOfAttackPub;

private:
  /// \brief Mutex to protect access to the desired altitude variable
  std::mutex desiredAltitudeMutex;

public:
  /// \brief Callback function for the angle of attack subscription
  void OnDesiredAltitudeMsg(const gz::msgs::Double &_msg)
  {
    std::lock_guard<std::mutex> lock(this->desiredAltitudeMutex);
    ignmsg << "Received desired altitude message: " << _msg.data() << std::endl;
    this->desired_altitude = _msg.data();
  }
};

//////////////////////////////////////////////////
void HelicopterControlPrivate::Load(const EntityComponentManager &_ecm,
                                    const sdf::ElementPtr &_sdf)
{

  this->angleOfAttackPub = this->node.Advertise<gz::msgs::Double>("/angle_of_attack");
  this->node.Subscribe("/desired_altitude", &HelicopterControlPrivate::OnDesiredAltitudeMsg, this);

  this->Kp_alt = _sdf->Get<double>("kp_alt", this->Kp_alt).first;
  this->Ki_alt = _sdf->Get<double>("ki_alt", this->Ki_alt).first;
  this->Kd_alt = _sdf->Get<double>("kd_alt", this->Kd_alt).first;
  this->desired_altitude = _sdf->Get<double>("desired_altitude", this->desired_altitude).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities = entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      ignerr << "Link with name[" << linkName << "] not found. "
             << "The HelicopterControl will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      ignwarn << "Multiple link entities with name[" << linkName << "] found. "
              << "Using the first one.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity, components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      ignerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    ignerr << "The HelicopterControl system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
HelicopterControl::HelicopterControl()
    : System(), dataPtr(std::make_unique<HelicopterControlPrivate>())
{
}

//////////////////////////////////////////////////
void HelicopterControlPrivate::Update(EntityComponentManager &_ecm)
{
  IGN_PROFILE("HelicopterControlPrivate::Update");

  // Get linear velocity and pose at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  if (!worldLinVel || !worldPose)
    return;

  const auto &vel = worldLinVel->Data();
  const auto &pose = worldPose->Data();

  this->current_velocity_alt = vel.Z();
  this->current_altitude = pose.Pos().Z();

  // Calculate altitude error
  double error_alt = this->desired_altitude - this->current_altitude;

  // PID control terms
  double Pout_alt = this->Kp_alt * error_alt;
  this->integral_alt += error_alt * 0.001; // Integrate error over time step (dt = 0.001)
  double Iout_alt = this->Ki_alt * this->integral_alt;
  double Dout_alt = this->Kd_alt * this->current_velocity_alt;

  // Compute control adjustment based  and PID
  double controlAdjustment = Pout_alt + Iout_alt - Dout_alt;

  // angle of attack range from 2deg to 8deg
  double angle_of_attack = std::max(0.0349066, std::min(controlAdjustment, 0.139626));

  // Publish angle of attack
  gz::msgs::Double angleOfAttackMsg;
  angleOfAttackMsg.set_data(angle_of_attack);
  this->angleOfAttackPub.Publish(angleOfAttackMsg);

  // igndbg << "Current Altitude: " << this->current_altitude
  //        << ", Desired Altitude: " << this->desired_altitude << std::endl;
}

//////////////////////////////////////////////////
void HelicopterControl::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "The HelicopterControl system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void HelicopterControl::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  IGN_PROFILE("HelicopterControl::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
            << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);
    }
  }

  if (_info.paused)
    return;

  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

IGNITION_ADD_PLUGIN(HelicopterControl,
                    System,
                    HelicopterControl::ISystemConfigure,
                    HelicopterControl::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(HelicopterControl, "gz::sim::systems::HelicopterControl")

// TODO: Deprecated, remove on version 8
IGNITION_ADD_PLUGIN_ALIAS(HelicopterControl, "ignition::gazebo::systems::HelicopterControl")
