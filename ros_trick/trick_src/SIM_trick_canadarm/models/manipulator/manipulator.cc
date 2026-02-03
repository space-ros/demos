// Copyright 2024 Blazej Fiderek (xfiderek)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "manipulator/manipulator.hh"
#include "trick/integrator_c_intf.h"

Manip::Manip() : rbdl_model()
{
  RigidBodyDynamics::Addons::URDFReadFromFile("./SSRMS_Canadarm2.urdf.xacro", &rbdl_model, false);
  std::cout << "Loaded URDF model into trick";
  std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(rbdl_model);

  rbdl_model.gravity = RigidBodyDynamics::Math::Vector3d::Zero();

  q_vec_ = RigidBodyDynamics::Math::VectorNd::Zero(rbdl_model.q_size);
  q_dotvec_ = RigidBodyDynamics::Math::VectorNd::Zero(rbdl_model.q_size);
  q_ddotvec_ = RigidBodyDynamics::Math::VectorNd::Zero(rbdl_model.q_size);
  torque_vec_ = RigidBodyDynamics::Math::VectorNd::Zero(rbdl_model.q_size);

  std::cout << "starting Trick Canadarm simulation" << std::endl;
}

int Manip::defaultData()
{
  for (size_t i = 0; i < NDOF; ++i)
  {
    q[i] = 0.0;
    q_dot[i] = 0.0;
    q_dotdot[i] = 0.0;
    input_torque[i] = 0.0;
    applied_torque[i] = 0.0;
    friction_torque[i] = 0.0;
    q_vec_[i] = 0.0;
    q_dotvec_[i] = 0.0;
    q_ddotvec_[i] = 0.0;
    torque_vec_[i] = 0.0;
    breakaway_friction_torque[i] = 2.5;
    coloumb_friction_torque[i] = 2.0;
    breakaway_friction_velocity[i] = 0.005;
    coulomb_velocity_threshold[i] = breakaway_friction_velocity[i] / 10;
    stribeck_velocity_threshold[i] = M_SQRT2 * breakaway_friction_torque[i];
    viscous_friction_coefficient[i] = 0.001;
  }

  return (0);
}

int Manip::forwardDynamics()
{
  // calculate torques and store as RBDL vector as well
  for (size_t i = 0; i < NDOF; ++i)
  {
    // https://www.mathworks.com/help/simscape/ref/rotationalfriction.html
    double curr_to_stribeck_vel = q_dot[i] / stribeck_velocity_threshold[i];
    double curr_to_coulomb_vel = q_dot[i] / coulomb_velocity_threshold[i];
    friction_torque[i] = M_SQRT2 * M_E * (breakaway_friction_torque[i] - coloumb_friction_torque[i]) *
                         std::exp(-std::pow(curr_to_stribeck_vel, 2)) * curr_to_stribeck_vel;
    friction_torque[i] += coloumb_friction_torque[i] * std::tanh(curr_to_coulomb_vel);
    friction_torque[i] += viscous_friction_coefficient[i] * q_dot[i];

    applied_torque[i] = input_torque[i] - friction_torque[i];

    //  copy positions and velocities from C-arrays to RBDL vectors
    torque_vec_[i] = applied_torque[i];
    q_vec_[i] = q[i];
    q_dotvec_[i] = q_dot[i];
    q_ddotvec_[i] = 0.0;
  }
  // calculate dynamics (q_dotdot) with RBDL
  RigidBodyDynamics::ForwardDynamics(rbdl_model, q_vec_, q_dotvec_, torque_vec_, q_ddotvec_);

  // copy Q_dotdot from RBDL vector type to simple C-arrays
  for (size_t i = 0; i < NDOF; ++i)
  {
    q_dotdot[i] = q_ddotvec_[i];
  }

  return (0);
}

int Manip::stateDeriv()
{
  int status_code = forwardDynamics();
  return (status_code);
}

int Manip::stateInteg()
{
  int integration_step;
  load_state(&q[0], &q[1], &q[2], &q[3], &q[4], &q[5], &q[6], &q_dot[0], &q_dot[1], &q_dot[2], &q_dot[3], &q_dot[4],
             &q_dot[5], &q_dot[6], NULL);
  load_deriv(&q_dot[0], &q_dot[1], &q_dot[2], &q_dot[3], &q_dot[4], &q_dot[5], &q_dot[6], &q_dotdot[0], &q_dotdot[1],
             &q_dotdot[2], &q_dotdot[3], &q_dotdot[4], &q_dotdot[5], &q_dotdot[6], NULL);
  // integrate
  integration_step = integrate();

  unload_state(&q[0], &q[1], &q[2], &q[3], &q[4], &q[5], &q[6], &q_dot[0], &q_dot[1], &q_dot[2], &q_dot[3], &q_dot[4],
               &q_dot[5], &q_dot[6], NULL);

  return (integration_step);
}
