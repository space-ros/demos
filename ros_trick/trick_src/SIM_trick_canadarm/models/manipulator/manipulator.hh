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


#ifndef __MANIPULATOR_HH_
#define __MANIPULATOR_HH_
/**************************************************************************
PURPOSE: (2D Manipulator class definitions including kinematics and control)
***************************************************************************/
#define TRICK_NO_MONTE_CARLO
#define TRICK_NO_MASTERSLAVE
#define TRICK_NO_INSTRUMENTATION

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cmath>
#include <iostream>
#include <new>

class Manip
{
public:
  Manip();

  /**
   * @brief Calculate Q_dotdot (joint accelerations) given current state and input torques
   * The dynamics is calculated using Articulated Body Algorithm (ABA) with RBDL library
   * Friction is modelled with Stribeck model taken from Mathworks docs
   * https://www.mathworks.com/help/simscape/ref/rotationalfriction.html
   * @return int status code
   */
  int forwardDynamics();

  /**
   * @brief Calculate state derivative. Calls forwardDynamics().
   *
   * @return int status code
   */
  int stateDeriv();
  /**
   * @brief Propagate state for the current timestamp by integration. Called after stateDeriv()
   *
   * @return int status code
   */
  int stateInteg();

  /**
   * @brief Initializes all data structures. Called on startup.
   *
   * @return int status code
   */
  int defaultData();

  /**
   * @brief Model encapsulating kinematics and dynamics of Canadarm with RBDL library
   *
   */
  RigidBodyDynamics::Model rbdl_model; /* ** -- class from RBDL that calculates forward dynamics */
  static const size_t NDOF = 7;        /* ** -- ndof */

  double input_torque[NDOF] = { 0.0 }; /* *i (N.m) input (commanded) torque for each joint */

  double q[NDOF] = { 0.0 };               /* *o rad angle of joints */
  double q_dot[NDOF] = { 0.0 };           /* *o (rad/s) velocity of joints */
  double q_dotdot[NDOF] = { 0.0 };        /* *o (rad/s^2) accelerations of joints */
  double friction_torque[NDOF] = { 0.0 }; /* *o (N.m) Torque comming from friction */
  double applied_torque[NDOF] = { 0.0 };  /* *o (N.m) final torque applied for each joint */

  // Friction-related parameters. For now we assume same params for every joint
  // The friction is modelled using Stribeck function
  // https://www.mathworks.com/help/simscape/ref/rotationalfriction.html
  double breakaway_friction_torque[NDOF];    /* *i (N.m)  */
  double coloumb_friction_torque[NDOF];      /* *i (N.m)  */
  double breakaway_friction_velocity[NDOF];  /* *i (rad/s)  */
  double coulomb_velocity_threshold[NDOF];   /* *i (rad.s)  */
  double stribeck_velocity_threshold[NDOF];  /* *i (N.m/rad.s)  */
  double viscous_friction_coefficient[NDOF]; /* *i (N.m/rad.s)  */

private:
  RigidBodyDynamics::Math::VectorNd q_vec_;
  RigidBodyDynamics::Math::VectorNd q_dotvec_;
  RigidBodyDynamics::Math::VectorNd q_ddotvec_;
  RigidBodyDynamics::Math::VectorNd torque_vec_;
};
#endif
