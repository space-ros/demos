/*
The `VehicleDustPrivate` is the main script which create a ignition gazebo node in oder to handle communication between the camera topics. 
A subsciber is created which subscribes to the `/model/car/cmd_vel` topic build published by the camera plugin in the gazebo world.
The image recieved is first converted into opencv format using the GzCVBridge script on top of which a custom post processing 
algorithm is applied using openCV in order to add the rain effect noise on the camera the frame. The rain augmented frame in them 
published on a seperate topic called `/model/car/link/dust_link/particle_emitter/emitter/cmd`.
*/

#include <gz/plugin/Register.hh>
#include <iostream>
#include <string>
#include <csignal>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "VehicleDust.hh"
#include <gz/msgs/particle_emitter.pb.h>


namespace vehicle_dust
{
  class VehicleDustPrivate
  {
    public: gz::transport::Node velDustNode;
    public: std::string vel_topic;
    public: std::string dust_topic;
    public: gz::transport::Node::Publisher dustPub;
    public: gz::msgs::Vector3d linear;
    public: double scale_rate = 0, rate = 0, linear_vel = 0;

    public: void publish_dust(double rate_calculated, double scale_rate_calculated);
    public: void cmd_vel_cb(const gz::msgs::Twist &_msg);
  };

  void VehicleDustPrivate::publish_dust(double rate_calculated, double scale_rate_calculated)
  { 


    gz::msgs::ParticleEmitter *msg = new gz::msgs::ParticleEmitter();
    gz::msgs::Float rate, scale_rate;

    rate.set_data(rate_calculated);
    scale_rate.set_data(scale_rate_calculated);

    msg->set_allocated_rate(&rate);
    msg->set_allocated_scale_rate(&scale_rate);

    this->dustPub.Publish(*msg);

  }

  /*
  Callback function for the image frame recieved by the camera plugin. The function initially sets the frame width and frame height 
  for further processing. The function recieves the final rain augmented frame by calling the `add_rain_effect` & `add_text` functions 
  and then finally outputs the frame using `publish_dust` function.
  */
  void VehicleDustPrivate::cmd_vel_cb(const gz::msgs::Twist &_msg)
  {
    this->linear = _msg.linear();
    this->linear_vel = abs(this->linear.x());

    if(this->linear_vel > MAX_VEL)
    {
      this->linear_vel = MAX_VEL;
    }

    this->scale_rate = this->linear_vel * SCALE_CONST;
    this->rate = this->linear_vel * RATE_CONST;

    // std::cout<< "Rate: "<< this->rate << " | ScaleRate: " << this->scale_rate << std::endl;

    this->publish_dust(this->rate, this->scale_rate);
  }

  VehicleDust::VehicleDust()
  : dataPtr(std::make_unique<VehicleDustPrivate>())
  {}

  VehicleDust::~VehicleDust()
  {}

  void VehicleDust::Configure(const gz::sim::Entity &,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &,
                  gz::sim::EventManager &)
    {
        std::string velTopic = "/model/car/cmd_vel";
        std::string emitterModel = "car";

        if(_sdf->HasElement("topic"))
          velTopic      = _sdf->Get<std::string>("topic");
        if(_sdf->HasElement("emitter_model"))
          emitterModel   = _sdf->Get<std::string>("emitter_model");

        std::string emitterTopic = "/model/"  + emitterModel + "/link/dust_link/particle_emitter/emitter/cmd";
        std::string velTopicProcessed = gz::transport::TopicUtils::AsValidTopic(velTopic);
        std::string emitterTopicProcessed = gz::transport::TopicUtils::AsValidTopic(emitterTopic);

        this->dataPtr->velDustNode.Subscribe(velTopicProcessed,  &VehicleDustPrivate::cmd_vel_cb, this->dataPtr.get());

        this->dataPtr->dustPub = this->dataPtr->velDustNode.Advertise<gz::msgs::ParticleEmitter>(emitterTopic);
    }
}

  GZ_ADD_PLUGIN(
      vehicle_dust::VehicleDust,
      gz::sim::System,
      vehicle_dust::VehicleDust::ISystemConfigure 
      )