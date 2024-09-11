/*
The `DroneDustPrivate` is the main script which create a ignition gazebo node in oder to handle communication between the camera topics. 
A subsciber is created which subscribes to the `/model/car/cmd_vel` topic build published by the camera plugin in the gazebo world.
The image recieved is first converted into opencv format using the GzCVBridge script on top of which a custom post processing 
algorithm is applied using openCV in order to add the rain effect noise on the camera the frame. The rain augmented frame in them 
published on a seperate topic called `/model/car/link/dust_link/particle_emitter/emitter/cmd`.
*/

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/msgs/particle_emitter.pb.h>

#include <iostream>
#include <string>
#include <csignal>

#include "DroneDust.hh"


namespace drone_dust
{
  class DroneDustPrivate
  {
    public: gz::transport::Node node;

    public: gz::transport::Node velDustNode;
    public: gz::transport::Node::Publisher dustPub;

    public: std::string robotName;
    public: std::string modelName;

    public: double actDist, minDist, densityConst, last_distance = 0;
    public: double robot_x = 0, robot_y = 0, robot_z = 0; 
    public: double world_x = 0, world_y = 0, world_z = 0; 
    public: double scaleConst = 0.01;
    public: double rateConst = 0.04;

    public: void publish_dust(double rate_calculated, double scale_rate_calculated);
    public: void PoseCb(const gz::msgs::Pose_V &_data);
  };

  void DroneDustPrivate::PoseCb(const gz::msgs::Pose_V &_data)
  {
    for (int i = 0; i < _data.pose().size(); i++) {
      // std::cout<<"Pose Name: "<< pose_name << std::endl;
      if ( _data.pose(i).name() == robotName)
      {
        robot_x = _data.pose(i).position().x();
        robot_y = _data.pose(i).position().y();
        robot_z = _data.pose(i).position().z();
        // std::cout << "Robot " << robotName << " Z: " << robot_z << "  index: " << i << std::endl;
       }
    }
    for (int i = 0; i < _data.pose().size(); i++) {
      std::string pose_name = _data.pose(i).name();
      if (pose_name.find(modelName) != std::string::npos)
      {
        world_x = _data.pose(i).position().x();
        world_y = _data.pose(i).position().y();
        world_z = _data.pose(i).position().z();
        // std::cout << "World Z: " << world_z << "  index: " << i << std::endl;
      }
    }
    double object_distance = robot_z - world_z;
    gzmsg << "Object Distance: " << object_distance << std::endl;

    if(object_distance < actDist && object_distance > minDist)
    {
      // std::cout << "World Z: " << world_z << std::endl;
      double rate = densityConst * (actDist/object_distance) * this->rateConst;
      double scale_rate = densityConst * (actDist/object_distance) * this->scaleConst;
      // std::cout << "Scale Rate " << scale_rate << std::endl;
      this->publish_dust(rate, scale_rate);
      // gzmsg << "Rate: " << rate << "   " << "Scale Rate: " << scale_rate << std::endl;
    }
    else
    {
      this->publish_dust(0, 0);
    }
    last_distance = object_distance;
  }

  void DroneDustPrivate::publish_dust(double rate_calculated, double scale_rate_calculated)
  { 

    gz::msgs::ParticleEmitter *msg = new gz::msgs::ParticleEmitter();
    gz::msgs::Float rate, scale_rate;

    rate.set_data(rate_calculated);
    scale_rate.set_data(scale_rate_calculated);

    msg->set_allocated_rate(&rate);
    msg->set_allocated_scale_rate(&scale_rate);

    this->dustPub.Publish(*msg);

  }

  DroneDust::DroneDust()
  : dataPtr(std::make_unique<DroneDustPrivate>())
  {}

  DroneDust::~DroneDust()
  {}

  void DroneDust::Configure(const gz::sim::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &_ecm,
                  gz::sim::EventManager &)
    {

        auto worldName = _ecm.Component<gz::sim::components::Name>(_entity);

        std::string poseInfoTopic = "/world/" + worldName->Data() + "/pose/info";

        if(_sdf->HasElement("activation_distance"))
          this->dataPtr->actDist = _sdf->Get<double>("activation_distance");
        if(_sdf->HasElement("min_distance"))
          this->dataPtr->minDist = _sdf->Get<double>("min_distance");
        if(_sdf->HasElement("density_constant"))
          this->dataPtr->densityConst = _sdf->Get<double>("density_constant");
        if(_sdf->HasElement("robot_name"))
          this->dataPtr->robotName = _sdf->Get<std::string>("robot_name");
        if(_sdf->HasElement("model_name"))
          this->dataPtr->modelName = _sdf->Get<std::string>("model_name");

        std::string emitterTopic = "/model/" + this->dataPtr->robotName +"/link/dust_link/particle_emitter/emitter/cmd";

        std::cout<< "Pose Topic: " << poseInfoTopic << std::endl;
        std::cout<< "Emitter Topic: " << emitterTopic << std::endl;
        std::cout<< "robot_name: " << this->dataPtr->robotName << std::endl;
        std::cout<< "model_name: " << this->dataPtr->modelName << std::endl;

        std::string poseTopicProcessed = gz::transport::TopicUtils::AsValidTopic(poseInfoTopic);
        std::string poseInfoTopicProcessed = gz::transport::TopicUtils::AsValidTopic(poseInfoTopic);
        
        this->dataPtr->node.Subscribe(poseInfoTopicProcessed, &DroneDustPrivate::PoseCb, this->dataPtr.get());

        this->dataPtr->dustPub = this->dataPtr->velDustNode.Advertise<gz::msgs::ParticleEmitter>(emitterTopic);
    }
}

  GZ_ADD_PLUGIN(
      drone_dust::DroneDust,
      gz::sim::System,
      drone_dust::DroneDust::ISystemConfigure 
      )