/*
 * Copyright (C) 2024 Naman Malik namanmalik0210@gmail.com
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
 *
 */

#include <gz/msgs/wind.pb.h>

#include <gz/rendering.hh>

#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/ParticleEmitter.hh>


#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParticleEmitter.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/rendering/Events.hh>

#include <gz/common/Image.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include "gz/gui/Plugin.hh"

#include <memory>

#include <mutex>
#include <string>
#include <math.h>
#include <cmath>
#include <tuple>

#include "DustManager.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace dust_manager
{

class DustManagerPrivate
{
    
    /// \brief Transport node
    public: gz::transport::Node node;

    /// \brief Dust Emitter publisher
    public: gz::transport::Node::Publisher dustPublisher;

    /// \brief Wind publisher
    public: gz::transport::Node::Publisher windPublisher;

    /// \brief Connection to the pre-render event
    public: common::ConnectionPtr sceneUpdateConn{nullptr};

    public: rendering::ParticleEmitterPtr emitterCurrentPtr{nullptr};

    /// \brief Pointer to EventManager
    public: EventManager *eventMgr{nullptr};

    /// \brief Map of dust models
    std::unordered_map<std::string, rendering::ParticleEmitterPtr> emitterMap;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr _scene{nullptr};

    /// \brief Visual whose color will be changed.
    public: rendering::VisualPtr ledVisual{nullptr};

    /// \brief Protect variables changed from transport and the user
    public: std::recursive_mutex mutex;

    /// \brief Name of topic for Wind Topic
    public: std::string windTopic{""};

    /// \brief List of dust topics publishing 
    public: QStringList dustModelList;

    /// \brief Name of the world
    public: std::string worldName{""};

    /// \brief Wind Msg Pointer
    public: gz::msgs::Wind *windMsg = new gz::msgs::Wind();
};

/////////////////////////////////////////////////
DustManager::DustManager(): dataPtr(std::make_unique<DustManagerPrivate>())
{

}
/////////////////////////////////////////////////
DustManager::~DustManager() = default;

/////////////////////////////////////////////////
void DustManager::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
    if (this->title.empty())
    {
        this->title = "Dust Control";
    }
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

bool DustManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the render thread, so it's safe to make
    // rendering calls here
    if (nullptr != this->dataPtr->_scene)
    {
        if (this->changeTexture)
        {
            std::shared_ptr<const gz::common::Image> imagePtr = std::make_shared<gz::common::Image>(this->image);
            std::cout << "Setting Texture 2" <<std::endl;
            this->material->SetTexture("New Image", imagePtr); // Throwing error here
            // this->material->SetTexture("/test_image.png"); // Simulation freezes here
            std::cout << "Changing Material" << std::endl;
            this->dataPtr->emitterCurrentPtr->SetMaterial(this->material);
            this->changeTexture = false;
        }
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void DustManager::Update(const gz::sim::UpdateInfo & /*_info*/,
                       gz::sim::EntityComponentManager &_ecm)
{
    if (!this->dataPtr->_scene)
    {
        this->dataPtr->_scene = rendering::sceneFromFirstRenderEngine();
    }
    // Get the name of the world
    if (this->dataPtr->worldName.empty())
    {
        _ecm.Each<components::World, components::Name>(
            [&](const Entity &,
                const components::World *,
                const components::Name *_name) -> bool
            {
            this->dataPtr->worldName = _name->Data();
            return false;
            });
        SetupWindTopic();
    }

    _ecm.Each<components::ParticleEmitter, components::Name>(
        [&](const Entity &,
            const components::ParticleEmitter *,
            const components::Name *) -> bool
    {
        FindEntityVisual(_ecm);
        return true;
    });
}

/////////////////////////////////////////////////
void DustManager::SetdustModelList(const QStringList &_dustModelList)
{
    this->dataPtr->dustModelList = _dustModelList;
}

///////////////////////////////////////////////////
QStringList DustManager::dustModelList() const
{
    return this->dataPtr->dustModelList;
}

//////////////////////////////////////////////////
void DustManager::FindEntityVisual(gz::sim::EntityComponentManager &_ecm)
{
  for (unsigned int i = 0; i < this->dataPtr->_scene->VisualCount(); ++i)
  {
    gz::rendering::VisualPtr visual = this->dataPtr->_scene->VisualByIndex(i);
    rendering::ParticleEmitterPtr emitterPtr = 
            std::dynamic_pointer_cast<gz::rendering::ParticleEmitter>(visual);

    if (emitterPtr)
    {   
        bool found = false;
        for (const auto &pair : this->dataPtr->emitterMap) {
            if (pair.second == emitterPtr) {
                found = true;
                break;
            }
        }
        if (!found) 
        {
            auto parentVisual = visual->Parent();
            gz::sim::Entity linkEntity = gz::sim::kNullEntity;

            if (parentVisual)
            {
                // Use parentVisual to get the associated link or model entity
                auto userDataOpt = parentVisual->UserData("gazebo-entity");

                if (std::holds_alternative<uint64_t>(userDataOpt))
                {
                    linkEntity = static_cast<gz::sim::Entity>(std::get<uint64_t>(userDataOpt));
                }
            }
            if (linkEntity != gz::sim::kNullEntity)
            {   
                gz::sim::Entity modelEntity = _ecm.ParentEntity(linkEntity);

                if (modelEntity != gz::sim::kNullEntity)
                {
                    const auto *modelName = _ecm.Component<gz::sim::components::Name>(modelEntity);
                    if (modelName)
                    {
                        const std::string& modelNameStr = modelName->Data();
                        this->dataPtr->emitterMap[modelNameStr] = emitterPtr;
                        this->dataPtr->dustModelList.push_back(QString::fromStdString(modelNameStr));
                        ConfigureEmitter(modelNameStr);
                        this->dustModelListChanged();
                    }
                }
            }
        }
    }
  }
}


/////////////////////////////////////////////////
void DustManager::SetupWindTopic()
{
    std::string windTopic = "/world/" + this->dataPtr->worldName + "/wind";
    this->dataPtr->windPublisher = this->dataPtr->node.Advertise<gz::msgs::Wind>(windTopic);
}

/////////////////////////////////////////////////
void DustManager::OnDustEmitterModel(const QString &_DustEmitterModel)
{
    // Mutex Guard
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

    std::string emitter_name = _DustEmitterModel.toStdString();

    if (!emitter_name.empty())
    {
        ConfigureEmitter(emitter_name);
    }
}

/////////////////////////////////////////////////
double DustManager::GetDensity(double _lifetime, double _rate, double _scalRate, gz::math::Vector3d _size)
{
    double density = _lifetime * _rate * _scalRate / (_size.X() * _size.Y() * _size.Z());
    return density;
}

/////////////////////////////////////////////////
QColor GzColorToQColor(const gz::math::v7::Color &gzColor)
{
    int red = static_cast<int>(gzColor.R() * 255);
    int green = static_cast<int>(gzColor.G() * 255);
    int blue = static_cast<int>(gzColor.B() * 255);
    int alpha = static_cast<int>(gzColor.A() * 255);

    gzmsg << "red: "<< red << " " << "green: "<< green << " " << "blue: "<< blue << " " << "Alpha: "<< alpha << std::endl;

    return QColor(red, green, blue, alpha);
}

/////////////////////////////////////////////////
void DustManager::ConfigureEmitter(std::string _emitter_name)
{
    gzmsg << "Selected Dust Model: " << _emitter_name << std::endl;
    this->dataPtr->emitterCurrentPtr = this->dataPtr->emitterMap[_emitter_name];

    if (this->dataPtr->emitterCurrentPtr)
    {
        // Getting all the paremeters from particle emitter and setting
        // them to the local variables
        this->lifetime = this->dataPtr->emitterCurrentPtr->Lifetime();
        this->rate = this->dataPtr->emitterCurrentPtr->Rate();
        this->scaleRate = this->dataPtr->emitterCurrentPtr->ScaleRate();
        this->size = this->dataPtr->emitterCurrentPtr->EmitterSize();
        this->pose = this->dataPtr->emitterCurrentPtr->LocalPose();
        this->material = this->dataPtr->emitterCurrentPtr->Material();
        std::string imagePath = material->Texture();

        if (!this->image.Load(imagePath))
        {
            gzmsg << "Image Loaded Successfully" << std::endl;
        }
        gz::math::Color avgColor = this->image.AvgColor();

        this->emitting = this->dataPtr->emitterCurrentPtr->Emitting();
        this->velocity = this->dataPtr->emitterCurrentPtr->MaxVelocity();
        this->density = GetDensity(lifetime, rate, scaleRate, size);
        this->sizeX = size.X();
        this->sizeY = size.Y();
        this->sizeZ = size.Z();
        this->poseX = pose.Pos().X();
        this->poseY = pose.Pos().Y();
        this->poseZ = pose.Pos().Z();
        this->angle = pose.Rot().Euler().Z();
        this->dustColor = GzColorToQColor(avgColor);

        emittingChanged();
        velocityChanged();
        densityChanged();
        sizeXChanged();
        sizeYChanged();
        sizeZChanged();
        poseXChanged();
        poseYChanged();
        poseZChanged();
        angleChanged();
        dustColorChanged();
    }
}

/////////////////////////////////////////////////
void DustManager::ToggleDust(bool _checked)
{
    if (this->dataPtr->emitterCurrentPtr)
    {
        if (_checked)
        {
            this->dataPtr->emitterCurrentPtr->SetEmitting(true);
            gzmsg << "Dust On" << std::endl;
        }
        else
        {
            this->dataPtr->emitterCurrentPtr->SetEmitting(false);
            gzmsg << "Dust Off" << std::endl;
        }
    }
    else
    {
        gzmsg << "Dust Emitter Not Set" << std::endl;
    }
}

////////////////////////////////////////////////
void DustManager::ToggleWind(bool _checked)
{
    gz::msgs::Boolean enable_wind;
    enable_wind.set_data(_checked);
    this->dataPtr->windMsg->set_enable_wind(_checked);

    if (_checked)
    {
        gzmsg << "Wind On" << std::endl;
    }
    else
    {
        gzmsg << "Wind Off" << std::endl;
    }

    SetWindVelocity(this->velocity, this->angle);
}

// ////////////////////////////////////////////////
void DustManager::OnDensityChanged(double _density)
{
    // Calcuating Lifetime and Rate using Density on set size
    double volume = this->sizeX * this->sizeY * this->sizeZ;
    double totalParticles = volume * _density;
    this->rate = std::sqrt(totalParticles);
    this->lifetime = totalParticles / (this->rate * this->scaleRate);
    this->dataPtr->emitterCurrentPtr->SetLifetime(this->lifetime);
    this->dataPtr->emitterCurrentPtr->SetRate(this->rate);
}

////////////////////////////////////////////////
void DustManager::SetWindVelocity(double _velocity, double _angle)
{
    // Setting Velocity of Wind
    gz::msgs::Vector3d *wind_linear_velocity = new gz::msgs::Vector3d();
    wind_linear_velocity->set_x(_velocity * cos(_angle));
    wind_linear_velocity->set_y(_velocity * sin(_angle));
    gzmsg << "Wind Velocity: " << _velocity << ", " << _angle << std::endl;
    this->dataPtr->windMsg->set_allocated_linear_velocity(wind_linear_velocity);
    gzmsg << "Publishing Wind" << std::endl;
    this->dataPtr->windPublisher.Publish(*(this->dataPtr->windMsg));
}

// ////////////////////////////////////////////////
void DustManager::OnVelocityChanged(double _velocity)
{
    // Setting Velocity of Particle Emitter
    this->velocity = _velocity;
    this->dataPtr->emitterCurrentPtr->SetVelocityRange(_velocity - 0.3, _velocity + 0.3);

    SetWindVelocity(this->velocity, this->angle);
}

// ////////////////////////////////////////////////
void DustManager::OnPoseChanged(double _x, double _y, double _z, double _angle)
{
    this->angle = _angle;

    SetWindVelocity(this->velocity, this->angle);

    this->dataPtr->emitterCurrentPtr->SetLocalPose({_x, _y, _z, 0, 0, _angle});
    gzmsg << "Pose Changed: " << _x << ", " << _y << ", " << _z << std::endl;
}

// ////////////////////////////////////////////////
void DustManager::OnSizeChanged(double _x, double _y, double _z)
{
    this->dataPtr->emitterCurrentPtr->SetEmitterSize({_x, _y, _z});
    gzmsg << "Size Changed: " << _x << ", " << _y << ", " << _z << std::endl;

    // Calculating new Rate and Scale rate to maintain the density
    double originalVolume = sizeX * sizeY * sizeZ;
    double newVolume = _x * _y * _z;
    double totalParticles = density * newVolume;
    double newRate = this->rate * (newVolume / originalVolume);
    double newScaleRate = totalParticles / (newRate * this->lifetime);

    this->dataPtr->emitterCurrentPtr->SetRate(newRate);
    this->dataPtr->emitterCurrentPtr->SetScaleRate(newScaleRate);
}

// ////////////////////////////////////////////////
void DustManager::OnColorChanged(const QColor _dustColor)
{
    // if (!this->image.Data().empty())
    // {
    //     gzmsg << " IMage date not found" << std::endl;
    //     return;
    // }
    
    gzmsg << "Changing Color" << std::endl;
    gzmsg << "Got New Colors: " << _dustColor.red() << ", " << _dustColor.green() << ", " << _dustColor.blue() << std::endl;

    this->dustColor = _dustColor;
    dustColorChanged();

    auto data = this->image.Data();

    for (size_t y = 0; y < this->image.Height(); ++y)
    {
        for (size_t x = 0; x < this->image.Width(); ++x)
        {
            // Get the index of the current pixel in the data array
            size_t index = (y * this->image.Width() + x) * 4;

            // Set the red, green, and blue values
            data[index + 0] = _dustColor.red();   // Red
            data[index + 1] = _dustColor.green(); // Green
            data[index + 2] = _dustColor.blue();  // Blue

            this->image.SetFromData(data.data(), this->image.Width(), this->image.Height(), this->image.PixelFormat());

            this->changeTexture = true;
        }
    }

}

}

// Register this plugin
GZ_ADD_PLUGIN(dust_manager::DustManager,
              gz::gui::Plugin)
