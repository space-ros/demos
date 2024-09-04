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

#ifndef GZ_GUI_PLUGINS_DustManager_HH_
#define GZ_GUI_PLUGINS_DustManager_HH_

#include <memory>

#include "gz/gui/Plugin.hh"
#include <gz/sim/System.hh>
#include <gz/sim/gui/GuiSystem.hh>

#include <gz/rendering/Material.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/ParticleEmitter.hh>


using namespace gz;
using namespace sim;
using namespace systems;

namespace dust_manager
{
class DustManagerPrivate;

class DustManager : public gz::sim::GuiSystem
{
    Q_OBJECT

    /// \brief Constructor
    public: DustManager();

    /// \brief Destructor
    public: ~DustManager();

    /// \brief Q_Property function for changing the values in the GUI
    Q_PROPERTY(QStringList dustModelList READ dustModelList WRITE SetdustModelList NOTIFY dustModelListChanged)
    Q_PROPERTY(bool emitting MEMBER emitting NOTIFY emittingChanged)
    Q_PROPERTY(double velocity MEMBER velocity NOTIFY velocityChanged)
    Q_PROPERTY(double density MEMBER density NOTIFY densityChanged)
    Q_PROPERTY(double sizeX MEMBER sizeX NOTIFY sizeXChanged)
    Q_PROPERTY(double sizeY MEMBER sizeY NOTIFY sizeYChanged)
    Q_PROPERTY(double sizeZ MEMBER sizeZ NOTIFY sizeZChanged)
    Q_PROPERTY(double poseX MEMBER poseX NOTIFY poseXChanged)
    Q_PROPERTY(double poseY MEMBER poseY NOTIFY poseYChanged)
    Q_PROPERTY(double poseZ MEMBER poseZ NOTIFY poseZChanged)
    Q_PROPERTY(double angle MEMBER angle NOTIFY angleChanged)
    Q_PROPERTY(QColor dustColor MEMBER dustColor NOTIFY dustColorChanged)

    /// \brief List of Dust Model Parameters
    public: bool emitting = false;
    public: gz::math::Vector3d size;
    public: gz::math::Pose3d pose;
    public: double lifetime;
    public: double rate;
    public: double scaleRate;
    public: double density;
    public: double velocity;
    public: double poseX;
    public: double poseY;
    public: double poseZ;
    public: double sizeX;
    public: double sizeY;
    public: double sizeZ;
    public: double angle;
    public: QColor dustColor = QColor(255, 255, 255);
    public: QColor dustColorBorder = QColor(0, 0, 0);
    gz::common::Image image;
    rendering::MaterialPtr material;
    public: bool changeTexture = false;

    /// \brief Notify that Model list has changed
    signals: void dustModelListChanged();
    signals: void velocityChanged();
    signals: void densityChanged();
    signals: void emittingChanged();
    signals: void sizeXChanged();
    signals: void sizeYChanged();
    signals: void sizeZChanged();
    signals: void poseXChanged();
    signals: void poseYChanged();
    signals: void poseZChanged();
    signals: void angleChanged();
    signals: void dustColorChanged();

    /// \return List of models
    public: Q_INVOKABLE QStringList dustModelList() const;

    /// \brief Set the topic list from a string
    /// \param[in] dustModelList List of topics.
    public: Q_INVOKABLE void SetdustModelList(
        const QStringList &_dustModelList);

    /// \brief Load parameters
    /// \param[in] _pluginElem plugin element
    public: Q_INVOKABLE void LoadConfig(const tinyxml2::XMLElement *_pluginElem);

    /// \brief Callback when dust emitter topic is changed
    /// \param[in] _dustEmitterTopic
    public: Q_INVOKABLE void OnDustEmitterModel(const QString &_dustEmitterModel);

    /// \brief Callback for density changd
    /// \param[in] _density
    public: Q_INVOKABLE void OnDensityChanged(double _density);

    /// \brief Callback for angle changed
    /// \param[in] _angle
    public: Q_INVOKABLE void OnAngleChanged(double _angle);

    /// \brief Callback for velocity changed
    /// \param[in] _velocity
    public: Q_INVOKABLE void OnVelocityChanged(double _velocity);

    /// \brief Callback for pose changed
    /// \param[in] _x
    /// \param[in] _y
    /// \param[in] _z
    /// \param[in] _angle
    public: Q_INVOKABLE void OnPoseChanged(double _x, double _y, double _z, double _angle);

    /// \brief Callback for size changed
    /// \param[in] _x
    /// \param[in] _y
    /// \param[in] _z
    public: Q_INVOKABLE void OnSizeChanged(double _x, double _y, double _z);

    /// \brief Callback for color changed
    /// \param[in] _dustColor
    public: Q_INVOKABLE void OnColorChanged(const QColor _dustColor);

    /// \brief Callback when dust on/off checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public: Q_INVOKABLE void ToggleDust(bool _checked);

    /// \brief Callback when wind on/off checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public: Q_INVOKABLE void ToggleWind(bool _checked);

    /// \brief Update function
    /// \param[in] _info
    /// \param[in] _ecm
    public: void Update(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;

    public: Q_INVOKABLE void SetupWindTopic();

    /// @brief Find all the visuals in the scene to get the ParticleEmitter pointer
    /// @param _ecm 
    public: void FindEntityVisual(
                    gz::sim::EntityComponentManager &_ecm);

    /// @brief Configure the emitter on selected model
    /// @param _emitter_name
    public: void ConfigureEmitter(std::string _emitter_name);

    /// @brief Get the density of the particle
    /// @param _lifetime
    /// @param _rate
    /// @param _scalRate
    /// @param _size
    public: double GetDensity(double _lifetime, double _rate, double _scalRate, gz::math::Vector3d _size);

    /// @brief Change the base color of the image
    /// @param image
    /// @param color
    public: gz::common::Image ChangeImageBaseColor(gz::common::Image image, const QColor color);
    
    /// @brief Set the wind velocity
    /// @param _velocity
    /// @param _windAngle
    public: void SetWindVelocity(double _velocity, double _windAngle);

    /// \brief Qt event filter
    /// \param _obj
    /// \param _event
    public: bool eventFilter(QObject *_obj, QEvent *_event);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<DustManagerPrivate> dataPtr;
};
}

#endif // GZ_GUI_PLUGINS_DustManager_HH_