/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#ifndef GZ_GUI_DAYLIGHTMANAGER_PLUGINS_HH_
#define GZ_GUI_DAYLIGHTMANAGER_PLUGINS_HH_

#include <memory>

#include <gz/gui/qt.h>
#include <gz/sim/gui/GuiSystem.hh>

#include <gz/gui/Plugin.hh>
#include <gz/rendering/Scene.hh>

namespace gz::sim::plugins {

class DayLightManagerPrivate;

/// \brief A class to manage daylight simulation in a GUI environment
class DayLightManager : public gz::sim::GuiSystem
{
  Q_OBJECT

public:
  /// \brief Constructor for DayLightManager
  DayLightManager();

  /// \brief Destructor for DayLightManager
  ~DayLightManager() override;

  /// \brief Loads configuration from an XML element
  /// \param[in] _pluginElem XML element containing configuration data
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Updates the DayLightManager state
  /// \param[in] _info Update information
  /// \param[in] _ecm Entity component manager
  void Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Sets up the DayLightManager
  /// \param[in] _ecm Entity component manager
  void Setup(gz::sim::EntityComponentManager &_ecm);
  
  /// \brief Sets the sun position based on current time and location
  void SetSunPosition();

  /// \brief Converts Euler angles to a quaternion
  /// \param[in] roll Roll angle in radians
  /// \param[in] pitch Pitch angle in radians
  /// \param[in] yaw Yaw angle in radians
  void EulerToQuat(float roll, float pitch, float yaw);

public slots:
  /// \brief Sets the planet for daylight calculations
  /// \param[in] _planet Name of the planet
  void SetPlanet(const QString & _planet);

  /// \brief Sets the latitude for daylight calculations
  /// \param[in] _latitude Latitude in degrees
  void SetLatitude(double _latitude);

  /// \brief Sets the time of day
  /// \param[in] _timeOfDay Time of day in minutes since midnight
  void SetTime(int _timeOfDay);

  /// \brief Sets the speed multiplier for time progression
  /// \param[in] _speedMultiplier Speed multiplier for time progression
  void SetSpeed(int _speedMultiplier);

  /// \brief Sets the radius for solar path 
  /// \param[in] _radius radius for solar path 
  void SetRadius(int _radius);

  /// \brief Sets X coordinate of sun 
  /// \param[in] _x_bias bias value to shift sun x coordinate 
  void SetX(int _x_bias);

  /// \brief Sets Y coordinate of sun 
  /// \param[in] _y_bias bias value to shift sun y coordinate 
  void SetY(int _y_bias);

  /// \brief Check if user set background transition or not 
  /// \param[in] e 
  void BgColor(bool _checked );

private:
  /// \brief Pointer to private data
  std::unique_ptr<DayLightManagerPrivate> _dataPtr;

public:
  /// \brief Converts degrees to radians
  /// \param[in] radian Angle in degrees
  /// \return Angle in radians
  float ToRadian(float radian);
  
  /// \brief Property for current time display
  Q_PROPERTY(QString time MEMBER time NOTIFY timeChanged)

signals:
  /// \brief Signal emitted when time changes
  void timeChanged();

public:
  /// \brief Current time display string
  QString time {QString::fromStdString("12 : 00 Hours")};

private: 
  /// \brief Pointer to the rendering scene
  gz::rendering::ScenePtr scene{nullptr};

  /// \brief Perform rendering operations
  void PerformRenderingOperations();

  /// \brief Event filter for Qt events
  /// \param[in] _obj Object that triggered the event
  /// \param[in] _event The event that occurred
  /// \return True if the event was handled, false otherwise
  bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief Find the rendering scene
  void FindScene();

public:
  /// \brief Set the background color
  void setBgColor();

public:

  /// \brief distance of sun from origin
  int _radius;

  /// \brief Time of day in minutes
  int _timeOfDayMin;

  /// \brief Current day
  int _day;

  /// \brief Time of day in hours
  float _timeOfDayHour;

  /// \brief Factor for hour angle calculation
  double _hourAngleFactor;

  /// \brief Hour angle
  float _hourAngle; 

  /// \brief Hour angle in radians
  float _hourAngleRadian; 

  /// \brief Observer's latitude
  double _observerLatitude; 

  /// \brief Sun's declination
  float _sunDeclination;  

  /// \brief Sun's declination in radians
  float _sunDeclinationRadian; 

  /// \brief Observer's latitude in radians
  float _observerLatitudeRadian; 
  
  /// \brief X coordinate of the sun
  float _x_coordinate;  

  /// \brief Y coordinate of the sun
  float _y_coordinate;

  /// \brief Z coordinate of the sun
  float _z_coordinate;

  /// \brief Y bias for sun position
  int _y_bias;

  /// \brief X bias for sun position
  int _x_bias;

  /// \brief Next R value for color transition
  float R_next;

  /// \brief Next G value for color transition
  float G_next;

  /// \brief Next B value for color transition
  float B_next;

  /// \brief Sun angle roll quaternion
  float _sunAngleRollQuat[4];

  /// \brief Speed multiplier for time progression
  float _speedMultiplier; 

  /// \brief Speed multiplier for time progression
  bool _bgSet;


};

}

#endif