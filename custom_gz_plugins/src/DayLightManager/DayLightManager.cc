/*
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

#include "DayLightManager.hh"

#include <cmath>
#include <string>
#include <iostream>
#include <map>

#include <gz/msgs.hh>
#include <gz/msgs/any.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/param_v.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/particle_emitter.pb.h>

#include <gz/transport.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/World.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Application.hh>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>

#include <gz/plugin/Register.hh>

namespace gz::sim::plugins
{
  // Private data structure for DayLightManager
  class DayLightManagerPrivate
  {
  public:
    // Gz setup variables
    gz::transport::Node _node;

    std::string _worldName;
    std::string _sunModelName{"sun_sphere"};
    std::string _serviceName;

    int _gzMsgsTimeout;

    std::string _planetSelect;
    gz::msgs::Pose _sunSpherePose;
    gz::msgs::Boolean _sunSpherePoseRes;
    bool _sunSpherePoseResult;

    // Code essential variables
    bool _init;

    int _curntRate;
    int _updateRateCnt;
    const int _updateRate{100};

    // Pointers and variables to hold current configurations
    int _totalMin;

    bool _updateSky;

    float **_selectPlanet;

    /* Planetary Parameters */

    // Earth Parameters
    const int EARTHOFFSET = 284;
    const int EARTHTOTALMIN = 24 * 60;
    const int EARTHDAYSINYEAR = 365;
    const double EARTHTILTAXIS = 23.44; // IN DEGREES
    const double EARTHHOURANGLEFACTOR = 15;

    // Mars Parameters
    const int MARSOFFSET = 250; // higher value use to model mars as earth
    const int MARSTOTALMIN = 24 * 60 + 40;
    const int MARSDAYSINYEAR = 687;
    const double MARSTILTAXIS = 25.19; // IN DEGREE
    const double MARSHOURANGLEFACTOR = 14.6;

    /* NOTE : Considering sky color of a planet as a function of solar altitude */

    float EARTHNIGHT[4] = {0.0, 0.0, 0.0, 1.0};
    float EARTHNOON[4] = {0.5294, 0.8078, 0.9216, 1.0};
    float EARTHDUSKDAWN[4] = {0.546, 0.45, 0.3423, 1.0};

    float *EARTHSKYCOLOR[3] = {&EARTHNIGHT[0], &EARTHDUSKDAWN[0], &EARTHNOON[0]};

  float MARSNIGHT[4] = {0.02, 0.02, 0.05, 1.0};
  float MARSNOON[4] = {1, 0.412 ,0.05, 1.0};
  float MARSDUSKDAWN[4] = {0.85, 0.45, 0.30, 1.0};

  float *MARSSKYCOLOR[3] = {&MARSNIGHT[0], &MARSDUSKDAWN[0], &MARSNOON[0]};

  float NIGHTTHRES = -0.3;
  float DUSKDAWNTHRES = 0.15; // after this point there will be a major transition in sky color ; Dusk -> Noon && Noon -< Dawn
  
  };
}

/////////////////////////////////////////////////
gz::sim::plugins::DayLightManager::DayLightManager() : GuiSystem(),
                                                       _dataPtr(new gz::sim::plugins::DayLightManagerPrivate),
                                                       _radius(150),               // : distance of sun position from origin, default 150
                                                       _timeOfDayMin(720),         // : variable time (in min) which changes
                                                       _day(1),                    // : Day 1 of Gregorian calendar, current constant , TODO : make it variable, increase as time passes
                                                       _observerLatitude(28.6139), // : Default observer Latitude
                                                       _y_bias(0),                 // : shift of sun position in y direction
                                                       _x_bias(0),                 // : shift of sun position in x direction
                                                       _speedMultiplier(1),        // : speed rate
                                                       _bgSet(false)               // : store whether or not background color is set on sun transition

{
  // Code essentials paramter intialize
  _dataPtr->_init = false;
  _dataPtr->_curntRate = 100;
  _dataPtr->_updateRateCnt = 0;
  _dataPtr->_gzMsgsTimeout = 1000;

  // Default planet parameter intialize
  _dataPtr->_planetSelect = "Earth";
  _dataPtr->_totalMin = _dataPtr->EARTHTOTALMIN;
  _dataPtr->_selectPlanet = _dataPtr->EARTHSKYCOLOR;

  this->_timeOfDayHour = (this->_timeOfDayMin * 1.0) / 60;

  this->_hourAngleFactor = this->_dataPtr->EARTHHOURANGLEFACTOR;
  this->_hourAngle = this->_hourAngleFactor * (this->_timeOfDayHour - 12);
  this->_hourAngleRadian = ToRadian(this->_hourAngle);

  this->_observerLatitudeRadian = ToRadian(this->_observerLatitude);

  this->_sunDeclination = _dataPtr->EARTHTILTAXIS * sin(2 * M_PI * (_dataPtr->EARTHOFFSET + this->_day) / _dataPtr->EARTHDAYSINYEAR);
  this->_sunDeclinationRadian = ToRadian(this->_sunDeclination);

  this->R_next = (this->_dataPtr->_selectPlanet[2])[0];
  this->G_next = (this->_dataPtr->_selectPlanet[2])[1];
  this->B_next = (this->_dataPtr->_selectPlanet[2])[2];

  // Connect with gz GUI application
  gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
gz::sim::plugins::DayLightManager::~DayLightManager() = default;

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "Day Light Manager";
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::Update(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm)
{
  if (!this->_dataPtr->_init) // one time: insert sun sphere and update its position
    gz::sim::plugins::DayLightManager::Setup(_ecm);

  if (!_info.paused)
  {
    if (this->_dataPtr->_init) // update sun positions depending upon Latitude, Month, and Day
      gz::sim::plugins::DayLightManager::SetSunPosition();

    this->_dataPtr->_updateRateCnt++;
  }
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::Setup(gz::sim::EntityComponentManager &_ecm)
{
  // Get the world name from ecm
  _ecm.Each<components::World, components::Name>(
      [&](const Entity &,
          const components::World *,
          const components::Name *_name) -> bool
      {
        this->_dataPtr->_worldName = _name->Data();
        return true;
      });

  // Remove pre-existing light source
  std::string lightSource = "sun";
  gz::msgs::Entity sunRemoveEntity;
  gz::msgs::Boolean sunRemoveRes;
  bool sunRemoveResult;
  sunRemoveEntity.set_name(lightSource);
  sunRemoveEntity.set_type(gz::msgs::Entity::LIGHT);
  std::string sunRemoveService = "/world/" + this->_dataPtr->_worldName + "/remove";
  this->_dataPtr->_node.Request(sunRemoveService, sunRemoveEntity, this->_dataPtr->_gzMsgsTimeout, sunRemoveRes, sunRemoveResult);

  // Insert sun_sphere model
  gz::msgs::EntityFactory sunSphereSpawnEntity;
  gz::msgs::Boolean sunSphereSpawnRes;
  bool sunSpawnResult;
  sunSphereSpawnEntity.set_sdf_filename("models/sun_sphere/model.sdf");
  sunSphereSpawnEntity.set_name(this->_dataPtr->_sunModelName);
  std::string sunSphereSpawnService = "/world/" + this->_dataPtr->_worldName + "/create";
  this->_dataPtr->_node.Request(sunSphereSpawnService, sunSphereSpawnEntity, this->_dataPtr->_gzMsgsTimeout, sunSphereSpawnRes, sunSpawnResult);

  // Set initial sun position
  gz::msgs::Vector3d *position = new gz::msgs::Vector3d;
  position->set_x(0);
  position->set_y(0);
  position->set_z(this->_radius);

  this->_dataPtr->_sunSpherePose.set_name(this->_dataPtr->_sunModelName);
  this->_dataPtr->_sunSpherePose.set_allocated_position(position);

  this->_dataPtr->_serviceName = "/world/" + this->_dataPtr->_worldName + "/set_pose";
  this->_dataPtr->_node.Request(this->_dataPtr->_serviceName, this->_dataPtr->_sunSpherePose, this->_dataPtr->_gzMsgsTimeout, this->_dataPtr->_sunSpherePoseRes, this->_dataPtr->_sunSpherePoseResult);

  this->_dataPtr->_init = true;
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::SetSunPosition()
{
  if (this->_dataPtr->_updateRateCnt >= this->_dataPtr->_curntRate)
  {

    /* _References_ */
    // https://andrewmarsh.com/apps/staging/sunpath3d.html
    // https://ntrs.nasa.gov/api/citations/20200003207/downloads/20200003207.pdf
    // https://stackoverflow.com/questions/8708048/position-of-the-sun-given-time-of-day-latitude-and-longitude
    // https://ntrs.nasa.gov/api/citations/19940010257/downloads/19940010257.pdf

    /* __Parameters needed to calculate sun position__ */
    // observer latitude (φ)
    // sun's declination (δ)
    // hour angle        (ω)
    // day               (1/jan as 1 day)

    // δ = 23.44° * sin(2π * (offset + day) / no_of_days_in_year)
    // θ = arctan2(sin(ω), (cos(ω) * sin(φ) - tan(δ) * cos(φ)))
    // α = arcsin(sin(φ) * sin(δ) + cos(φ) * cos(δ) * cos(ω))

    // convert polar to cartesian coordinates
    // x = r * cos(θ) * cos(α)
    // y = r * sin(θ) * cos(α)
    // z = r * sin(α)

    // Calculate time of day and hour angle
    this->_timeOfDayHour = (this->_timeOfDayMin * 1.0) / 60;
    this->_hourAngle = this->_hourAngleFactor * (this->_timeOfDayHour - 12);
    this->_hourAngleRadian = ToRadian(this->_hourAngle);

    // Calculate sun position
    float theta = atan2(sin(this->_hourAngleRadian), (cos(this->_hourAngleRadian) * sin(this->_observerLatitudeRadian) - tan(this->_sunDeclinationRadian) * cos(this->_observerLatitudeRadian)));
    float alpha = asin(sin(this->_observerLatitudeRadian) * sin(this->_sunDeclinationRadian) + cos(this->_observerLatitudeRadian) * cos(this->_sunDeclinationRadian) * cos(this->_hourAngleRadian));

    // Update sun coordinates
    this->_x_coordinate = this->_radius * cos(alpha) * cos(theta) + this->_x_bias;
    this->_y_coordinate = this->_radius * cos(alpha) * sin(theta) + this->_y_bias;
    this->_z_coordinate = this->_radius * sin(alpha);

    // Set new sun position
    gz::msgs::Vector3d *position = new gz::msgs::Vector3d;
    position->set_x(this->_x_coordinate);
    position->set_y(this->_y_coordinate);
    position->set_z(this->_z_coordinate);

    double yaw = atan2(this->_y_coordinate, this->_x_coordinate);
    double pitch = atan2(sqrt((pow(this->_x_coordinate, 2) + pow(this->_y_coordinate, 2))), this->_z_coordinate);
    double roll = 0;

    gz::sim::plugins::DayLightManager::EulerToQuat(roll, pitch, yaw);

    gz::msgs::Quaternion *orientation = new gz::msgs::Quaternion;
    orientation->set_w(this->_sunAngleRollQuat[0]);
    orientation->set_x(this->_sunAngleRollQuat[1]);
    orientation->set_y(this->_sunAngleRollQuat[2]);
    orientation->set_z(this->_sunAngleRollQuat[3]);

    this->_dataPtr->_sunSpherePose.set_name(this->_dataPtr->_sunModelName);
    this->_dataPtr->_sunSpherePose.set_allocated_position(position);
    this->_dataPtr->_sunSpherePose.set_allocated_orientation(orientation);

    this->_dataPtr->_node.Request(this->_dataPtr->_serviceName, this->_dataPtr->_sunSpherePose, this->_dataPtr->_gzMsgsTimeout, this->_dataPtr->_sunSpherePoseRes, this->_dataPtr->_sunSpherePoseResult);

    // Update time display
    std::string displayTime = std::to_string(int(this->_timeOfDayHour)) + " : " + std::to_string(int(this->_timeOfDayMin) % 60) + " Hours";
    this->time = QString::fromStdString(displayTime);
    gz::sim::plugins::DayLightManager::timeChanged();

    if (_bgSet)
      gz::sim::plugins::DayLightManager::setBgColor();

    this->_dataPtr->_updateSky = true;

    // Increment time
    this->_timeOfDayMin += 1;
    this->_timeOfDayMin = this->_timeOfDayMin % this->_dataPtr->_totalMin;

    this->_dataPtr->_updateRateCnt = 0;
  }
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::setBgColor()
{

  float normalizeHeight = this->_z_coordinate / this->_radius;

  // Lambda function for smooth transition
  auto sigmoid = [](float x, float smoothness = 1.0f)
  {
    return 1.0f / (1.0f + std::exp(-smoothness * x));
  };

  // Adjust the night threshold
  if (normalizeHeight <= this->_dataPtr->NIGHTTHRES)
  {
    // Full night
    this->R_next = (this->_dataPtr->_selectPlanet[0])[0];
    this->G_next = (this->_dataPtr->_selectPlanet[0])[1];
    this->B_next = (this->_dataPtr->_selectPlanet[0])[2];
  }

  else
  {
    // Calculate transition factor
    float nightToDuskFactor = std::clamp((normalizeHeight - this->_dataPtr->NIGHTTHRES) / (this->_dataPtr->DUSKDAWNTHRES - this->_dataPtr->NIGHTTHRES), 0.0f, 1.0f);
    float t1 = sigmoid(nightToDuskFactor * 2 - 1, 4.0f);

    // Interpolate between night and dusk
    float r1 = (this->_dataPtr->_selectPlanet[0])[0] * (1 - t1) + (this->_dataPtr->_selectPlanet[1])[0] * t1;
    float g1 = (this->_dataPtr->_selectPlanet[0])[1] * (1 - t1) + (this->_dataPtr->_selectPlanet[1])[1] * t1;
    float b1 = (this->_dataPtr->_selectPlanet[0])[2] * (1 - t1) + (this->_dataPtr->_selectPlanet[1])[2] * t1;

    if (normalizeHeight <= this->_dataPtr->DUSKDAWNTHRES)
    {
      // Between night and full dusk/dawn
      this->R_next = r1;
      this->G_next = g1;
      this->B_next = b1;
    }
    else
    {
      // Transition from dusk to day
      float duskToDayFactor = std::clamp((normalizeHeight - this->_dataPtr->DUSKDAWNTHRES) / (1 - this->_dataPtr->DUSKDAWNTHRES), 0.0f, 1.0f);
      float t2 = sigmoid(duskToDayFactor * 2 - 1, 4.0f);

      // Interpolate between dusk and day
      this->R_next = r1 * (1 - t2) + (this->_dataPtr->_selectPlanet[2])[0] * t2;
      this->G_next = g1 * (1 - t2) + (this->_dataPtr->_selectPlanet[2])[1] * t2;
      this->B_next = b1 * (1 - t2) + (this->_dataPtr->_selectPlanet[2])[2] * t2;
    }
  }
}

/////////////////////////////////////////////////
float gz::sim::plugins::DayLightManager::ToRadian(float degree)
{
  return (degree) * (M_PI / 180);
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::SetPlanet(const QString &_planet)
{
  this->_dataPtr->_planetSelect = _planet.toStdString();

  if (this->_dataPtr->_planetSelect == "Earth")
  {

    this->_day = 1;
    this->_sunDeclination = this->_dataPtr->EARTHTILTAXIS * sin(2 * M_PI * (this->_dataPtr->EARTHOFFSET + this->_day) / this->_dataPtr->EARTHDAYSINYEAR);
    this->_sunDeclinationRadian = ToRadian(this->_sunDeclination);

    this->_timeOfDayMin = 0;
    this->_hourAngleFactor = this->_dataPtr->EARTHHOURANGLEFACTOR;
    this->_dataPtr->_totalMin = _dataPtr->EARTHTOTALMIN;
    _dataPtr->_selectPlanet = _dataPtr->EARTHSKYCOLOR;
  }

  if (this->_dataPtr->_planetSelect == "Mars")
  {

    this->_day = 1;
    this->_sunDeclination = this->_dataPtr->MARSTILTAXIS * sin(2 * M_PI * (this->_dataPtr->MARSOFFSET + this->_day) / this->_dataPtr->MARSDAYSINYEAR);
    this->_sunDeclinationRadian = ToRadian(this->_sunDeclination);

    this->_timeOfDayMin = 0;
    this->_hourAngleFactor = this->_dataPtr->MARSHOURANGLEFACTOR;
    this->_dataPtr->_totalMin = _dataPtr->MARSTOTALMIN;
    _dataPtr->_selectPlanet = _dataPtr->MARSSKYCOLOR;
  }
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::SetLatitude(double _latitude)
{
  this->_observerLatitude = _latitude;
  this->_observerLatitudeRadian = ToRadian(this->_observerLatitude);
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::SetTime(int _timeOfDay)
{
  this->_timeOfDayMin = _timeOfDay * 60;
}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::SetSpeed(int _speedMultiplier)
{
  this->_speedMultiplier = _speedMultiplier;
  this->_dataPtr->_curntRate = this->_dataPtr->_updateRate / this->_speedMultiplier;
  this->_dataPtr->_updateRateCnt = 0;
}

void gz::sim::plugins::DayLightManager::SetRadius(int _radius)
{
  this->_radius = _radius;
}

void gz::sim::plugins::DayLightManager::SetX(int _x_bias)
{

  this->_x_bias = _x_bias;
}

void gz::sim::plugins::DayLightManager::SetY(int _y_bias)
{

  this->_y_bias = _y_bias;
}

void gz::sim::plugins::DayLightManager::BgColor(bool _checked )
{
  this->_bgSet = _checked;
}
/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::EulerToQuat(float roll, float pitch, float yaw)
{
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  float w = cr * cp * cy + sr * sp * sy;
  float x = sr * cp * cy - cr * sp * sy;
  float y = cr * sp * cy + sr * cp * sy;
  float z = cr * cp * sy - sr * sp * cy;

  // Normalize the quaternion
  float magnitude = std::sqrt(w * w + x * x + y * y + z * z);
  if (magnitude != 0.0f)
  {
    this->_sunAngleRollQuat[0] = w / magnitude;
    this->_sunAngleRollQuat[1] = x / magnitude;
    this->_sunAngleRollQuat[2] = y / magnitude;
    this->_sunAngleRollQuat[3] = z / magnitude;
  }
  else
  {
    // Handle error case (division by zero)
    this->_sunAngleRollQuat[0] = 1.0;
    this->_sunAngleRollQuat[1] = 0.0;
    this->_sunAngleRollQuat[2] = 0.0;
    this->_sunAngleRollQuat[3] = 0.0;
  }
}

/////////////////////////////////////////////////
bool gz::sim::plugins::DayLightManager::eventFilter(QObject *_obj, QEvent *_event)
{

  if (_event->type() == gz::gui::events::Render::kType)
    if (this->_dataPtr->_updateSky)
      gz::sim::plugins::DayLightManager::PerformRenderingOperations();

  return QObject::eventFilter(_obj, _event);
}

void gz::sim::plugins::DayLightManager::PerformRenderingOperations()
{

  if (nullptr == this->scene)
    gz::sim::plugins::DayLightManager::FindScene();

  if (nullptr == this->scene)
    return;

  if (_bgSet){
    this->scene->SetBackgroundColor({ this->R_next,
                                      this->G_next, 
                                      this->B_next, 
                                      1.0});

    this->_dataPtr->_updateSky = false;
  }

}

/////////////////////////////////////////////////
void gz::sim::plugins::DayLightManager::FindScene()
{

  auto loadedEngNames = gz::rendering::loadedEngines();

  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
          << "Using engine [" << engineName << "]" << std::endl;
  }

  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
          << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
    gzdbg << "More than one scene is available. " << "Using scene [" << scene->Name() << "]" << std::endl;

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
    return;

  this->scene = scenePtr;
}

/////////////////////////////////////////////////
GZ_ADD_PLUGIN(gz::sim::plugins::DayLightManager,
              gz::gui::Plugin)