#include "common.h"
#include "sensor_plugin.h"

#include <FGFDMExec.h>
#include <tinyxml.h>
#include <Eigen/Eigen>


class px4comm : public SensorPlugin {
 public:
  px4comm(JSBSim::FGFDMExec *jsbsim);
  ~px4comm();
  bool CommandToProperty(Eigen::Vector3d &AttitudeMAV, Eigen::Vector3d &gpsMAV, Eigen::Vector3d &airspeedMAV, Eigen::Vector3d &accelMAV);
  Eigen::Vector4d getActuatorsFromJSBSim();
  bool getArmState();
  void setSensorConfigs(const TiXmlElement& configs);

 private:
  double _last_sim_time;
};