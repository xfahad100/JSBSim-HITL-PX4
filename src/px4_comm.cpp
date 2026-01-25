#include "px4_comm.h"

px4comm::px4comm(JSBSim::FGFDMExec *jsbsim) : SensorPlugin(jsbsim), _last_sim_time(0.0) {}

px4comm::~px4comm() {}

void px4comm::setSensorConfigs(const TiXmlElement& configs) {

}

bool px4comm::CommandToProperty(Eigen::Vector3d &AttitudeMAV, Eigen::Vector3d &gpsMAV,  Eigen::Vector3d &airspeedMAV, Eigen::Vector3d &accelMAV) {


   // std::cout << "servo1::  " <<airspeedMAV[0] <<endl;
    //std::cout << "servo2:  " <<airspeedMAV[1] <<endl;
     _sim_ptr->SetPropertyValue("pixhawk/sensor/airspeed", airspeedMAV[0]);
     _sim_ptr->SetPropertyValue("pixhawk/sensor/hdotmav", airspeedMAV[1]);
     _sim_ptr->SetPropertyValue("pixhawk/sensor/alt", airspeedMAV[2]);

    _sim_ptr->SetPropertyValue("pixhawk/sensor/hdotcalc", gpsMAV[0]);
    _sim_ptr->SetPropertyValue("pixhawk/sensor/lon", gpsMAV[1]);
    _sim_ptr->SetPropertyValue("pixhawk/sensor/alt", gpsMAV[2]);
   //temp
     _sim_ptr->SetPropertyValue("pixhawk/sensor/theta-deg", AttitudeMAV[0]);
    _sim_ptr->SetPropertyValue("pixhawk/sensor/phi-deg", AttitudeMAV[1]);
    _sim_ptr->SetPropertyValue("pixhawk/sensor/psi-deg", AttitudeMAV[2]);

    //_sim_ptr->SetPropertyValue("pixhawk/sensor/theta-deg", accelMAV[0]);
   //_sim_ptr->SetPropertyValue("pixhawk/sensor/phi-deg", accelMAV[1]);
   //_sim_ptr->SetPropertyValue("pixhawk/sensor/psi-deg", accelMAV[2]);
  return true;
}

Eigen::Vector4d px4comm::getActuatorsFromJSBSim() {
  double x = _sim_ptr->GetPropertyValue("fcs/elevator-pos-rad");
  double y = _sim_ptr->GetPropertyValue("fcs/left-aileron-pos-rad");
  double z = _sim_ptr->GetPropertyValue("fcs/rudder-pos-rad");
  double w = _sim_ptr->GetPropertyValue("fcs/throttle-cmd-norm[0]");
// std::cout << "elv::  " << x<<endl;
//  std::cout << "ail::  " << y<<endl;
//  std::cout << "rud::  " << z<<endl;
  return Eigen::Vector4d(x, y, z, w);
}

bool px4comm::getArmState() {
  bool armstate = _sim_ptr->GetPropertyValue("pixhawk/arming");
  return armstate;
}