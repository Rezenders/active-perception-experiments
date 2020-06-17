#include <thread>
#include <mutex>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include "ros/ros.h"
#include "std_srvs/SetBool.h"


namespace gazebo {
  class CameraSwitchPlugin : public SensorPlugin {
  private:
    sensors::SensorPtr sensor;

    std::string robot_namespace_;
    std::string camera_name_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::ServiceServer service;


    bool state = false;
    std::mutex mtx;
    std::thread switchThread_;

    void SwitchThread(){
      while (this->rosNode->ok()){
        mtx.lock();
        if(this->sensor->IsActive() != this->state){
          this->sensor->SetActive(this->state);
        }
        mtx.unlock();
      }
    }

  public:
    CameraSwitchPlugin(){}

    ~CameraSwitchPlugin(){
      rosNode->shutdown();
      switchThread_.join();
    }

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      // state.store(false);
      this->sensor = _sensor;

      this->robot_namespace_ =  GetRobotNamespace(_sensor, _sdf, "Camera");
      if (!_sdf->HasElement("cameraName"))
        ROS_DEBUG("Camera plugin missing <cameraName>, default to empty");
      else
        this->camera_name_ = _sdf->Get<std::string>("cameraName");

      this->rosNode.reset(new ros::NodeHandle(this->robot_namespace_ + "/" + this->camera_name_));

      service = rosNode->advertiseService("camera_switch", &CameraSwitchPlugin::OnRosMsg, this);
      this->switchThread_ = std::thread(&CameraSwitchPlugin::SwitchThread, this);

    }


    bool OnRosMsg(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
      mtx.lock();
      this->state = req.data;
      mtx.unlock();
      return true;
    }

  };
  GZ_REGISTER_SENSOR_PLUGIN(CameraSwitchPlugin);
}
