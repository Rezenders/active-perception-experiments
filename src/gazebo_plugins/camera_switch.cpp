#include <thread>
#include <atomic>

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

    std::atomic<bool> state;

    std::thread switchThread_;
    void SwitchThread(){
      // static const double timeout = 0.01;
      while (this->rosNode->ok()){
        if(this->sensor->IsActive() != state.load()){
          this->sensor->SetActive(state.load());
        }
      }
    }

  public:
    CameraSwitchPlugin(){}

    ~CameraSwitchPlugin(){
      rosNode->shutdown();
      // rosQueueThread.join();
    }

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      state = false;
      this->sensor = _sensor;

      this->robot_namespace_ =  GetRobotNamespace(_sensor, _sdf, "Camera");
      if (!_sdf->HasElement("cameraName"))
        ROS_DEBUG("Camera plugin missing <cameraName>, default to empty");
      else
        this->camera_name_ = _sdf->Get<std::string>("cameraName");

      this->rosNode.reset(new ros::NodeHandle(this->robot_namespace_ + "/" + this->camera_name_));

      this->switchThread_ =
        std::thread(std::bind(&CameraSwitchPlugin::SwitchThread, this));
        
      service = rosNode->advertiseService("camera_switch", &CameraSwitchPlugin::OnRosMsg, this);
    }

    bool OnRosMsg(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
      this->state = req.data;
      // while(this->sensor->IsActive() != req.data){
      //   this->sensor->SetActive(req.data);
      // }
      return true;
    }

  };
  GZ_REGISTER_SENSOR_PLUGIN(CameraSwitchPlugin);
}
