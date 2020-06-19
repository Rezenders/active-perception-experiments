#include <thread>
#include <mutex>
#include <chrono>

#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
// #include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"


namespace gazebo {
  class CameraSwitchPlugin : public SensorPlugin {
  private:
    sensors::SensorPtr sensor;

    std::string robot_namespace_;
    std::string camera_name_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    // ros::ServiceServer service;

    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

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
        std::this_thread::sleep_for (std::chrono::milliseconds(100));
      }
    }

    void QueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  public:
    CameraSwitchPlugin(){}

    ~CameraSwitchPlugin(){
      rosQueue.clear();
      rosQueue.disable();
      rosNode->shutdown();
      rosQueueThread.join();
      switchThread_.join();
    }

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

      this->sensor = _sensor;

      this->robot_namespace_ =  GetRobotNamespace(_sensor, _sdf, "Camera");
      if (!_sdf->HasElement("cameraName"))
        ROS_DEBUG("Camera plugin missing <cameraName>, default to empty");
      else
        this->camera_name_ = _sdf->Get<std::string>("cameraName");

      this->rosNode.reset(new ros::NodeHandle(this->robot_namespace_ + "/" + this->camera_name_));

      this->switchThread_ = std::thread(&CameraSwitchPlugin::SwitchThread, this);

      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Bool>(
          "camera_switch",
          1,
          boost::bind(&CameraSwitchPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread =
        std::thread(std::bind(&CameraSwitchPlugin::QueueThread, this));
      // service = rosNode->advertiseService("camera_switch", &CameraSwitchPlugin::OnRosMsg, this);

    }

    void OnRosMsg(const std_msgs::BoolConstPtr &_msg){
      mtx.lock();
      this->state = _msg->data;
      mtx.unlock();
    }
    // bool OnRosMsg(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    //   mtx.lock();
    //   this->state = req.data;
    //   mtx.unlock();
    //   return true;
    // }
    // bool OnRosMsg(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){
    //   if(this->sensor->IsActive() != req.data){
    //     this->sensor->SetActive(req.data);
    //   }
    //   return true;
    // }

  };
  GZ_REGISTER_SENSOR_PLUGIN(CameraSwitchPlugin);
}
