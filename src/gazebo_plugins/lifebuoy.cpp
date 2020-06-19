#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <ignition/math/Pose3.hh>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/DeleteModel.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

#include <iostream>
#include <math.h>
#include <unistd.h>

double euclideanDistance(geometry_msgs::Pose &buoyPose, ignition::math::Pose3d &modelPose){
  return  pow(pow(buoyPose.position.x - modelPose.Pos().X(), 2) +
              pow(buoyPose.position.y - modelPose.Pos().Y(), 2) +
              pow(buoyPose.position.z - modelPose.Pos().Z(), 2), 0.5);
}

namespace gazebo{
  class LifeBuoyPlugin : public ModelPlugin{
  private:
    physics::ModelPtr model;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher buoyHandlePub;

    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

    void QueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  public:
    LifeBuoyPlugin() : ModelPlugin(){}

    ~LifeBuoyPlugin(){
      rosQueue.clear();
      rosQueue.disable();
      rosNode->shutdown();                     // This MUST BE CALLED before thread join()!!
      rosQueueThread.join();
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;

      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      this->rosNode.reset(new ros::NodeHandle("buoy"));
      buoyHandlePub = rosNode->advertise<std_msgs::String>("/rescue_world/delete_model", 1);

      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
          "/gazebo/model_states",
          1,
          boost::bind(&LifeBuoyPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread =
        std::thread(std::bind(&LifeBuoyPlugin::QueueThread, this));
    }

    void OnRosMsg(const gazebo_msgs::ModelStatesConstPtr &_msg){
      for (size_t i = 0; i < _msg->name.size(); i++) {
        if(_msg->name.at(i).compare(0, 6,"victim") == 0){
          geometry_msgs::Pose buoyPose = _msg->pose.at(i);
          ignition::math::Pose3d modelPose = model->WorldPose();

          double distance = euclideanDistance(buoyPose, modelPose);
          if(distance < 2.5){
            // ros::Duration(1.5).sleep();
            gazebo_msgs::DeleteModel del_model;
            del_model.request.model_name = _msg->name.at(i);
            ros::service::call("/gazebo/delete_model", del_model);

            std_msgs::String del_model2;
            del_model2.data = model->GetName();
            buoyHandlePub.publish(del_model2);

            i = _msg->name.size();
          }
        }
      }
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(LifeBuoyPlugin)
}
