#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <ignition/math/Pose3.hh>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Pose.h"

namespace gazebo{
  class SpawnBuoyPlugin : public WorldPlugin{
  private:
    physics::WorldPtr world;

    std::unique_ptr<ros::NodeHandle> rosNode;
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
    SpawnBuoyPlugin() : WorldPlugin(){}

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      world = _world;
      this->rosNode.reset(new ros::NodeHandle("spawn_buoy"));

      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Pose>(
          "/rescue_world/drop_buoy",
          1,
          boost::bind(&SpawnBuoyPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread =
        std::thread(std::bind(&SpawnBuoyPlugin::QueueThread, this));

    }

    void OnRosMsg(const geometry_msgs::PoseConstPtr &_msg){
      transport::NodePtr node(new transport::Node());

      node->Init(world->Name());

      transport::PublisherPtr factoryPub =
        node->Advertise<msgs::Factory>("~/factory");

      msgs::Factory factory_msg;
      factory_msg.set_sdf_filename("model://lifebuoy");

      msgs::Set(factory_msg.mutable_pose(),
          ignition::math::Pose3d(
            ignition::math::Vector3d(_msg->position.x, _msg->position.y, _msg->position.z),
            ignition::math::Quaterniond(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z )));

      factoryPub->Publish(factory_msg);
    }

  };
  GZ_REGISTER_WORLD_PLUGIN(SpawnBuoyPlugin)
}
