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
#include "std_msgs/String.h"
#include "gazebo_msgs/DeleteModel.h"

namespace gazebo{
  class SpawnBuoyPlugin : public WorldPlugin{
  private:
    physics::WorldPtr world;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber spawnBuoySub;
    ros::CallbackQueue spawnBuoyQueue;
    std::thread spawnBuoyThread;

    void SpawnBuoyQueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->spawnBuoyQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    ros::Subscriber deleteBuoySub;
    ros::CallbackQueue deleteBuoyQueue;
    std::thread deleteBuoyThread;

    void DeleteBuoyQueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->deleteBuoyQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  public:
    SpawnBuoyPlugin() : WorldPlugin(){}
    ~SpawnBuoyPlugin(){
      spawnBuoyQueue.clear();
      deleteBuoyQueue.clear();
      spawnBuoyQueue.disable();
      deleteBuoyQueue.disable();
      rosNode->shutdown();
      spawnBuoyThread.join();
      deleteBuoyThread.join();
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }


      world = _world;
      this->rosNode.reset(new ros::NodeHandle("buoy_handler"));

      ros::SubscribeOptions soSpawnBuoy =
        ros::SubscribeOptions::create<geometry_msgs::Pose>(
          "/rescue_world/drop_buoy",
          1,
          boost::bind(&SpawnBuoyPlugin::spawnBuoyCallback, this, _1),
          ros::VoidPtr(), &this->spawnBuoyQueue);

      this->spawnBuoySub = this->rosNode->subscribe(soSpawnBuoy);
      this->spawnBuoyThread =
        std::thread(std::bind(&SpawnBuoyPlugin::SpawnBuoyQueueThread, this));

      ros::SubscribeOptions soDeleteBuoy =
        ros::SubscribeOptions::create<std_msgs::String>(
          "/rescue_world/delete_model",
          1,
          boost::bind(&SpawnBuoyPlugin::deleteBuoyCallBack, this, _1),
          ros::VoidPtr(), &this->deleteBuoyQueue);
      this->deleteBuoySub = this->rosNode->subscribe(soDeleteBuoy);
      this->deleteBuoyThread =
        std::thread(std::bind(&SpawnBuoyPlugin::DeleteBuoyQueueThread, this));
    }

    void spawnBuoyCallback(const geometry_msgs::PoseConstPtr &_msg){
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

    void deleteBuoyCallBack(const std_msgs::String::ConstPtr& msg){
      gazebo_msgs::DeleteModel del_model;
      del_model.request.model_name = msg->data;
      std::cout<<msg->data<<std::endl;
      ros::service::call("/gazebo/delete_model", del_model);
    }

  };
  GZ_REGISTER_WORLD_PLUGIN(SpawnBuoyPlugin)
}
