#include <thread>
#include <chrono>
#include <mutex>

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo{
  class VictimPlugin : public ModelPlugin{
  private:
    gazebo::transport::NodePtr node;
    std::unique_ptr<ros::NodeHandle> rosNode;

    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;

    bool drowning = true;

    physics::ModelPtr model;

    int drowning_time_ = 0;
    std::thread timer_thread_;
    std::mutex mtx;

    void timerThreadFunction(){
      std::this_thread::sleep_for(
        std::chrono::milliseconds(this->drowning_time_));

      // mtx.lock();
      if(this->drowning){
        gazebo::transport::PublisherPtr request_pub = this->node->Advertise<msgs::Request>("~/request");
        msgs::Request *msg = msgs::CreateRequest("entity_delete", this->model->GetName());
        request_pub->Publish(*msg);
        delete msg;
      }
      // mtx.unlock();
    }

    void QueueThread(){
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  public:
    VictimPlugin() : ModelPlugin(){}
    ~VictimPlugin(){
      timer_thread_.join();
      rosQueue.clear();
      rosQueue.disable();
      rosNode->shutdown();
      rosQueueThread.join();
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
      model = _model;

      this->node = boost::make_shared<gazebo::transport::Node>();
      this->node->Init();

      if (!_sdf->HasElement("drowningTime")){
        this->drowning_time_ = 5000;
        ROS_DEBUG("Victim plugin missing <drowningTime>, default to 5000ms");
      }else{
        this->drowning_time_ = _sdf->Get<int>("drowningTime");
      }

      if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      this->rosNode.reset(new ros::NodeHandle("victim"));

      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Bool>(
          "/victim/"+this->model->GetName(),
          1,
          boost::bind(&VictimPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

      this->rosSub = this->rosNode->subscribe(so);

      this->rosQueueThread =
        std::thread(std::bind(&VictimPlugin::QueueThread, this));

      this->timer_thread_ =
        std::thread(std::bind(&VictimPlugin::timerThreadFunction, this));
    }

    void OnRosMsg(const std_msgs::BoolConstPtr &_msg){
      // mtx.lock();
      this->drowning = _msg->data;
      // mtx.unlock();
    }
  };
  GZ_REGISTER_MODEL_PLUGIN(VictimPlugin)
}
