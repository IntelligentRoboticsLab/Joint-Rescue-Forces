// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <stdio.h>

#include <termios.h>
#include <iostream>

#define D_SDFGET_NAME          this->model->GetName().c_str()
//#define D_SDFGET_NAME          this->model->URI().Str().c_str()
//#define D_SDFGET_NAME          this->sdf->GetName().c_str()
//#define D_SDFGET(N,T)          this->sdf->GetElement(N)->Get<## T ##>()
//#define D_SDFGET_TYPE(X,N,D,T) if(!this->sdf->HasElement(N))\
//                               {ROS_WARN("%s:No <%s>, used default value",\
//                                D_SDFGET_NAME,N);X=D;} else {X=SDFGET(N,T);}
#define D_SDFGET_ELMT(N)       ((!this->sdf->HasElement(N))?NULL:\
                               this->sdf->GetElement(N)->Get<std::string>()\
                               .c_str())
#define D_SDFGET_JOINT(J,N)    ROS_INFO("%s:%s=%s",\
                                            D_SDFGET_NAME,N,D_SDFGET_ELMT(N));\
                               if(!(J=this->model->GetJoint(D_SDFGET_ELMT(N))))\
                               {ROS_ERROR("%s:No JOINT <%s>",D_SDFGET_NAME,N);\
                                return false;}
#define D_SDFGET_STRING(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<std::string>();}
#define D_SDFGET_DOUBLE(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<double>();}
#define D_SDFGET_BOOL(X,N,D) if(!this->sdf->HasElement(N))\
                               {ROS_WARN("%s:No <%s>, used default value",\
                                D_SDFGET_NAME,N);X=D;} else\
                               {X=this->sdf->GetElement(N)->Get<bool>();}
//#define D_SDFGET_STRING(S,N,D) D_SDFGET_TYPE(S,N,D,std::string)
//#define D_SDFGET_DOUBLE(S,N,D) D_SDFGET_TYPE(S,N,D,double)
//#define D_SDFGET_BOOL(S,N,D)   D_SDFGET_TYPE(S,N,D,bool)

using namespace gazebo;

class ObjectPosePlugin : public ModelPlugin
{
  transport::NodePtr node;
  physics::ModelPtr  model;
  sdf::ElementPtr    sdf;
  common::Time       simTime;
/*
  // Gazebo Topic
  transport::SubscriberPtr velSub;
  transport::SubscriberPtr flpSub;
*/
  // Loop Event
  event::ConnectionPtr updateConnection;
 
  // Update Rate
  double       update_rate_;
  double       update_period_;
  common::Time last_update_time_;
            
  // ROS Callback Queue
  ros::CallbackQueue queue_;
  GazeboRosPtr       gazebo_ros_;
  boost::mutex       lock_;
  boost::thread      callback_queue_thread_;

  // ROS STUFF
  ros::Subscriber pose_subscriber;

  std::string robotnamespace_;
  std::string topicname_pose_;
  
  public:
  ObjectPosePlugin(void){}

  ~ObjectPosePlugin()
  {
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while(/*alive_ &&*/gazebo_ros_->node()->ok())
    {
      queue_.callAvailable (ros::WallDuration(timeout));
    }
  }

  /////////////////////////////////////////////////
  void poseCallback(const geometry_msgs::Pose::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock_);
    math::Pose nextPose(math::Vector3(
     cmd_msg->position.x, cmd_msg->position.y, cmd_msg->position.z), 
     math::Quaternion(cmd_msg->orientation.x, cmd_msg->orientation.y, cmd_msg->orientation.z));
    this->model->SetWorldPose(nextPose);
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // physics::WorldPtr world = physics::get_world("default");
    this->model = _model;
    this->sdf   = _sdf;
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_ = GazeboRosPtr(new GazeboRos(model, _sdf, "robot"));
    gazebo_ros_->isInitialized();
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());
    // ROS: setting parameters
    gazebo_ros_->getParameter<std::string> (robotnamespace_, 
                                          "robotNamespace", robotnamespace_);
//    gzerr<<robotnamespace_;
    gazebo_ros_->getParameter<std::string> (topicname_pose_, 
                                               "topicName", topicname_pose_);
    // ROS: Registering Subscribers and Publishers
    ros::SubscribeOptions sopose =
      ros::SubscribeOptions::create<geometry_msgs::Pose>
          (topicname_pose_,
          1, boost::bind(&ObjectPosePlugin::poseCallback, this, _1),
          ros::VoidPtr(), &queue_);
    pose_subscriber = gazebo_ros_->node()->subscribe(sopose);
    // Start custom queue for hearing ROS topics
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&ObjectPosePlugin::QueueThread, this));
  }
};

GZ_REGISTER_MODEL_PLUGIN(ObjectPosePlugin)
