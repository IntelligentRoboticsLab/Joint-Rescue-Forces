/*
BSD 3-Clause License

Copyright (c) 2019, Arnoud Visser
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// GAZEBO
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Color.hh>

// THREAD
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

// ROS
#include <ros/ros.h>
#include <gazebo_msgs/SetLightProperties.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

using namespace gazebo;

class LEDVisualPlugin : public VisualPlugin
{
  // Constructor
  public: LEDVisualPlugin();

  // Destructor
  public: ~LEDVisualPlugin();

  // Load the controller
  public: void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

  private: rendering::VisualPtr model_;
  private: sdf::ElementPtr sdf_;

  private: ros::CallbackQueue model_queue_;
  private: void VisualQueueThread();
  private: boost::thread callback_queue_thread_;

  // Pointer to ros node
  private: ros::NodeHandle* rosnode_;

  private: ros::ServiceServer srv_;
  private: std::string service_name_;
  private: std::string robot_namespace_;

  // Callback when using service
  private:  bool ServiceCallback(gazebo_msgs::SetLightProperties::Request &req,
   gazebo_msgs::SetLightProperties::Response &res);
};

LEDVisualPlugin::LEDVisualPlugin(){}
LEDVisualPlugin::~LEDVisualPlugin()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

void LEDVisualPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
    return;
  }
  this->model_ = _visual;
  this->sdf_ = _sdf;
  this->robot_namespace_ = "";
  if (this->sdf_->HasElement("robotNamespace"))
  {
    this->robot_namespace_ = this->sdf_->Get<std::string>("robotNamespace") + "/";
  }
/*
  ROS_INFO("========================== %s\n", this->robot_namespace_.c_str());
  ROS_WARN("========================== %s\n", this->robot_namespace_.c_str());
  ROS_ERROR("========================== %s\n", this->robot_namespace_.c_str());
  gzmsg << "==========================" << this->robot_namespace_;
  gzwarn << "==========================" << this->robot_namespace_;
  gzerr << "==========================" << this->robot_namespace_;
*/
  this->service_name_ = "/led_switch";
  if (this->sdf_->HasElement("serviceName"))
  {
    this->service_name_ = this->sdf_->Get<std::string>("serviceName");
  }
  this->service_name_ = this->robot_namespace_ + this->service_name_;
  if (this->sdf_->HasElement("color"))
  {
    gazebo::common::Color default_color = this->sdf_->Get<gazebo::common::Color>("color");
    this->model_->SetAmbient(default_color);
    this->model_->SetDiffuse(default_color);
  }
  // Initialize the ROS node for the gazebo client if necessary
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "change_led", ros::init_options::NoSigintHandler);
  }
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  // advertise services on the custom queue
  ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLightProperties>(
   this->service_name_, boost::bind(&LEDVisualPlugin::ServiceCallback, this, _1, _2), ros::VoidPtr(), &this->model_queue_);
  this->srv_ = rosnode_->advertiseService(aso);
  this->callback_queue_thread_ = boost::thread(boost::bind(&LEDVisualPlugin::VisualQueueThread, this));
}

#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

// Callback when using service
bool LEDVisualPlugin::ServiceCallback(gazebo_msgs::SetLightProperties::Request &req, 
gazebo_msgs::SetLightProperties::Response &res)
{
  gazebo::common::Color ledOn (1.0, 0.0, 0.0, 1.0);
  gazebo::common::Color ledOff(0.2, 0.2, 0.2, 0.5);
  // LED:ON(1sec.) and OFF(1sec.)
  this->model_->SetAmbient(ledOn);
  this->model_->SetDiffuse(ledOn);
  std::this_thread::sleep_for (std::chrono::seconds(1));
  this->model_->SetAmbient(ledOff);
  this->model_->SetDiffuse(ledOff);
  std::this_thread::sleep_for (std::chrono::seconds(1));
  // LED:ON(1sec.) and OFF(1sec.)
  this->model_->SetAmbient(ledOn);
  this->model_->SetDiffuse(ledOn);
  std::this_thread::sleep_for (std::chrono::seconds(1));
  this->model_->SetAmbient(ledOff);
  this->model_->SetDiffuse(ledOff);
  std::this_thread::sleep_for (std::chrono::seconds(1));
  // LED:ON(1sec.) and OFF
  this->model_->SetAmbient(ledOn);
  this->model_->SetDiffuse(ledOn);
  std::this_thread::sleep_for (std::chrono::seconds(1));
  this->model_->SetAmbient(ledOff);
  this->model_->SetDiffuse(ledOff);
  res.success = true;
  res.status_message = "Finished the flashing led";
  return true; 
}

void LEDVisualPlugin::VisualQueueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    model_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(LEDVisualPlugin)

