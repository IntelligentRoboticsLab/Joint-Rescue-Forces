// This came from a Gazebo tutrial [World Plugins](http://gazebosim.org/tutorials?cat=install&tut=plugins_world&ver=7%20-%208)

// Boost thread
#include <boost/thread.hpp>
#include <boost/bind.hpp>
// Boost random
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
// Boost XML Parse : Follow the class ObjectPreferences
#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Byte.h>
#include <ros/package.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <stdio.h>
#include <termios.h>
#include <iostream>

/*
    ROS_INFO("========================== %s\n", this->robot_namespace_.c_str());
    ROS_WARN("========================== %s\n", this->robot_namespace_.c_str());
    ROS_ERROR("========================== %s\n", this->robot_namespace_.c_str());
    gzmsg << "==========================" << this->robot_namespace_;
    gzwarn << "==========================" << this->robot_namespace_;
    gzerr << "==========================" << this->robot_namespace_;
    gzmsg << "===================================================================" << endl;
*/

using namespace gazebo;
using namespace std;
using namespace boost::property_tree;

class SpecialAreaPreferences
{
public:
  string     name;
  float      size_x, size_y, size_z;
  float      pose_x, pose_y, pose_z;
  // parseXML
  void parseXML(const ptree::value_type &xml)
  {
    name = xml.second.get<std::string>("name");
    size_x = xml.second.get<float>("size_x");
    size_y = xml.second.get<float>("size_y");
    size_z = xml.second.get<float>("size_z");
    pose_x = xml.second.get<float>("pose_x");
    pose_y = xml.second.get<float>("pose_y");
    pose_z = xml.second.get<float>("pose_z");
  }
  // print a preference
  void printOut()
  {
    gzmsg << "SPECIAL AREA name:" << name << "=======" << endl;
    gzmsg << "size x:" << size_x << " , y:" << size_y << " , z:" << size_z << endl;
    gzmsg << "pose x:" << pose_x << " , y:" << pose_y << " , z:" << pose_z << endl;
  }
};

void setSpecialAreaPreferences(std::vector<SpecialAreaPreferences>& SpclAreas_, string xmlfilename)
{
  SpecialAreaPreferences *sp = new SpecialAreaPreferences();
  ptree pt;
  read_xml(xmlfilename, pt);
  for(const ptree::value_type &specialarea: pt.get_child("root_specialAreaPreference"))
  {
    sp->parseXML(specialarea);
    SpclAreas_.push_back(*sp);
    sp->printOut();
  }
}

class InhibitAreaPreferences
{
public:
  string     name;
  float      size_x, size_y, size_z;
  float      pose_x, pose_y, pose_z;
  // parseXML
  void parseXML(const ptree::value_type &xml)
  {
    name = xml.second.get<std::string>("name");
    size_x = xml.second.get<float>("size_x");
    size_y = xml.second.get<float>("size_y");
    size_z = xml.second.get<float>("size_z");
    pose_x = xml.second.get<float>("pose_x");
    pose_y = xml.second.get<float>("pose_y");
    pose_z = xml.second.get<float>("pose_z");
  }
  // print a preference
  void printOut()
  {
    gzmsg << "INHIBIT AREA name:" << name << "=======" << endl;
    gzmsg << "size x:" << size_x << " , y:" << size_y << " , z:" << size_z << endl;
    gzmsg << "pose x:" << pose_x << " , y:" << pose_y << " , z:" << pose_z << endl;
  }
};

void setInhibitAreaPreferences(std::vector<InhibitAreaPreferences>& InhbAreas_, string xmlfilename)
{
  InhibitAreaPreferences *ip = new InhibitAreaPreferences();
  ptree pt;
  read_xml(xmlfilename, pt);
  for(const ptree::value_type &inhibitarea: pt.get_child("root_inhibitAreaPreference"))
  {
    ip->parseXML(inhibitarea);
    InhbAreas_.push_back(*ip);
    ip->printOut();
  }
}

class TrapAreaPreferences
{
public:
  string     name;
  float      size_x, size_y, size_z;
  float      pose_x, pose_y, pose_z;
  // parseXML
  void parseXML(const ptree::value_type &xml)
  {
    name = xml.second.get<std::string>("name");
    size_x = xml.second.get<float>("size_x");
    size_y = xml.second.get<float>("size_y");
    size_z = xml.second.get<float>("size_z");
    pose_x = xml.second.get<float>("pose_x");
    pose_y = xml.second.get<float>("pose_y");
    pose_z = xml.second.get<float>("pose_z");
  }
  // print a preference
  void printOut()
  {
    gzmsg << "Trap AREA name:" << name << "=======" << endl;
    gzmsg << "size x:" << size_x << " , y:" << size_y << " , z:" << size_z << endl;
    gzmsg << "pose x:" << pose_x << " , y:" << pose_y << " , z:" << pose_z << endl;
  }
};

void setTrapAreaPreferences(std::vector<TrapAreaPreferences>& TrapAreas_, string xmlfilename)
{
  TrapAreaPreferences *ip = new TrapAreaPreferences();
  ptree pt;
  read_xml(xmlfilename, pt);
  for(const ptree::value_type &Traparea: pt.get_child("root_trapAreaPreference"))
  {
    ip->parseXML(Traparea);
    TrapAreas_.push_back(*ip);
    ip->printOut();
  }
}

class ObjectPreferences
{
public:
  string     name;
  float      size_x, size_y, size_z;
  float      pose_x, pose_y, pose_z;
  string     color;
  int        point[2]; // 0:regular , 1:special
  physics::ModelPtr model;
  physics::WorldPtr theWorld;
  ignition::math::Pose3d init_pose, crnt_pose, next_pose;

  // parseXML
  void parseXML(const ptree::value_type &xml)
  {
    name = xml.second.get<std::string>("name");
    size_x = xml.second.get<float>("size_x");
    size_y = xml.second.get<float>("size_y");
    size_z = xml.second.get<float>("size_z");
    color = xml.second.get<std::string>("color");
    pose_x = xml.second.get<float>("pose_x");
    pose_y = xml.second.get<float>("pose_y");
    pose_z = xml.second.get<float>("pose_z");
    setPose(pose_x, pose_y, pose_z);
    init_pose = crnt_pose;
    point[0] = xml.second.get<int>("regular_point");
    point[1] = xml.second.get<int>("special_point");
  }
  // set pose_xyz and crnt_pose
  void setPose(float _x, float _y, float _z)
  {
    pose_x = _x;
    pose_y = _y;
    pose_z = _z;
    crnt_pose = ignition::math::Pose3d(ignition::math::Vector3d(pose_x, pose_y, pose_z), ignition::math::Quaterniond(0, 0, 0));
  }
  // reset pose
  void resetPose(void)
  {
    setPose(init_pose.Pos().X(), init_pose.Pos().Y(), init_pose.Pos().Z());
  }
  // print a preference
  void printOut()
  {
    gzmsg << "OBJECT name:" << name << "=======" << endl;
    gzmsg << "size x:" << size_x << " , y:" << size_y << " , z:" << size_z << endl;
    gzmsg << "color:" << color << endl;
    gzmsg << "Init pose x:" << init_pose.Pos().X() << " , y:" << init_pose.Pos().Y() << " , z:" << init_pose.Pos().Z() << endl;
    gzmsg << "regular point:" << point[0] << " , special point:" << point[1] << endl;
  }
  // spawn a object
  void Spawn_Box(physics::WorldPtr _parent, const string modelname
                                    , const string color
                                    , float mass
                                    , float sx, float sy, float sz
                                    , float x, float y, float z
                                    , float roll=0, float pitch=0, float yaw=0)
  {
    sdf::SDF tmp_modelSDF;
    char stringcmd[2048];
    sprintf(stringcmd,  
       "<sdf version ='1.6'>\
          <model name ='%s'>\
            <pose>%f %f %f %f %f %f</pose>\
            <link name ='%s_link'>\
              <inertial>\
                <mass>%f</mass>\
              </inertial>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>%f %f %f</size></box>\
                </geometry>\
                <surface>\
                  <friction>\
                    <ode>\
                      <mu>0.7</mu>\
                      <mu2>0.7</mu2>\
                      <slip1>0</slip1>\
                      <slip2>0</slip2>\
                    </ode>\
                  </friction>\
                </surface>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>%f %f %f</size></box>\
                </geometry>\
                <material>\
                  <script>\
                    <uri>file://media/materials/scripts/gazebo.material</uri>\
                    <name>Gazebo/%s</name>\
                  </script>\
                </material>\
              </visual>\
            </link>\
          </model>\
        </sdf>", modelname.c_str(), x, y, z, roll, pitch, yaw, modelname.c_str()
               , mass, sx, sy, sz, sx, sy, sz, color.c_str());
/* DO NOT ERASE! THIS IS STACK OF CODES!
                  <ambient>1 1 1 1</ambient>\
            <static>1</static>\
            <joint name='%s_joint' type='fixed'>\
              <parent>world</parent>\
              <child>%s_link</child>\
              <pose frame=''>0 0 0 0 -0 0</pose>\
              <physics>\
                <ode>\
                  <limit>\
                    <cfm>0</cfm>\
                    <erp>0.2</erp>\
                  </limit>\
                  <suspension>\
                    <cfm>0</cfm>\
                    <erp>0.2</erp>\
                  </suspension>\
                </ode>\
              </physics>\
            </joint>\
               , modelname.c_str()
               , modelname.c_str());

*/
//    gzerr << stringcmd << endl;
    tmp_modelSDF.SetFromString(stringcmd);
#if(GAZEBO_MAJOR_VERSION > 7)
    sdf::ElementPtr model = tmp_modelSDF.Root()->GetElement("model");
#else
    sdf::ElementPtr model = tmp_modelSDF.root->GetElement("model");
#endif
    model->GetAttribute("name")->SetFromString(modelname);
    _parent->InsertModelSDF(tmp_modelSDF);
  }

  void spawnObject(physics::WorldPtr _theWorld)
  {
    theWorld = _theWorld;
    Spawn_Box(theWorld, name, color, 0.1, size_x, size_y, size_z,
      pose_x, pose_y, pose_z, 0.0, 0.0, 0.0);
  }

  void putObject(void)
  {
//    gzmsg << name << ": " << x << ", " << y << ", " << z << endl;
    model = theWorld->ModelByName(name);
//    gzerr << "model = " << model << " : world = " << theWorld << "  " << _parent << endl;
    if(model)
      model->SetWorldPose(crnt_pose);
  }  
/*
  int IsInsideBox(float _x, float _y, auto& area, float inhibitMargin_)
  {
    if( (_x > area
  }
*/
  int CollisionInhibitAreas(const ObjectPreferences& OP
  , const std::vector<InhibitAreaPreferences>& InhbAreas_, float inhibitMargin_)
  {
    int ret = 0;
    for(auto &ia:InhbAreas_)
    {
      if(!(((OP.pose_x + OP.size_x/2) < (ia.pose_x - ia.size_x/2 - inhibitMargin_))
        || ((OP.pose_x - OP.size_x/2) > (ia.pose_x + ia.size_x/2 + inhibitMargin_))
        || ((OP.pose_y + OP.size_y/2) < (ia.pose_y - ia.size_y/2 - inhibitMargin_))
        || ((OP.pose_y - OP.size_y/2) > (ia.pose_y + ia.size_y/2 + inhibitMargin_))))
      {
        ret = 1;
        break;
      }
    }
//    gzerr << "Collision:INHB["<<OP.name<<"]"<<ret<<endl;
    return ret;
  }

  int CollisionTrapAreas(const ObjectPreferences& OP
  , const std::vector<TrapAreaPreferences>& TrapAreas_, float inhibitMargin_)
  {
    int ret = 0;    
    for(auto &ta:TrapAreas_)
    {
      if(!(((OP.pose_x + OP.size_x/2) < (ta.pose_x - ta.size_x/2 - inhibitMargin_))
        || ((OP.pose_x - OP.size_x/2) > (ta.pose_x + ta.size_x/2 + inhibitMargin_))
        || ((OP.pose_y + OP.size_y/2) < (ta.pose_y - ta.size_y/2 - inhibitMargin_))
        || ((OP.pose_y - OP.size_y/2) > (ta.pose_y + ta.size_y/2 + inhibitMargin_))))
      {
        ret = 1;
        break;
      }
    }
//    gzerr << "Collision:TRAP["<<OP.name<<"]"<<ret<<endl;
    return ret;
  }

  int CollisionOtherObjects(const ObjectPreferences& OP
  , const std::vector<ObjectPreferences>& ObjPrefs_, float inhibitMargin_)
  {
    int ret = 0;    
    for(auto &op:ObjPrefs_)
    {
      if(&OP == &op)
        continue;
      if(!(((OP.pose_x + OP.size_x/2) < (op.pose_x - op.size_x/2 - inhibitMargin_))
        || ((OP.pose_x - OP.size_x/2) > (op.pose_x + op.size_x/2 + inhibitMargin_))
        || ((OP.pose_y + OP.size_y/2) < (op.pose_y - op.size_y/2 - inhibitMargin_))
        || ((OP.pose_y - OP.size_y/2) > (op.pose_y + op.size_y/2 + inhibitMargin_))))
      {
        ret = 1;
        break;
      }
    }
//    gzerr << "Collision:OBJS["<<OP.name<<"]"<<ret<<endl;
    return ret;
  }

  void randomObject(float floorSizeX_, float floorSizeY_
  , const std::vector<InhibitAreaPreferences>& InhbAreas_
  , const std::vector<TrapAreaPreferences>& TrapAreas_
  , const std::vector<ObjectPreferences>& ObjPrefs_
  , float inhibitMargin_)
  {
    float x=0, y=0;
    boost::random::random_device seed_gen;
    boost::random::mt19937 gen(seed_gen);
    boost::random::uniform_01<double> Uniform01;
//  boost::random::uniform_real_distribution<double> Uniformf(0.0, 1.0);
//  boost::random::uniform_int_distribution<int> Uniformi(0, std::numeric_limits<int>::max());
//  boost::random::normal_distribution<double> Normal(0.0, 1.0);
    do
    {
      x = Uniform01(gen) * (floorSizeX_ - 2*inhibitMargin_ - size_x/2) 
                                        - (floorSizeX_ - 2*inhibitMargin_) / 2;
      y = Uniform01(gen) * (floorSizeY_ - 2*inhibitMargin_ - size_y/2)
                                        - (floorSizeY_ - 2*inhibitMargin_) / 2;
      setPose(x, y, init_pose.Pos().Z());
//      gzerr << "RAND[" << name << "] " << x << " , "  << y << endl;
    }while(  CollisionInhibitAreas(*this, InhbAreas_, inhibitMargin_)
          || CollisionTrapAreas   (*this, TrapAreas_, inhibitMargin_)
          || CollisionOtherObjects(*this, ObjPrefs_,  inhibitMargin_) );
  }  

  int InSpecialArea(float _x, float _y
  , const std::vector<SpecialAreaPreferences>& SpclAreas_)
  {
    int ret = 0;
    for(auto &sa:SpclAreas_)
    {
      if(  (_x >= (sa.pose_x - sa.size_x/2))
        && (_x <= (sa.pose_x + sa.size_x/2))
        && (_y >= (sa.pose_y - sa.size_y/2))
        && (_y <= (sa.pose_y + sa.size_y/2)) )
      {
        ret = 1;
        break;
      }
    }
    return ret;
  }
};

void setObjectPreferences(std::vector<ObjectPreferences>& ObjPrefs_, string xmlfilename)
{
  ObjectPreferences *op = new ObjectPreferences();
  ptree pt;
  read_xml(xmlfilename, pt);
  for(const ptree::value_type &objectpreference: pt.get_child("root_objectPreference"))
  {
    op->parseXML(objectpreference);
    ObjPrefs_.push_back(*op);
    op->printOut();
  }
}

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

class ObjectManager : public WorldPlugin
{
  public:
  physics::WorldPtr  theWorld_;
  sdf::ElementPtr    sdf_;
  transport::NodePtr node_;
  common::Time       simTime;
  ros::Subscriber    objmgr_subscriber_;
  float              inhibitMargin_;
  float              floorSizeX_, floorSizeY_;
//  void               objmgrCallback(const std_msgs::Byte::ConstPtr& objmgr_msg);

  // Arrays of data bases
  std::vector<ObjectPreferences>  ObjPrefs_;
  std::vector<SpecialAreaPreferences>  SpclAreas_;
  std::vector<InhibitAreaPreferences>  InhbAreas_;
  std::vector<TrapAreaPreferences>     TrapAreas_;

  // ROS Callback Queue
  ros::CallbackQueue queue_;
  GazeboRosPtr       gazebo_ros_;
  boost::mutex       lock_;
  boost::thread      callback_queue_thread_;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  std::string      topic_name_;
  std::string      robot_namespace_;
  std::string      packageName_;
  std::string      packagePath_;
  std::string      objectPreferences_file_;
  ros::Subscriber  pose_sub;  
  ros::Publisher   pose_pub;  

  ObjectManager()
  {
    ObjPrefs_.clear();
  }

  ~ObjectManager()
  {
    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    callback_queue_thread_.join();
    delete this->rosnode_;
  }

  void readParams(void)
  {
    this->robot_namespace_ = "ObjectManager";
    if(this->sdf_->HasElement("robotNamespace"))
    {
      this->robot_namespace_ = this->sdf_->Get<std::string>("robotNamespace");
    }

    this->topic_name_ = "cmd_objmgr";
    if(this->sdf_->HasElement("topicName"))
    {
      this->topic_name_ = this->sdf_->Get<std::string>("topicName");
    }
    this->topic_name_ = this->robot_namespace_ + "/" + this->topic_name_;

    this->packageName_ = "csrobot_ros";
    if(this->sdf_->HasElement("packageName"))
    {
      this->packageName_ = this->sdf_->Get<std::string>("packageName");
    }
    this->packagePath_ = ros::package::getPath(this->packageName_);
//    gzerr << this->packageName_ << endl;

    this->objectPreferences_file_ = "ObjectsPreferences.xml";
    if(this->sdf_->HasElement("preferencesFile"))
    {
      this->objectPreferences_file_ = this->sdf_->Get<std::string>("preferencesFile");
    }
    this->objectPreferences_file_ = this->packagePath_ + "/" + this->objectPreferences_file_;
//    gzerr << this->objectPreferences_file_ << endl;;

    this->inhibitMargin_ =0.05;
    if(this->sdf_->HasElement("inhibitMargin"))
    {
      this->inhibitMargin_ = this->sdf_->Get<float>("inhibitMargin");
    }

    this->floorSizeX_ =2.40;
    if(this->sdf_->HasElement("floorSizeX"))
    {
      this->floorSizeX_ = this->sdf_->Get<float>("floorSizeX");
    }

    this->floorSizeY_ =1.80;
    if(this->sdf_->HasElement("floorSizeY"))
    {
      this->floorSizeY_ = this->sdf_->Get<float>("floorSizeY");
    }

    // Preparing Objects
    setObjectPreferences(this->ObjPrefs_, this->objectPreferences_file_);

    // Preparing Special Areas
    setSpecialAreaPreferences(this->SpclAreas_, this->objectPreferences_file_);

    // Preparing Inhibit Areas
    setInhibitAreaPreferences(this->InhbAreas_, this->objectPreferences_file_);

    // Preparing Trap Areas
    setTrapAreaPreferences(this->TrapAreas_, this->objectPreferences_file_);
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    if(!_parent || !_sdf)
    {
      gzerr << "No world or SDF element specified. Plugin won't load." << std::endl;
      return;
    }
        
    this->theWorld_ = _parent;
    this->sdf_      = _sdf;
    this->node_     = transport::NodePtr(new transport::Node());
    this->node_->Init(this->theWorld_->Name());

    // Make sure the ROS node for Gazebo has already been initialized
    this->gazebo_ros_ = GazeboRosPtr(new GazeboRos(NULL, this->sdf_, "world"));
    this->gazebo_ros_->isInitialized();

    // Read Parameters
    readParams();

    // Spawn Objects
    for(auto &op:ObjPrefs_)
    {
      op.spawnObject(theWorld_);
//      op.printOut(); 
    }

    // Start custom queue for hearing ROS topics
    // Initialize the ROS node for the gazebo client if necessary
    if(!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, this->robot_namespace_, ros::init_options::NoSigintHandler);
    }
//    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
    this->rosnode_ = new ros::NodeHandle();
    // ROS: Registering Subscriber
    ros::SubscribeOptions soobjmgr =
      ros::SubscribeOptions::create<std_msgs::Byte>
        (this->topic_name_, 1, boost::bind(&ObjectManager::objmgrCallback, this, _1),
        ros::VoidPtr(), &this->queue_);
    this->objmgr_subscriber_ = this->rosnode_->subscribe(soobjmgr);
    this->callback_queue_thread_ = boost::thread(boost::bind(&ObjectManager::QueueThread, this));
  }

  void objmgrCallback(const std_msgs::Byte::ConstPtr& objmgr_msg)
  {
    boost::mutex::scoped_lock scoped_lock(lock_);
//    std::this_thread::sleep_for (std::chrono::seconds(2));
//    gzmsg << "===================================================================" << endl;
    for(auto &op:ObjPrefs_)
    {
      op.randomObject(this->floorSizeX_, this->floorSizeY_, this->InhbAreas_
      , this->TrapAreas_, this->ObjPrefs_, this->inhibitMargin_);
      op.putObject();
    }
  }

  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosnode_->ok())
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ObjectManager)

