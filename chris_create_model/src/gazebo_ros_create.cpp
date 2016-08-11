#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <chris_create_node/CreateSensorState.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include "chris_create_model/gazebo_ros_create.h"

using namespace gazebo;

enum {LEFT= 0, RIGHT=1, FRONT=2, REAR=3};

GazeboRosCreate::GazeboRosCreate()
  : gazebo_node_(new transport::Node())
{
  this->spinner_thread_ = new boost::thread( boost::bind( &GazeboRosCreate::spin, this) );

  wheel_speed_ = new float[2];
  wheel_speed_[LEFT] = 0.0;
  wheel_speed_[RIGHT] = 0.0;

  set_joints_[0] = false;
  set_joints_[1] = false;
  set_joints_[2] = false;
  set_joints_[3] = false;
  joints_[0].reset();
  joints_[1].reset();
  joints_[2].reset();
  joints_[3].reset();
  gazebo_node_->Init("default");
}

GazeboRosCreate::~GazeboRosCreate()
{
  rosnode_->shutdown();
  this->spinner_thread_->join();
  delete this->spinner_thread_;
  delete [] wheel_speed_;
  delete rosnode_;
}

void GazeboRosCreate::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->my_world_ = _parent->GetWorld();

  this->my_parent_ = _parent;
  if (!this->my_parent_)
  {
    ROS_FATAL("Gazebo_ROS_Create controller requires a Model as its parent");
    return;
  }


  this->node_namespace_ = "";
  if (_sdf->HasElement("node_namespace"))
    this->node_namespace_ = _sdf->GetElement("node_namespace")->Get<std::string>() + "/";


  left_wheel_joint_name_ = "left_wheel_joint";
  if (_sdf->HasElement("left_wheel_joint"))
    left_wheel_joint_name_ = _sdf->GetElement("left_wheel_joint")->Get<std::string>();

  right_wheel_joint_name_ = "right_wheel_joint";
  if (_sdf->HasElement("right_wheel_joint"))
    right_wheel_joint_name_ = _sdf->GetElement("right_wheel_joint")->Get<std::string>();

  front_castor_joint_name_ = "front_castor_joint";
  if (_sdf->HasElement("front_castor_joint"))
    front_castor_joint_name_ = _sdf->GetElement("front_castor_joint")->Get<std::string>();

  rear_castor_joint_name_ = "rear_castor_joint";
  if (_sdf->HasElement("rear_castor_joint"))
    rear_castor_joint_name_ = _sdf->GetElement("rear_castor_joint")->Get<std::string>();

  wheel_sep_ = 0.34;
  if (_sdf->HasElement("wheel_separation"))
    wheel_sep_ = _sdf->GetElement("wheel_separation")->Get<double>();

  wheel_sep_ = 0.34;
  if (_sdf->HasElement("wheel_separation"))
    wheel_sep_ = _sdf->GetElement("wheel_separation")->Get<double>();

  wheel_diam_ = 0.15;
  if (_sdf->HasElement("wheel_diameter"))
    wheel_diam_ = _sdf->GetElement("wheel_diameter")->Get<double>();

  torque_ = 10.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->Get<double>();

  // Get the mapping base
  base_geom_name_ = "base_footprint";
  if (_sdf->HasElement("base_geom"))
  {
    base_geom_name_ = _sdf->GetElement("base_geom")->Get<std::string>();
    ROS_WARN(" Using base_geom_name=<%s> from sdf",base_geom_name_.c_str());
  }
  physics::BasePtr test_base = my_parent_->GetChild(base_geom_name_);
  if (!test_base)
  {
      ROS_ERROR("Unable to find geom[%s]",base_geom_name_.c_str());
      return;
  }

  // Get the collision pointer
  std::string base_collision_name = base_geom_name_+"_collision";
  base_collision_ptr_ = my_parent_->GetChildCollision(base_collision_name);
  if (!base_collision_ptr_)
  {
        ROS_ERROR("Unable to find collision geometry[%s]",base_collision_name.c_str());
          for (int i=0; i < my_parent_->GetChildCount();i++)
          {
              std::cout << "child " << i<< std::endl;
              physics::BasePtr child = my_parent_->GetChild(i);
              std::string cn = child->GetName();

              std::cout << "   " << i << "  " << cn << " type=" << child->GetType();
              if (physics::CollisionPtr col = my_parent_->GetChildCollision(cn))
              {
                  std::cout << " collision=" << col->GetName();
              }
              std::cout << std::endl;
              for (int j=0; j < child->GetChildCount();j++)
              {
                  physics::BasePtr grandchild = child->GetChild(j);
                  std::string gcn = grandchild->GetName();
                  std::cout << "      j=" << j << " " << gcn << " type=" << child->GetType() ;
                  if (physics::CollisionPtr col = my_parent_->GetChildCollision(gcn))
                  {
                      std::cout << " collision=" << col->GetName();
                  }
                  std::cout << std::endl;
              }
          }
        return;
  }

  //base_geom_->SetContactsEnabled(true);
  //contact_event_ = base_geom_->ConnectContact(boost::bind(&GazeboRosCreate::OnContact, this, _1, _2));
  physics::ContactManager *mgr = my_world_->GetPhysicsEngine()->GetContactManager();

  // Get all the collision elements
  std::string entityName =  this->my_parent_->GetScopedName();
  std::string filterName = entityName + "::" + base_geom_name_+"::"+ base_collision_name;
  ROS_INFO("    contact filter name<%s>",filterName.c_str());

  std::string topic = mgr->CreateFilter(filterName, filterName);
  contact_sub_ = gazebo_node_->Subscribe(topic, &GazeboRosCreate::OnContact, this);

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  wall_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("wall_sensor"));
  if (!wall_sensor_)
  {
    ROS_ERROR("Unable to find sensor[wall_sensor]");
    return;
  }

  cliff_range_ = 0.03;
  if (_sdf->HasElement("cliff_range"))
  {
    cliff_range_ = _sdf->GetElement("cliff_range")->Get<double>();
    ROS_WARN(" Using cliff_range=%f from sdf",cliff_range_);
  }

  left_cliff_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("left_cliff_sensor"));
  right_cliff_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("right_cliff_sensor"));
  leftfront_cliff_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("leftfront_cliff_sensor"));
  rightfront_cliff_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(
    sensors::SensorManager::Instance()->GetSensor("rightfront_cliff_sensor"));

  wall_sensor_->SetActive(true);
  left_cliff_sensor_->SetActive(true);
  right_cliff_sensor_->SetActive(true);
  rightfront_cliff_sensor_->SetActive(true);
  leftfront_cliff_sensor_->SetActive(true);

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_create", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &GazeboRosCreate::OnCmdVel, this );

  sensor_state_pub_      = rosnode_->advertise<chris_create_node::CreateSensorState>("sensor_state", 1);
  odom_pub_              = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
  odom_ground_truth_pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom_ground_truth", 1);
  ground_truth_pub_      = rosnode_->advertise<nav_msgs::Odometry>("ground_truth", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

  js_.name.push_back( left_wheel_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( right_wheel_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( front_castor_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( rear_castor_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  prev_update_time_ = 0;
  last_cmd_vel_time_ = 0;


  // Initialize bumpers
  sensor_state_.bumps_wheeldrops = 0x0;

  joints_[LEFT] = my_parent_->GetJoint(left_wheel_joint_name_);
  joints_[RIGHT] = my_parent_->GetJoint(right_wheel_joint_name_);
  joints_[FRONT] = my_parent_->GetJoint(front_castor_joint_name_);
  joints_[REAR] = my_parent_->GetJoint(rear_castor_joint_name_);

  if (joints_[LEFT]) set_joints_[LEFT] = true;
  if (joints_[RIGHT]) set_joints_[RIGHT] = true;
  if (joints_[FRONT]) set_joints_[FRONT] = true;
  if (joints_[REAR]) set_joints_[REAR] = true;

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->my_world_->GetSimTime();
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosCreate::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";
}


void GazeboRosCreate::OnContact(ConstContactsPtr &contacts)
{
  const float y_overlap = 0.16495 * cos( 10 * (M_PI/180.0) ); // don't trigger bumper on glancing blow

  if (contacts->contact_size() < 1) return; // no contacts to check

  math::Pose  world = my_parent_->GetWorldPose();
  math::Quaternion qi = world.rot.GetInverse();
  //bool not_contacted = true; // flag to print header only on at least one bumper contact

  for (int i=0; i < contacts->contact_size(); i++ )
  {
    const msgs::Contact & contact = contacts->contact(i);
    const int contact_count = contact.position_size();
    for (unsigned int j=0; j < contact_count; j++)
    {

      // Calculate the body centric contact point
      math::Vector3 dV = math::Vector3(contact.position(j).x(),contact.position(j).y(),contact.position(j).z()) - world.pos;

      dV = qi*dV;


      // Make sure the contact is on the front bumper
      if (dV.x > 0.012 && dV.z < 0.06 && dV.z > 0.01)
      {
//          if (not_contacted)
//          {
//              std::cout << "   Create::OnContact with " << i << " of "<< contacts->contact_size() << std::endl;
//              not_contacted = false;
//          }
//          std::cout << "      Create::OnContact with " << j << " of "<< contact_count << " World posn=" ;
//          std::cout << "   ( " << contact.position(j).x() << ", " << contact.position(j).y() << ", " << contact.position(j).z() ;
//          std::cout << "  ) body=(" << dV.x << ", " << dV.y << ", " << dV.z ;
//          std::cout << " )";

          if (fabs(dV.y) < 0.2*y_overlap)
          {
              sensor_state_.bumps_wheeldrops |= 0x3; // trigger both on front impact
          }
          else
          {
            // Right bump sensor
            if (dV.y >= -y_overlap && dV.y < 0.0)
                sensor_state_.bumps_wheeldrops |= 0x1;
            // Left bump sensor
            if (dV.y <= y_overlap && dV.y > 0.0)
                sensor_state_.bumps_wheeldrops |= 0x2;
          }
//        std::cout << "  - bumps=" << (int)sensor_state_.bumps_wheeldrops << " !" << std::endl;
      }
    }
  }

}

void GazeboRosCreate::UpdateChild()
{
  common::Time time_now = this->my_world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  double wd, ws;
  double d1, d2;
  double dr, da;

  wd = wheel_diam_;
  ws = wheel_sep_;

  d1 = d2 = 0;
  dr = da = 0;

  // Distance travelled by front wheels
  if (set_joints_[LEFT])
    d1 = step_time.Double() * (wd / 2) * joints_[LEFT]->GetVelocity(0);
  if (set_joints_[RIGHT])
    d2 = step_time.Double() * (wd / 2) * joints_[RIGHT]->GetVelocity(0);

  // Can see NaN values here, just zero them out if needed
  if (std::isnan(d1)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Create plugin. NaN in d1. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[LEFT]->GetVelocity(0));
    d1 = 0;
  }

  if (std::isnan(d2)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Create plugin. NaN in d2. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[RIGHT]->GetVelocity(0));
    d2 = 0;
  }

  dr = (d1 + d2) / 2;
  da = (d2 - d1) / ws;

  // Compute odometric pose
  odom_pose_[0] += dr * cos( odom_pose_[2] );
  odom_pose_[1] += dr * sin( odom_pose_[2] );
  odom_pose_[2] += da;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = dr / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = da / step_time.Double();


  if (set_joints_[LEFT])
  {
    joints_[LEFT]->SetVelocity( 0, wheel_speed_[LEFT] / (wd/2.0) );
    joints_[LEFT]->SetEffortLimit(0, torque_ );
  }
  if (set_joints_[RIGHT])
  {
    joints_[RIGHT]->SetVelocity( 0, wheel_speed_[RIGHT] / (wd / 2.0) );
    joints_[RIGHT]->SetEffortLimit(0, torque_ );
  }

  nav_msgs::Odometry odom;
  odom.header.stamp.sec = time_now.sec;
  odom.header.stamp.nsec = time_now.nsec;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = base_geom_name_;
  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setEuler(0,0,odom_pose_[2]);

  odom.pose.pose.orientation.x = qt.getX();
  odom.pose.pose.orientation.y = qt.getY();
  odom.pose.pose.orientation.z = qt.getZ();
  odom.pose.pose.orientation.w = qt.getW();

  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy( &odom.pose.covariance[0], pose_cov, sizeof(double)*36 );
  memcpy( &odom.twist.covariance[0], pose_cov, sizeof(double)*36 );

  odom.twist.twist.linear.x = odom_vel_[0];
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = odom_vel_[2];

  odom_pub_.publish( odom );

  nav_msgs::Odometry odom_ground_truth = odom;
  odom_ground_truth.header.frame_id = "world";
  odom_ground_truth.child_frame_id  = "odom";
  math::Pose  world = my_parent_->GetWorldPose();
  math::Pose  odom2world = world*math::Pose(gazebo::math::Vector3(odom_pose_[0],odom_pose_[1], 0.0),
                                            gazebo::math::Quaternion(odom.pose.pose.orientation.w,odom.pose.pose.orientation.x,
                                                                     odom.pose.pose.orientation.y,odom.pose.pose.orientation.z)).GetInverse();
  odom_ground_truth.pose.pose.position.x = odom2world.pos.x;
  odom_ground_truth.pose.pose.position.y = odom2world.pos.y;
  odom_ground_truth.pose.pose.position.z = odom2world.pos.z;

  odom_ground_truth.pose.pose.orientation.x = odom2world.rot.x;
  odom_ground_truth.pose.pose.orientation.y = odom2world.rot.y;
  odom_ground_truth.pose.pose.orientation.z = odom2world.rot.z;
  odom_ground_truth.pose.pose.orientation.w = odom2world.rot.w;
  odom_ground_truth_pub_.publish( odom_ground_truth );

  nav_msgs::Odometry base_ground_truth = odom;
  base_ground_truth.header.frame_id = "map";
  base_ground_truth.child_frame_id  = base_geom_name_;

  base_ground_truth.pose.pose.position.x = world.pos.x;
  base_ground_truth.pose.pose.position.y = world.pos.y;
  base_ground_truth.pose.pose.position.z = world.pos.z;

  base_ground_truth.pose.pose.orientation.x = world.rot.x;
  base_ground_truth.pose.pose.orientation.y = world.rot.y;
  base_ground_truth.pose.pose.orientation.z = world.rot.z;
  base_ground_truth.pose.pose.orientation.w = world.rot.w;
  ground_truth_pub_.publish( base_ground_truth );


  js_.header.stamp.sec = time_now.sec;
  js_.header.stamp.nsec = time_now.nsec;
  if (this->set_joints_[LEFT])
  {
    js_.position[0] = joints_[LEFT]->GetAngle(0).Radian();
    js_.velocity[0] = joints_[LEFT]->GetVelocity(0);
  }

  if (this->set_joints_[RIGHT])
  {
    js_.position[1] = joints_[RIGHT]->GetAngle(0).Radian();
    js_.velocity[1] = joints_[RIGHT]->GetVelocity(0);
  }

  if (this->set_joints_[FRONT])
  {
    js_.position[2] = joints_[FRONT]->GetAngle(0).Radian();
    js_.velocity[2] = joints_[FRONT]->GetVelocity(0);
  }

  if (this->set_joints_[REAR])
  {
    js_.position[3] = joints_[REAR]->GetAngle(0).Radian();
    js_.velocity[3] = joints_[REAR]->GetVelocity(0);
  }

  joint_state_pub_.publish( js_ );

  this->UpdateSensors();

  //timeout if didn't receive cmd in a while
  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
  if (time_since_last_cmd.Double() > 0.6)
  {
    wheel_speed_[LEFT] = 0;
    wheel_speed_[RIGHT] = 0;
  }

}

void GazeboRosCreate::UpdateSensors()
{
  if (wall_sensor_->Range(0) < 0.04)
    sensor_state_.wall = true;
  else
    sensor_state_.wall = false;

  if (left_cliff_sensor_->Range(0) > cliff_range_)
    sensor_state_.cliff_left = true;
  else
    sensor_state_.cliff_left = false;

  if (right_cliff_sensor_->Range(0) > cliff_range_)
    sensor_state_.cliff_right = true;
  else
    sensor_state_.cliff_right = false;

  if (rightfront_cliff_sensor_->Range(0) > cliff_range_)
    sensor_state_.cliff_front_right = true;
  else
    sensor_state_.cliff_front_right = false;

  if (leftfront_cliff_sensor_->Range(0) > cliff_range_)
    sensor_state_.cliff_front_left = true;
  else
    sensor_state_.cliff_front_left = false;

  sensor_state_pub_.publish(sensor_state_);

  // Reset the bump sensors after publishing
  sensor_state_.bumps_wheeldrops = 0x0;
}

void GazeboRosCreate::OnCmdVel( const geometry_msgs::TwistStampedConstPtr &msg)
{
  last_cmd_vel_time_ = this->my_world_->GetSimTime();
  double vr, va;
  vr = msg->twist.linear.x;
  va = msg->twist.angular.z;

  wheel_speed_[LEFT] = vr - va * (wheel_sep_) / 2;
  wheel_speed_[RIGHT] = vr + va * (wheel_sep_) / 2;
}

void GazeboRosCreate::spin()
{
  while(ros::ok()) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosCreate);

