#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace gazebo
{
  class GazeboRosCreate : public ModelPlugin
  {
    public:
      GazeboRosCreate();
      virtual ~GazeboRosCreate();

      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();

    private:

      void UpdateSensors();
      void OnContact(ConstContactsPtr &contact);
      void OnCmdVel( const geometry_msgs::TwistStampedConstPtr &msg);


      /// Parameters
      std::string node_namespace_;
      std::string left_wheel_joint_name_;
      std::string right_wheel_joint_name_;
      std::string front_castor_joint_name_;
      std::string rear_castor_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;


      ros::NodeHandle *rosnode_;
      //ros::Service operating_mode_srv_;
      //ros::Service digital_output_srv_;

      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher odom_ground_truth_pub_; // odometry frame to world
      ros::Publisher ground_truth_pub_;      // base in world frame
      ros::Publisher joint_state_pub_;

      ros::Subscriber cmd_vel_sub_;

      physics::WorldPtr my_world_;
      physics::ModelPtr my_parent_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];

      bool set_joints_[4];
      physics::JointPtr joints_[4];
      physics::CollisionPtr base_collision_ptr_;

      sensors::RaySensorPtr left_cliff_sensor_;
      sensors::RaySensorPtr leftfront_cliff_sensor_;
      sensors::RaySensorPtr rightfront_cliff_sensor_;
      sensors::RaySensorPtr right_cliff_sensor_;
      sensors::RaySensorPtr wall_sensor_;
      double                cliff_range_;

      sensor_msgs::JointState js_;

      chris_create_node::CreateSensorState sensor_state_;

      void spin();
      boost::thread *spinner_thread_;

      transport::NodePtr gazebo_node_;
      transport::SubscriberPtr contact_sub_;

      // http://answers.gazebosim.org/question/12317/some-contacts-not-detected/
      transport::SubscriberPtr dummy_contact_sub_;
      void OnDummyContact(ConstContactsPtr &contact){};

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };
}
#endif
