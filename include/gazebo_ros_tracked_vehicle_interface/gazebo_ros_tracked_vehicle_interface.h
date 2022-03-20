#ifndef TRACKED_VEHICLE_INTERFACE_PLUGIN_HH
#define TRACKED_VEHICLE_INTERFACE_PLUGIN_HH

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/msgs/twist.pb.h"


// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {
    class GazeboRosTrackedVehicleInterface : public ModelPlugin {

        enum OdomSource
        {
            ENCODER = 0,
            WORLD = 1,
        };

        public:
            GazeboRosTrackedVehicleInterface();
            ~GazeboRosTrackedVehicleInterface();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void Reset();
        private:
            void publishOdometry(double step_time);
            void getWheelVelocities();
            void publishWheelJointState();
            void UpdateOdometryEncoder();

            GazeboRosPtr gazebo_ros_;
            transport::NodePtr ignition_node_;
            physics::ModelPtr parent;
            physics::WorldPtr world_;

            // global variables for storing parameters
            std::string robot_namespace_;
            std::string command_ros_topic_;
            std::string command_ign_topic_;
            std::string odometry_topic_;
            std::string odometry_frame_;
            std::string track_speed_topic_;
            std::string robot_base_frame_;
            double track_separation_;

            // Flags
            bool publish_tf_;
            bool publishOdomTF_;
            bool alive_;


            // ros transport
            ros::Publisher odometry_publisher_ros_;
            ros::Subscriber cmd_vel_subscriber_ros_;
            boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

            ros::CallbackQueue queue_;
            boost::thread callback_queue_thread_;
            void QueueThread();

            // ignition transport
            transport::PublisherPtr cmd_vel_publisher_ign_;
            transport::SubscriberPtr tracks_vel_subscriber_ign_;
            event::ConnectionPtr update_connection_;
        
            double update_rate_;
            double update_period_;
            common::Time last_update_time_;

            OdomSource odom_source_;

            void OnTrackVelMsg (ConstVector2dPtr &_msg);
            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &_msg);

            double track_speed_[2];
            nav_msgs::Odometry odom_;
            msgs::Twist cmd_vel_;
            geometry_msgs::Pose2D pose_encoder_;
            common::Time last_odom_update_;

        protected:
            virtual void Update();
            virtual void FiniChild();
    };
}

#endif