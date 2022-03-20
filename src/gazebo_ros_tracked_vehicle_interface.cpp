#include <gazebo_ros_tracked_vehicle_interface/gazebo_ros_tracked_vehicle_interface.h>

#include <gazebo/transport/Node.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{

    // used as index number
    enum {
        RIGHT,
        LEFT,
    };
    GazeboRosTrackedVehicleInterface::GazeboRosTrackedVehicleInterface() {
        printf("Starting Tracked Vehicle Interface!\n");
        msgs::Set(cmd_vel_.mutable_linear(),
              ignition::math::Vector3d::Zero);
        msgs::Set(cmd_vel_.mutable_angular(),
              ignition::math::Vector3d::Zero);
    }

    // Destructor
    GazeboRosTrackedVehicleInterface::~GazeboRosTrackedVehicleInterface()
    {
        FiniChild();
    }

    void GazeboRosTrackedVehicleInterface::FiniChild()
    {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void GazeboRosTrackedVehicleInterface::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {

        this->parent = _parent;
        gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "TrackedVehicleInterface" ) );
        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();


        /* LOAD PARAMETERS FROM SDF */

        // parameters for publishing tf & odometry
        gazebo_ros_->getParameter<std::string> ( command_ros_topic_, "commandROSTopic", "cmd_vel_ros" );
        gazebo_ros_->getParameter<std::string> ( command_ign_topic_, "commandIGNTopic", "cmd_vel_ign" );
        gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
        gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
        gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
        gazebo_ros_->getParameter<std::string> ( track_speed_topic_, "trackSpeedTopic", "track_speed" );
        // gazebo_ros_->getParameter<std::string> ( robot_namespace_, "robot_namespace", _model->GetName() );

        this->ignition_node_.reset(new transport::Node());
  // Initialize the node with the world name
#if GAZEBO_MAJOR_VERSION >= 8
        this->ignition_node_->Init(_parent->GetWorld()->Name());
#else
        this->ignition_node_->Init(this->world_->GetName());
#endif

        // enable or disable publishing
        // here the joint state was omitted because tracked vehicle currently doesn't have wheel joints
        // but motor joint could later be added...
        gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);

        gazebo_ros_->getParameter<double> ( track_separation_, "wheelSeparation", 0.34 );
        gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );

        std::map<std::string, OdomSource> odomOptions;
        odomOptions["encoder"] = ENCODER;
        odomOptions["world"] = WORLD;
        gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

        this->publish_tf_ = true;
        if (!_sdf->HasElement("publishTf")) {
            ROS_WARN_NAMED("tracked_vehicle_interface", "GazeboRosTrackedVehicleInterface Plugin (ns = %s) missing <publishTf>, defaults to %d",
            this->robot_namespace_.c_str(), this->publish_tf_);
        } else {
            this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
        }


        if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent->GetWorld()->SimTime();
#else
        last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

        // Initialize velocity stuff
        track_speed_[RIGHT] = 0;
        track_speed_[LEFT] = 0;

        alive_ = true;

        /* SETUP OF ROS AND IGNITION PUB SUBS & CO */

        // ROS PUB SUB & CO

        transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ROS_INFO_NAMED("tracked_vehicle_interface", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_ros_topic_.c_str());

        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<geometry_msgs::Twist>(command_ros_topic_, 1,
                    boost::bind(&GazeboRosTrackedVehicleInterface::cmdVelCallback, this, _1),
                    ros::VoidPtr(), &queue_);

        cmd_vel_subscriber_ros_ = gazebo_ros_->node()->subscribe(so);
        ROS_INFO_NAMED("tracked_vehicle_interface", "%s: Subscribe to %s", gazebo_ros_->info(), command_ros_topic_.c_str());

        if (this->publish_tf_)
        {
            odometry_publisher_ros_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
            ROS_INFO_NAMED("tracked_vehicle_interface", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
        }

        // start custom queue for diff drive
        this->callback_queue_thread_ = boost::thread ( boost::bind ( &GazeboRosTrackedVehicleInterface::QueueThread, this ) );

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
            event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosTrackedVehicleInterface::Update, this ) );


        // IGNITION PUB SUB & CO
        this->tracks_vel_subscriber_ign_ =
            this->ignition_node_->Subscribe<msgs::Vector2d,GazeboRosTrackedVehicleInterface>(
                track_speed_topic_, 
                &GazeboRosTrackedVehicleInterface::OnTrackVelMsg, this);

        this->cmd_vel_publisher_ign_ = 
            this->ignition_node_->Advertise<msgs::Twist>(command_ign_topic_);
    }

    void GazeboRosTrackedVehicleInterface::OnTrackVelMsg(ConstVector2dPtr &_msg)
    {
        track_speed_[LEFT] = _msg->x();
        track_speed_[RIGHT] = _msg->y();
    }

    void GazeboRosTrackedVehicleInterface::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &_msg) {
        cmd_vel_.mutable_linear()->set_x(_msg->linear.x);
        cmd_vel_.mutable_angular()->set_z(-_msg->angular.z);

        this->cmd_vel_publisher_ign_->Publish(cmd_vel_);
    }

    void GazeboRosTrackedVehicleInterface::QueueThread()
    {
        static const double timeout = 0.01;

        while ( alive_ && gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
    }


    void GazeboRosTrackedVehicleInterface::Update() {
        if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent->GetWorld()->SimTime();
#else
        common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
        double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

        if ( seconds_since_last_update > update_period_ ) {
            if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
            last_update_time_+= common::Time ( update_period_ );
        }
    }

    void GazeboRosTrackedVehicleInterface::UpdateOdometryEncoder()
    {
        double vl = track_speed_[LEFT];
        double vr = track_speed_[RIGHT];
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = parent->GetWorld()->SimTime();
#else
        common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
        double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
        last_odom_update_ = current_time;

        double b = track_separation_;

        // Book: Sigwart 2011 Autonompus Mobile Robots page:337
        double sl = vl * seconds_since_last_update;
        double sr = vr * seconds_since_last_update;
        double ssum = sl + sr;

        double sdiff = sr - sl;

        double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
        double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
        double dtheta = ( sdiff ) /b;

        pose_encoder_.x += dx;
        pose_encoder_.y += dy;
        pose_encoder_.theta += dtheta;

        double w = dtheta/seconds_since_last_update;
        double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

        tf::Quaternion qt;
        tf::Vector3 vt;
        qt.setRPY ( 0,0,pose_encoder_.theta );
        vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        odom_.twist.twist.angular.z = w;
        odom_.twist.twist.linear.x = v;
        odom_.twist.twist.linear.y = 0;
    }

    void GazeboRosTrackedVehicleInterface::publishOdometry ( double step_time )
    {

        ros::Time current_time = ros::Time::now();
        std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
        std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

        tf::Quaternion qt;
        tf::Vector3 vt;

        if ( odom_source_ == ENCODER ) {
            // getting data form encoder integration
            qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
            vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

        }
        if ( odom_source_ == WORLD ) {
            // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d pose = parent->WorldPose();
#else
            ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
            qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
            vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

            odom_.pose.pose.position.x = vt.x();
            odom_.pose.pose.position.y = vt.y();
            odom_.pose.pose.position.z = vt.z();

            odom_.pose.pose.orientation.x = qt.x();
            odom_.pose.pose.orientation.y = qt.y();
            odom_.pose.pose.orientation.z = qt.z();
            odom_.pose.pose.orientation.w = qt.w();

            // get velocity in /odom frame
            ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
            linear = parent->WorldLinearVel();
            odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
            linear = parent->GetWorldLinearVel().Ign();
            odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

            // convert velocity to child_frame_id (aka base_footprint)
            float yaw = pose.Rot().Yaw();
            odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
            odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
        }

        if (publishOdomTF_ == true){
            tf::Transform base_footprint_to_odom ( qt, vt );
            transform_broadcaster_->sendTransform (
                tf::StampedTransform ( base_footprint_to_odom, current_time,
                                    odom_frame, base_footprint_frame ) );
        }


        // set covariance
        odom_.pose.covariance[0] = 0.00001;
        odom_.pose.covariance[7] = 0.00001;
        odom_.pose.covariance[14] = 1000000000000.0;
        odom_.pose.covariance[21] = 1000000000000.0;
        odom_.pose.covariance[28] = 1000000000000.0;
        odom_.pose.covariance[35] = 0.001;


        // set header
        odom_.header.stamp = current_time;
        odom_.header.frame_id = odom_frame;
        odom_.child_frame_id = base_footprint_frame;

        odometry_publisher_ros_.publish ( odom_ );
    }

    void GazeboRosTrackedVehicleInterface::Reset()
    {
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = parent->GetWorld()->SimTime();
#else
        last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
        pose_encoder_.x = 0;
        pose_encoder_.y = 0;
        pose_encoder_.theta = 0;
    }

    GZ_REGISTER_MODEL_PLUGIN ( GazeboRosTrackedVehicleInterface )
}