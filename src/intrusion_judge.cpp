#include <intrusion_judge/intrusion_judge.hpp>

namespace intrusion_judge
{
    IntrusionJudge::IntrusionJudge(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<int>("hz", param_.hz, 10);
        private_nh.param<float>("off_limits_radius_trans", param_.off_limits_radius_trans, 1.0);
        private_nh.param<float>("off_limits_radius_turn", param_.off_limits_radius_turn, 0.6);
        private_nh.param<float>("off_limits_angle_trans", param_.off_limits_angle_trans, M_PI/2);
        private_nh.param<float>("border_angle_reso", param_.border_angle_reso, 0.1);
        private_nh.param<float>("moving_th_trans", param_.moving_th_trans, 0.05);
        private_nh.param<float>("moving_th_turn", param_.moving_th_turn, 0.05);
        private_nh.param<std::string>("person_poses_topic_name", param_.person_poses_topic_name, "/person_poses");
        private_nh.param<std::string>("cmd_vel_topic_name", param_.cmd_vel_topic_name, "/cmd_vel");
        private_nh.param<std::string>("base_frame", param_.base_frame, "base_link");
        private_nh.param<std::string>("world_frame", param_.world_frame, "map");

        person_poses_sub_ = nh.subscribe<geometry_msgs::PoseArray>(param_.person_poses_topic_name, 1, &IntrusionJudge::pose_array_callback, this);
        cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>(param_.cmd_vel_topic_name, 1, &IntrusionJudge::cmd_vel_callback, this);
        off_limits_border_pub_ = nh.advertise<nav_msgs::Path>("/intrusion_judge/off_limits_border", 1);
        off_limits_border_intruded_pub_ = nh.advertise<nav_msgs::Path>("/intrusion_judge/off_limits_border_intruded", 1);
        intrusion_flag_pub_ = nh.advertise<std_msgs::Bool>("/intrusion_judge/intrusion_flag", 1);
    }

    void IntrusionJudge::pose_array_callback(const geometry_msgs::PoseArrayConstPtr &msg)
    {
        person_poses_ = *msg;
        try
        {
            geometry_msgs::TransformStamped transform;
            // target frame: base_frame, source frame: world_frame
            transform = tf_buffer_.lookupTransform(param_.base_frame, param_.world_frame, ros::Time(0));
            for(auto& person_pose: person_poses_.value().poses)
                tf2::doTransform(person_pose, person_pose, transform);
            person_poses_.value().header.frame_id = param_.base_frame;
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            person_poses_.reset();
        }
    }

    void IntrusionJudge::cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
    {
        cmd_vel_ = *msg;
    }

    double IntrusionJudge::adjust_yaw(double yaw)
    {
        if(yaw > M_PI){yaw -= 2*M_PI;}
        if(yaw < -M_PI){yaw += 2*M_PI;}

        return yaw;
    }

    std::vector<geometry_msgs::PoseStamped> IntrusionJudge::calc_arc(float start_angle, float end_angle, float radius)
    {
        std::vector<geometry_msgs::PoseStamped> arc;
        for(float angle=start_angle; angle<end_angle; angle+=param_.border_angle_reso)
        {
            geometry_msgs::PoseStamped point;
            point.pose.position.x = radius * cos(adjust_yaw(angle));
            point.pose.position.y = radius * sin(adjust_yaw(angle));
            arc.push_back(point);
        }

        return arc;
    }

    nav_msgs::Path IntrusionJudge::calc_off_limits_border_trans()
    {
        nav_msgs::Path off_limits_border;
        off_limits_border.header.frame_id = param_.base_frame;
        off_limits_border.header.stamp = ros::Time::now();
        
        geometry_msgs::PoseStamped origin;
        off_limits_border.poses.push_back(origin);

        std::vector<geometry_msgs::PoseStamped> arc;
        arc = calc_arc(trans_direction_ - param_.off_limits_angle_trans/2,
                    trans_direction_ + param_.off_limits_angle_trans/2,
                    param_.off_limits_radius_trans);
        off_limits_border.poses.insert(off_limits_border.poses.end(), arc.begin(), arc.end());

        off_limits_border.poses.push_back(origin);

        return off_limits_border;
    }

    nav_msgs::Path IntrusionJudge::calc_off_limits_border_turn()
    {
        nav_msgs::Path off_limits_border;
        off_limits_border.header.frame_id = param_.base_frame;
        off_limits_border.header.stamp = ros::Time::now();
        
        std::vector<geometry_msgs::PoseStamped> arc;
        arc = calc_arc(-M_PI, M_PI, param_.off_limits_radius_turn);
        off_limits_border.poses.insert(off_limits_border.poses.end(), arc.begin(), arc.end());

        return off_limits_border;

    }

    void IntrusionJudge::visualize_off_limits_border()
    {
        nav_msgs::Path off_limits_border;
        switch(motion_state_)
        {
            case MotionState::Stop:
                off_limits_border = generate_empty_border();
                break;
            case MotionState::Trans:
                off_limits_border = calc_off_limits_border_trans();
                break;
            case MotionState::Turn:
                off_limits_border = calc_off_limits_border_turn();
                break;
        }

        if(intrusion_flag_)
        {
            off_limits_border_intruded_pub_.publish(off_limits_border);
            off_limits_border_pub_.publish(generate_empty_border());
        }
        else{
            off_limits_border_pub_.publish(off_limits_border);
            off_limits_border_intruded_pub_.publish(generate_empty_border());
        }
    }


    double IntrusionJudge::calc_dist_origin_to_pose(geometry_msgs::Pose pose)
    {
        return std::sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2));
    }

    bool IntrusionJudge::angle_intrusion_judge(float start_angle, float end_angle, float target_angle)
    {
        if(target_angle > start_angle && target_angle < end_angle) return true;
        else return false;
    }

    bool IntrusionJudge::intrusion_judge_pose_array(geometry_msgs::PoseArray& pose_array)
    {
        for(auto& pose: pose_array.poses)
            if(intrusion_judge_pose(pose)) return true;
        return false;
    }

    bool IntrusionJudge::intrusion_judge_pose(geometry_msgs::Pose& pose)
    {
        double dist_origin_to_pose = calc_dist_origin_to_pose(pose);
        switch(motion_state_)
        {
            case MotionState::Stop:
                return false;
                break;

            case MotionState::Trans:
                {
                    bool angle_intrusion_flag = angle_intrusion_judge(
                            adjust_yaw(trans_direction_ - param_.off_limits_angle_trans/2),
                            adjust_yaw(trans_direction_ + param_.off_limits_angle_trans/2),
                            atan2(pose.position.y, pose.position.x));
                    if(dist_origin_to_pose < param_.off_limits_radius_trans && angle_intrusion_flag) return true;
                    else return false;
                } // blockないとcompile error
                break;

            case MotionState::Turn:
                if(dist_origin_to_pose < param_.off_limits_radius_turn) return true;
                else return false;
                break;
            default:
                return false;
                break;
        }

    }

    nav_msgs::Path IntrusionJudge::generate_empty_border()
    {
        nav_msgs::Path empty_border;
        empty_border.header.frame_id = param_.base_frame;
        empty_border.header.stamp = ros::Time::now();

        return empty_border;
    }

    MotionState IntrusionJudge::judge_motion_state(geometry_msgs::Twist cmd_vel)
    {
        if(sqrt(std::pow(cmd_vel.linear.x, 2) + std::pow(cmd_vel.linear.y, 2)) > param_.moving_th_trans)
            return MotionState::Trans;

        if(abs(cmd_vel.angular.z) > param_.moving_th_turn)
            return MotionState::Turn;

        return MotionState::Stop;
    }

    float IntrusionJudge::calc_trans_direction(geometry_msgs::Twist cmd_vel)
    {
        return atan2(cmd_vel.linear.y, cmd_vel.linear.x);
    }

    void IntrusionJudge::publish_intrusion_flag()
    {
        std_msgs::Bool intrusion_flag_msg;
        intrusion_flag_msg.data = intrusion_flag_;
        intrusion_flag_pub_.publish(intrusion_flag_msg);
    }

    void IntrusionJudge::process()
    {
        ros::Rate loop_rate(param_.hz);
        tf2_ros::TransformListener tf_listener(tf_buffer_);

        while (ros::ok())
        {
            if(person_poses_.has_value() && cmd_vel_.has_value())
            {
                motion_state_ = judge_motion_state(cmd_vel_.value());
                if(motion_state_ == MotionState::Trans) trans_direction_ = calc_trans_direction(cmd_vel_.value()); 
                else trans_direction_ = 0;

                intrusion_flag_ = intrusion_judge_pose_array(person_poses_.value());
                visualize_off_limits_border();
                publish_intrusion_flag();
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
