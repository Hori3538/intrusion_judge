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

        empty_border_.header.frame_id = "base_link";
        empty_border_.header.stamp = ros::Time::now();


        person_poses_sub_ = nh.subscribe<geometry_msgs::PoseArray>("person_poses", 1, &IntrusionJudge::pose_array_callback, this);
        off_limits_border_pub_ = nh.advertise<nav_msgs::Path>("/intrusion_judge/off_limits_border", 1);
        off_limits_border_intruded_pub_ = nh.advertise<nav_msgs::Path>("/intrusion_judge/off_limits_border_intruded", 1);
    }

    void IntrusionJudge::pose_array_callback(const geometry_msgs::PoseArrayConstPtr &msg)
    {
        //後々mapにlookupするtfを書く必要あり
        person_poses_ = *msg;

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
        off_limits_border.header.frame_id = "base_link";
        off_limits_border.header.stamp = ros::Time::now();
        
        geometry_msgs::PoseStamped origin;
        off_limits_border.poses.push_back(origin);

        std::vector<geometry_msgs::PoseStamped> arc;
        arc = calc_arc(adjust_yaw(trans_direction_ - param_.off_limits_angle_trans/2),
                    adjust_yaw(trans_direction_ + param_.off_limits_angle_trans/2),
                    param_.off_limits_radius_trans);
        off_limits_border.poses.insert(off_limits_border.poses.end(), arc.begin(), arc.end());

        off_limits_border.poses.push_back(origin);

        return off_limits_border;
    }

    nav_msgs::Path IntrusionJudge::calc_off_limits_border_turn()
    {
        nav_msgs::Path off_limits_border;
        off_limits_border.header.frame_id = "base_link";
        off_limits_border.header.stamp = ros::Time::now();
        
        std::vector<geometry_msgs::PoseStamped> arc;
        arc = calc_arc(-M_PI, M_PI, param_.off_limits_radius_turn);
        off_limits_border.poses.insert(off_limits_border.poses.end(), arc.begin(), arc.end());

        return off_limits_border;

    }

    void IntrusionJudge::visualize_off_limits_border()
    {
        nav_msgs::Path off_limits_border;
        if(turning_flag_) off_limits_border = calc_off_limits_border_turn();
        else off_limits_border = calc_off_limits_border_trans();

        if(intrusion_flag_)
        {
            off_limits_border_intruded_pub_.publish(off_limits_border);
            off_limits_border_pub_.publish(empty_border_);
        }
        else{
            off_limits_border_pub_.publish(off_limits_border);
            off_limits_border_intruded_pub_.publish(empty_border_);
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
        if(turning_flag_)
        {
            if(dist_origin_to_pose < param_.off_limits_radius_turn) return true;
            else return false;
        }
        else
        {
            bool angle_intrusion_flag = angle_intrusion_judge(
                    adjust_yaw(trans_direction_ - param_.off_limits_angle_trans/2),
                    adjust_yaw(trans_direction_ + param_.off_limits_angle_trans/2),
                    atan2(pose.position.y, pose.position.x));
            if(dist_origin_to_pose < param_.off_limits_radius_trans && angle_intrusion_flag) return true;
            else return false;

        }

    }

    void IntrusionJudge::process()
    {
        ros::Rate loop_rate(param_.hz);

        while (ros::ok())
        {
            intrusion_flag_ = intrusion_judge_pose_array(person_poses_);
            visualize_off_limits_border();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
