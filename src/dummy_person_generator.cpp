#include "geometry_msgs/PoseArray.h"
#include <dummy_person_generator/dummy_person_generator.hpp>

namespace dummy_person_generator
{
    DummyPersonGenerator::DummyPersonGenerator(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        private_nh.param<int>("hz", param_.hz, 10);
        private_nh.param<float>("start_x", param_.start_x, 0.4);
        private_nh.param<float>("start_y", param_.start_y, -3.0);
        private_nh.param<float>("end_y", param_.end_y, 3.0);
        private_nh.param<float>("velocity", param_.velocity, 0.6);

        person_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("/person_poses", 1);

        current_pose_.position.x = param_.start_x;
        current_pose_.position.y = param_.start_y;
    }

    void DummyPersonGenerator::update_pose()
    {
        current_pose_.position.y += param_.velocity / param_.hz;
        if(current_pose_.position.y > param_.end_y) current_pose_.position.y = param_.start_y;
    }

    void DummyPersonGenerator::process()
    {
        ros::Rate loop_rate(param_.hz);

        while (ros::ok())
        {
            geometry_msgs::PoseArray person_poses;
            person_poses.header.frame_id = "base_link";
            person_poses.header.stamp = ros::Time::now();
            person_poses.poses.push_back(current_pose_);
            person_poses_pub_.publish(person_poses);

            update_pose();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
