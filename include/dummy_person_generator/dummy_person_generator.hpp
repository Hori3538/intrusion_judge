#ifndef DUMMY_PERSON_GENERATOR
#define DUMMY_PERSON_GENERATOR

#include "geometry_msgs/Pose.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

namespace dummy_person_generator
{
    struct Param
    {
        int hz;
        float start_x;
        float start_y;
        float end_y;
        float velocity;
    };

    class DummyPersonGenerator
    {
        public:
            DummyPersonGenerator(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void update_pose();
            void process();
        private:

            geometry_msgs::Pose current_pose_;
            Param param_;
            ros::Publisher person_poses_pub_;
    };

}
#endif
