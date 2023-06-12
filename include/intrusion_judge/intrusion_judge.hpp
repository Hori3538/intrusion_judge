#ifndef INTRUSION_JUDGE
#define INTRUSION_JUDGE

#include "ros/subscriber.h"
#include <iterator>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>

namespace intrusion_judge
{
    struct Param
    {
        int hz;
        float off_limits_radius_trans;
        float off_limits_radius_turn;
        float off_limits_angle_trans;
        float border_angle_reso;
    };

    class IntrusionJudge
    {
        public:
            IntrusionJudge(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void process();
        private:
            void pose_array_callback(const geometry_msgs::PoseArrayConstPtr &msg);
            double adjust_yaw(double yaw);
            std::vector<geometry_msgs::PoseStamped> calc_arc(float start_angle, float end_angle, float radius);
            nav_msgs::Path calc_off_limits_border_trans();
            nav_msgs::Path calc_off_limits_border_turn();
            void visualize_off_limits_border();
            double calc_dist_origin_to_pose(geometry_msgs::Pose pose);
            bool angle_intrusion_judge(float start_angle, float end_angle, float target_angle);
            bool intrusion_judge_pose_array(geometry_msgs::PoseArray& pose_array);
            bool intrusion_judge_pose(geometry_msgs::Pose& pose);

            Param param_;

            bool turning_flag_ = false;
            bool intrusion_flag_ = false;
            float trans_direction_ = 0;
            nav_msgs::Path off_limits_border_; //使ってない
                                               //
            nav_msgs::Path empty_border_;

            geometry_msgs::PoseArray person_poses_;

            ros::Subscriber person_poses_sub_;
            ros::Publisher off_limits_border_pub_;
            ros::Publisher off_limits_border_intruded_pub_;
    };

}
#endif
