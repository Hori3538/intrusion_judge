#ifndef INTRUSION_JUDGE
#define INTRUSION_JUDGE

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <optional>

namespace intrusion_judge
{
    struct Param
    {
        int hz;
        float off_limits_radius_trans;
        float off_limits_radius_turn;
        float off_limits_angle_trans;
        float border_angle_reso;

        float moving_th_trans; // threshold for judging whether robot is stop in translation direction
        float moving_th_turn; // threshold for judging whether robot is stop rotating in direction

        std::string person_poses_topic_name;
        std::string cmd_vel_topic_name;
    };

    enum class MotionState
    {
        Stop,
        Trans,
        Turn
    };


    class IntrusionJudge
    {
        public:
            IntrusionJudge(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
            void process();
        private:
            void pose_array_callback(const geometry_msgs::PoseArrayConstPtr &msg);
            void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg);
            double adjust_yaw(double yaw);
            std::vector<geometry_msgs::PoseStamped> calc_arc(float start_angle, float end_angle, float radius);
            nav_msgs::Path calc_off_limits_border_trans();
            nav_msgs::Path calc_off_limits_border_turn();
            void visualize_off_limits_border();
            double calc_dist_origin_to_pose(geometry_msgs::Pose pose);
            bool angle_intrusion_judge(float start_angle, float end_angle, float target_angle);
            bool intrusion_judge_pose_array(geometry_msgs::PoseArray& pose_array);
            bool intrusion_judge_pose(geometry_msgs::Pose& pose);
            nav_msgs::Path generate_empty_border();
            MotionState judge_motion_state(geometry_msgs::Twist cmd_vel);
            float calc_trans_direction(geometry_msgs::Twist cmd_vel);
            void publish_intrusion_flag();

            Param param_;

            bool turning_flag_ = false;
            bool intrusion_flag_ = false;
            float trans_direction_ = 0;
            MotionState motion_state_ = MotionState::Stop;

            std::optional<geometry_msgs::PoseArray> person_poses_;
            std::optional<geometry_msgs::Twist> cmd_vel_;

            ros::Subscriber person_poses_sub_;
            ros::Subscriber cmd_vel_sub_;

            ros::Publisher off_limits_border_pub_;
            ros::Publisher off_limits_border_intruded_pub_;
            ros::Publisher intrusion_flag_pub_;
    };

}
#endif
