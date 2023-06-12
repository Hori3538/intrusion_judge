#include <intrusion_judge/intrusion_judge.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "intrusion_judge_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    intrusion_judge::IntrusionJudge intrusion_judge(nh, private_nh);

    intrusion_judge.process();
    return 0;
}
