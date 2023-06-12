#include <dummy_person_generator/dummy_person_generator.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_person_generator_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    dummy_person_generator::DummyPersonGenerator dummy_person_generator(nh, private_nh);

    dummy_person_generator.process();
    return 0;
}
