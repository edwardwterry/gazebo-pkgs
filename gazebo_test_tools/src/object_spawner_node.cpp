#include <gazebo_test_tools/gazebo_object_spawner.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_object_spawner");

    if (argc < 5)
    {
        ROS_INFO("Usage: %s <name> [x y z] [frame_id].",argv[0]);
        return 0;
    }

    ros::NodeHandle node; 
    gazebo_test_tools::GazeboObjectSpawner spawner(node);

    ROS_INFO("Running spawn object once..");

    std::string name=argv[1];
    float x=0;
    float y=0;
    float z=0;

    std::string frame_id="world";
    if (argc>2) x=atof(argv[2]);
    if (argc>3) y=atof(argv[3]);
    if (argc>4) z=atof(argv[4]);
    if (argc>5) frame_id=argv[5];

    spawner.spawnObject(name,frame_id,x,y,z,0,0,0,1);
    return 0;
}
