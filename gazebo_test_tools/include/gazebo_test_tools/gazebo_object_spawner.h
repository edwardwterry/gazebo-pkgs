#ifndef GAZEBO_TEST_TOOLS_GAZEBOOBJECTSPAWNER
#define GAZEBO_TEST_TOOLS_GAZEBOOBJECTSPAWNER

#include <ros/ros.h>

namespace gazebo_test_tools
{

/**
 * Spawns a object of given size, position and orientation into
 * Gazebo.
 *
 * \author Jennifer Buehler
 */
class GazeboObjectSpawner
{
  public:
    GazeboObjectSpawner(ros::NodeHandle &n);

    /**
     * \param isObject if true, spawn a object. If false, spawn cylinder,
     *      where \e w is the radius and \e h is the height (\e d will be ignored).
     */
    void spawnObject(const std::string &name, const std::string &frame_id,
                        float x, float y, float z, float qx, float qy, float qz, float qw);

  private:
    ros::NodeHandle nh;
    ros::ServiceClient spawn_object;
};

} // namespace gazebo_test_tools

#endif // GAZEBO_TEST_TOOLS_GAZEBOOBJECTSPAWNER
