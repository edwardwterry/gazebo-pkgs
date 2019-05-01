#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>

#include <gazebo_test_tools/gazebo_object_spawner.h>

#include <sstream>

using gazebo_test_tools::GazeboObjectSpawner;
using ros::NodeHandle;

#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"

GazeboObjectSpawner::GazeboObjectSpawner(NodeHandle &n) : nh(n)
{
  spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}

void GazeboObjectSpawner::spawnObject(const std::string &name, const std::string &frame_id,
                                      float x, float y, float z, float qx, float qy, float qz, float qw)
{

  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  gazebo_msgs::SpawnModel spawn;
  spawn.request.model_name = name;

  std::stringstream s;

  s << "<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"
    << name << "'>\
      <link name='link'>\
        <inertial>\
          <inertia>\
            <ixx>0.0133113</ixx>\
            <ixy>-0.00030365</ixy>\
            <ixz>-0.00034148</ixz>\
            <iyy>0.0115766</iyy>\
            <iyz>0.00088073</iyz>\
            <izz>0.00378028</izz>\
          </inertia>\
          <mass>1.50252</mass>\
        </inertial>\
        <collision name='collision'>\
          <pose frame=''>0 0 0 0 -0 0</pose>\ 
          <geometry>\
            <mesh>\
              <uri>model://"
    << name << "/meshes/" << name << ".stl</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
          <max_contacts>10</max_contacts>\
        </collision>\
        <visual name='visual'>\
          <pose frame=''>0 0 -0.09 0 -0 0</pose>\
          <geometry>\
            <mesh>\
              <uri>model://"
    << name << "/meshes/" << name << ".dae</uri>\
              <scale>1 1 1</scale>\
            </mesh>\
          </geometry>\
        </visual>\
        <self_collide>0</self_collide>\
        <kinematic>0</kinematic>\
        <gravity>1</gravity>\
      </link>\
    </model>\
    </sdf>";
  // was 0 0 -0.09
  spawn.request.model_xml = s.str();
  spawn.request.robot_namespace = "object_spawner";
  spawn.request.initial_pose = pose;
  spawn.request.reference_frame = frame_id;

  //ROS_INFO("Resulting model: \n %s",s.str().c_str());

  //ROS_INFO("Waiting for service");
  spawn_object.waitForExistence();
  //ROS_INFO("Calling service");

  //std::cout<<spawn.request<<std::endl;

  if (!spawn_object.call(spawn))
  {
    ROS_ERROR("Failed to call service %s", SPAWN_OBJECT_TOPIC);
  }
  ROS_INFO("Result: %s, code %u", spawn.response.status_message.c_str(), spawn.response.success);
}
