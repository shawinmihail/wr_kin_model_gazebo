#include <ignition/math/Vector3.hh>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>

#include <iostream>
#include <cmath>

#include "ros/ros.h"

#include "curve_math.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo_spawn_node");
    
    // init
    ignition::transport::Node node;
    ignition::msgs::Marker markerMsg;
    
    int n = 0;
    for (double t = 0; t < 30; t = t + 0.5)
    {
        
        n = n + 1;
        
        double x = 30.0 * cos (t * 2.0 * 3.1415 / 100) - 30.0;
        double y = 30.0 * sin (t * 2.0 * 3.1415 / 100);
        double z = 6;
        
        double t1 = t + 0.5;
        double x1 = 30.0 * cos (t1 * 2.0 * 3.1415 / 100) - 30.0;
        double y1 = 30.0 * sin (t1 * 2.0 * 3.1415 / 100);
        double z1 = 6;
        
        markerMsg.set_id(n);
        ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
        markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
        markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
        ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(x, y, z));
        ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(x1, y1, z1));
        node.Request("/marker", markerMsg);
    }
    
    return 0;
}


void spawnTest()
{

}


