#include <ignition/math/Vector3.hh>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>

#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "wr_msgs/route_line.h"
#include "geometry_msgs/Vector3.h"

#include "curve_math.h"


bool routeCbCalled = false;
wr_msgs::route_line routeMsg;


ignition::transport::Node node;
ignition::msgs::Marker markerMsg;
ignition::msgs::Material *matMsg = markerMsg.mutable_material();

void routeSubCb(wr_msgs::route_line msg)
{
    ROS_INFO("New route with %d points received", msg.points.size());
    routeMsg = msg;
    routeCbCalled = true;
}

void drawRoute()
{
    int N = routeMsg.points.size();
    for (int i = 0; i < N; ++i )
    {        
        geometry_msgs::Vector3 p = routeMsg.points.at(i);
        
        markerMsg.set_id(i);
        ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
        markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
        markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
        ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(p.x, p.y, p.z));
    }
    
        node.Request("/marker", markerMsg);
        matMsg->mutable_script()->set_name("Gazebo/Red");
        node.Request("/marker", markerMsg);
}

int main(int argc, char **argv)
{
    // ros init
    ros::init(argc, argv, "kin_model_spawn_node");
    ros::NodeHandle nodeHandle;
    ros::Subscriber routeSub;
    routeSub = nodeHandle.subscribe("kin_model/route_input", 1, &routeSubCb);
    ros::Rate rate(50);
    
    while(ros::ok())
    {
        if (routeCbCalled)
        {
            drawRoute();
            routeCbCalled = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}


