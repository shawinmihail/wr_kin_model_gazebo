#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <iostream>
#include <cmath>

#include "curve_math.h"


std::string string_eVector3(const eVector3& v)
{
    std::ostringstream os;
    os << v[0] << " " << v[1] << " " << v[2];
    std::string s = os.str();
    return s;
};

namespace gazebo
{
class gazeboWrSpawnRoute : public WorldPlugin
{

public:

    gazeboWrSpawnRoute() : beaconCounter(0)
    {}

    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        parent = _parent;
        std::vector<eVector3> wps = waypoints();
        std::vector<eMatrix43> cfs_list = splineCoeffs(wps);
        
        for (int i = 0; i < cfs_list.size(); i++)
        {
            eVector3 wp = wps.at(i);
            eMatrix43 cfs = cfs_list.at(i);
            spawnWp(wp);
            

            // spawn b splane curve
            for(double a = 0; a < 1; a += 0.25)
            {
                eVector3 splinePoint = getSplinePoint(cfs, a);
                //spawnSplPoint(splinePoint);
            }
        }
    }

    void spawnWp(const eVector3& pos){
        sdf::SDF SDF;
        
        SDF.SetFromString(
        "<?xml version=\"1.0\"?>\n<sdf version=\"1.6\">\n<model name=\"route_beacon_" + std::to_string(beaconCounter) + "\">\n" + 
        "<pose>" + string_eVector3(pos) + " 0 0 0</pose>\n<static>true</static>\n<link name=\"link\">\n<visual name=\"visual\">\n" +
        "<geometry>\n<sphere><radius>0.25</radius></sphere>\n</geometry>\n" + 
        "<material>\n<script>\n<name>Gazebo/Red</name>\n<uri>file://media/materials/scripts/gazebo.material</uri>\n</script>\n</material>\n" + 
        "</visual>\n</link>\n</model>\n</sdf>"
        );
        
        sdf::ElementPtr model = SDF.Root()->GetElement("model");
        //model->GetAttribute("name")->SetFromString("sterh" + id);
        parent->InsertModelSDF(SDF);
        beaconCounter ++;
    };
    
    void spawnSplPoint(const eVector3& pos){
        sdf::SDF SDF;
        
        SDF.SetFromString(
        "<?xml version=1.0?>\
        <sdf version=1.6>\
        <model name='route_beacon_"+
        std::to_string(beaconCounter) +
        "'>\
        <pose>" +
        string_eVector3(pos) +
        " 0 0 0</pose>\
        <static>true</static>\
        <link name='link'>\
        <visual name='visual'>\
        <geometry>\
        <sphere><radius>0.1</radius></sphere>\
        </geometry>\
        <material>\
        <script>\
		<name>Gazebo/Blue</name>\
		<uri>file://media/materials/scripts/gazebo.material</uri>\
        </script>\
	    </material>\
        </visual>\
        </link>\
        </model>\
        </sdf>"
        );
        
        sdf::ElementPtr model = SDF.Root()->GetElement("model");
        //model->GetAttribute("name")->SetFromString("sterh" + id);
        parent->InsertModelSDF(SDF);
        beaconCounter ++;
    };

    physics::WorldPtr parent;
    event::ConnectionPtr updateConnection;
    int beaconCounter;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(gazeboWrSpawnRoute)
}
