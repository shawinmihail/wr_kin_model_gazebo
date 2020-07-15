#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/msgs/msgs.hh"
#include <ignition/math/Pose3.hh>

#include <iostream>
#include <stdio.h>
#include <cmath>

#include "curve_math.h"

namespace gazebo
{

class gazeboWrKinPlugin : public ModelPlugin
  {
    public:

    gazeboWrKinPlugin():
    lastTime(0.0)
    ,surfMotModel(0.2,0.5,1.6)
    {
    };

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model = _parent;
        world = model->GetWorld();
        updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gazeboWrKinPlugin::OnUpdate, this, _1));
        baseLink = this->model->GetLink("body");
        
        
        splineCfsList = splineCoeffs(waypoints());
    }
    
    void goWithTraj(double t)
    {
        double v = 1;
        Position currPose = baseLink->WorldInertialPose();
        eVector3 rNext = aim_traj(t / 1.0);
        
        /*
        std::cout << rNext << std::endl;
        std::cout << t << std::endl;
        std::cout << "-----" << std::endl;
        */
        
        eVector3 n =  surf_n(rNext[0], rNext[1]);
        double yaw = yaw_on_aim_traj(t);
        eVector4 qNext = quatFromDirAndYaw(n, yaw);
        //Quaternion qNext(1,0,0,0);
        //Position nextPose(rNext, qNext);
        //baseLink->SetWorldPose(nextPose);
    }
    
    void goWithModel(double v, double u, double dt)
    {
        
        surfMotModel.calcNext(u, v, dt);
        baseLink->SetWorldPose(surfMotModel.getPosition());
    }

    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        double t = world->SimTime().Double();
        double dt = world->SimTime().Double() - lastTime.Double();
        lastTime = world->SimTime();
        //goWithTraj(t);
        double v = 1;
        double u = 0.0*sin(t*2*3.1415 / 15.0);
        
        Position currPose = baseLink->WorldInertialPose();
        eVector3 pos(currPose.Pos().X(), currPose.Pos().Y(), currPose.Pos().Z());
        eVector4 q(currPose.Rot().W(), currPose.Rot().X(), currPose.Rot().Y(), currPose.Rot().Z());
        u = calc_ctrl(pos, q, splineCfsList, v);
        
        goWithModel(v, u, dt);
    }

   
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        physics::WorldPtr world;
        physics::LinkPtr baseLink;
        common::Time lastTime;
        
        SurfMotModel surfMotModel;
        std::vector<eMatrix43> splineCfsList;
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(gazeboWrKinPlugin);
}
