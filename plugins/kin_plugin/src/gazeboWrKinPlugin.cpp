#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/msgs/msgs.hh"
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/transport/transport.hh"

#include "ros/ros.h"
#include "wr_msgs/ctrl_stamped.h"
#include "wr_msgs/imu_stamped.h"
#include "wr_msgs/ninelives_triplet_stamped.h"
#include "wr_msgs/est_state_stamped.h"

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
    ,surfMotModel(8.0, -2.0, 1.5)
    ,input_u(0)
    ,input_v(0)
    {
  
    };

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model = _parent;
        world = model->GetWorld();
        updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&gazeboWrKinPlugin::OnUpdate, this, _1));
        baseLink = this->model->GetLink("body");
        splineCfsList = splineCoeffs(waypoints());
        initSubsPubs();
        
        /* TODO define params in sdf and launch file*/
        double PI = 3.1415926535898;
        eVector3 refLatLonAlt(55.944663 * PI / 180.0, 38.141938 * PI / 180.0, 200.0);
        eVector3 dr_base(-0.4, 0.0, 0.4);
        eVector3 dr_slave1(0.73, 0.23, 0.0);
        eVector3 dr_slave2(0.73, -0.23, 0.0);
        double posBaseRms = 0.005;
        double velBaseRms = 0.003;
        double posSlavesRms = 0.005;
        gnnsMesModel.setParams(refLatLonAlt, posBaseRms, velBaseRms, posSlavesRms, dr_base, dr_slave1, dr_slave2);
        
        double accRms = 0.9;
        double angvelRms = 0.05;
        eVector3 g(0,0,-9.8);
        imuMesModel.setParams(accRms, angvelRms, g);
        
        /*
        double x0 = 8.2;
        double y0 = 0.0;
        double z0 = surf(x0, y0);
        eVector4 q = quatFromEul(eVector3(0,0,3.14/2.0));
        Position p0(Vector(x0, y0, z0), Quaternion(q[0], q[1], q[2], q[3]));
        model->SetRelativePose(p0);
        std::cout << p0 <<std::endl;
        */
    }
    
    void initSubsPubs()
    {
        /* TODO name topics like in robot*/
        statePub = nodeHandle.advertise<wr_msgs::est_state_stamped>("wr_sensors/model_state", 1);
        imuMesPub = nodeHandle.advertise<wr_msgs::imu_stamped>("wr_sensors/imu", 1);
        gnnsMesPub = nodeHandle.advertise<wr_msgs::ninelives_triplet_stamped>("wr_sensors/nl_triplet", 1);
        throttleSub = nodeHandle.subscribe("wr_actuators/throttle", 1, &gazeboWrKinPlugin::throttleCb, this);
        steerSub = nodeHandle.subscribe("wr_actuators/steer", 1, &gazeboWrKinPlugin::steerCb, this);
        lightSub = nodeHandle.subscribe("wr_actuators/light", 1, &gazeboWrKinPlugin::lightCb, this);
    }
    
    void pubImu()
    {
        eVector3 a = imuMesModel.getImuAcc(ignition2evector(currAcc), ignition2evector(currAtt));
        eVector3 w = imuMesModel.getImuAngVel(ignition2evector(currAngVel));
        
        wr_msgs::imu_stamped msg;
        msg.stamp = ros::Time::now();
                
        msg.acc.x = a[0];
        msg.acc.y = a[1];
        msg.acc.z = a[2];
        
        msg.ang_vel.x = w[0];
        msg.ang_vel.y = w[1];
        msg.ang_vel.z = w[2];

        imuMesPub.publish(msg);
    }
    
    void pubGnns()
    {
        eVector3 r = gnnsMesModel.getBasePosEcef(ignition2evector(currPose), ignition2evector(currAtt));
        eVector3 dr1 = gnnsMesModel.getSlave1DrEcef(ignition2evector(currAtt));
        eVector3 dr2 = gnnsMesModel.getSlave2DrEcef(ignition2evector(currAtt));
        eVector3 v = gnnsMesModel.getBaseVelEcef(ignition2evector(currVel), ignition2evector(currAtt), ignition2evector(currAngVel));
        
        wr_msgs::ninelives_triplet_stamped msg;
        msg.stamp = ros::Time::now();
        
        
        msg.r_ecef_master.x = r[0];
        msg.r_ecef_master.y = r[1];
        msg.r_ecef_master.z = r[2];
        
        msg.v_ecef_master.x = v[0];
        msg.v_ecef_master.y = v[1];
        msg.v_ecef_master.z = v[2];
        
        msg.dr_ecef_slave1.x = dr1[0];
        msg.dr_ecef_slave1.y = dr1[1];
        msg.dr_ecef_slave1.z = dr1[2];
        
        msg.dr_ecef_slave2.x = dr2[0];
        msg.dr_ecef_slave2.y = dr2[1];
        msg.dr_ecef_slave2.z = dr2[2];
        
        msg.status_master = 4;
        msg.status_slave1 = 4;
        msg.status_slave2 = 4;

        gnnsMesPub.publish(msg);
    }
    
    void pubState()
    {
        wr_msgs::est_state_stamped msg;
        
        msg.stamp = ros::Time::now();
        msg.pos.x = currPose.X();
        msg.pos.y = currPose.Y();
        msg.pos.z = currPose.Z();

        msg.att.w = currAtt.W();
        msg.att.x = currAtt.X();
        msg.att.y = currAtt.Y();
        msg.att.z = currAtt.Z();

        msg.vel.x = currVel.X();
        msg.vel.y = currVel.Y();
        msg.vel.z = currVel.Z();

        statePub.publish(msg);
    }
    
    void throttleCb(const wr_msgs::ctrl_stamped& msg)
    {
        input_v = msg.vel * 3.0 / 100.0;
    }
    
    void steerCb(const wr_msgs::ctrl_stamped& msg)
    {
        input_u = msg.ang * 3.1415926535898 / 180.0;
        double H =  0.725;
        input_u = tan(input_u)/H;
    }
    
    void lightCb(const wr_msgs::ctrl_stamped& msg)
    {
        input_l = msg.light;
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
        model->SetRelativePose(surfMotModel.getIgnitionPosition());
        
        currPose = evector2ignition((eVector3)surfMotModel.getState().segment(0,3));
        currAtt = evector2ignition((eVector4)surfMotModel.getState().segment(6,4));
        currVel = evector2ignition((eVector3)surfMotModel.getState().segment(3,3));
        currAngVel = evector2ignition((eVector3)surfMotModel.getAcc());
        currAcc = evector2ignition((eVector3)surfMotModel.getRotVel());
    }
    
    void goWithForce(double v, double u, double dt, double t)
    {
    
        ignition::math::Quaterniond qCurr = baseLink->WorldPose().Rot();
        ignition::math::Vector3d eulCurr = qCurr.Euler();
        double yawCurr = eulCurr.Z();
        ignition::math::Vector3d wCurr = baseLink->RelativeAngularVel();
        ignition::math::Vector3d vCurr = baseLink->WorldLinearVel();
        ignition::math::Vector3d rCurr = baseLink->WorldPose().Pos();
        eVector3 ev_n = surf_n(rCurr.X(), rCurr.Y());
        double ev_h = surf(rCurr.X(), rCurr.Y());
        eVector4 ev_qCurr(qCurr.W(), qCurr.X(), qCurr.Y(), qCurr.Z());
        eVector3 ev_vCurr(vCurr.X(), vCurr.Y(), vCurr.Z());
        eVector3 ev_vB = quatRotate(quatInverse(ev_qCurr), ev_vCurr);
        
        eVector3 ev_vB0(v, 0, 0);
        eVector3 ev_dV = ev_vB0 - ev_vB;
        double Kv_acc = 2;
        double Kv_model = 6;
        double Kp_model = 5;
        ignition::math::Vector3d relForce(Kv_acc*ev_dV[0], Kv_model*ev_dV[1], Kv_model*ev_dV[2]);
        ignition::math::Vector3d force(0, 0, Kp_model*(ev_h - rCurr.Z()));
        if (t > 7)
        {
            baseLink->AddForce(force);
            baseLink->AddRelativeForce(relForce);
        }
        
        
        eVector4 ev_q0 = quatFromDirAndYaw(ev_n, yawCurr);
        ignition::math::Quaterniond qDes(ev_q0[0], ev_q0[1], ev_q0[2], ev_q0[3]);
        ignition::math::Quaterniond dq = qDes.Inverse() * qCurr;
        //ignition::math::Vector3d dqv(dq.X(), dq.Y(), dq.Z());
        ignition::math::Vector3d dqv = qDes.Euler() - qCurr.Euler();
        eVector3 ev_wB0(0, 0, ev_vB.norm()*u);
        ignition::math::Vector3d wDes(ev_wB0[0], ev_wB0[1], ev_wB0[2]);
        ignition::math::Vector3d dw = wDes - wCurr;
        double Kq_model = 4;
        double Kw_model = 2;
        double Kq_control = 0.1;
        double Kw_control = 1.0;

        ignition::math::Vector3d tau = ignition::math::Vector3d(Kw_model * dw.X(), Kw_model * dw.Y(), Kw_control * dw.Z()) +
                                       ignition::math::Vector3d(Kq_model * dqv.X(), Kq_model * dqv.Y(), Kq_control * dqv.Z());
        baseLink->AddRelativeTorque(tau);
    }

    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        double t = world->SimTime().Double();
        double dt = world->SimTime().Double() - lastTime.Double();
        if (dt > 0.05) // TODO need to use supposed sim rate
        {
            dt = 0.05;
        }
        lastTime = world->SimTime();
                
        //double v = 1;
        //double u = calc_ctrl(ignition2evector(currPose), ignition2evector(currAtt), splineCfsList, v);
        double u = input_u;
        double v = input_v;
                
        goWithModel(v, u, dt);
        
        /* TODO pub clear acc*/
        /* TODO control pub rate*/
        pubState();
        pubGnns();
        pubImu();
        
        ros::spinOnce();
    }

    private:
        ignition::math::Vector3d currPose;
        ignition::math::Vector3d currVel;
        ignition::math::Quaterniond currAtt;
        ignition::math::Vector3d currAngVel;
        ignition::math::Vector3d currAcc;
    private:
        double input_u;
        double input_v;
        int input_l;
    private:
        GnnsTripletMeasurmentsModel gnnsMesModel;
        ImuMeasurmentsModel imuMesModel;
    private:
        event::ConnectionPtr updateConnection;
        physics::ModelPtr model;
        physics::WorldPtr world;
        physics::LinkPtr baseLink;
        common::Time lastTime;
    private:
        SurfMotModel surfMotModel;
        std::vector<eMatrix43> splineCfsList;
    private:
        ros::NodeHandle nodeHandle;
        ros::Subscriber throttleSub;
        ros::Subscriber steerSub;
        ros::Subscriber lightSub;
        ros::Publisher statePub;
        ros::Publisher imuMesPub;
        ros::Publisher gnnsMesPub;
    private:
        eVector3 ignition2evector(const ignition::math::Vector3d& iv)
        {
            return eVector3 (iv.X(), iv.Y(), iv.Z());
        }
        
        eVector4 ignition2evector(const ignition::math::Quaterniond& iv)
        {
            return eVector4 (iv.W(), iv.X(), iv.Y(), iv.Z());
        }
        
        ignition::math::Vector3d evector2ignition(const eVector3& iv)
        {
            return ignition::math::Vector3d(iv[0], iv[1], iv[2]);
        }
        
        ignition::math::Quaterniond evector2ignition(const eVector4& iv)
        {
            return ignition::math::Quaterniond (iv[0], iv[1], iv[2], iv[3]);
        }
};

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(gazeboWrKinPlugin);
}
