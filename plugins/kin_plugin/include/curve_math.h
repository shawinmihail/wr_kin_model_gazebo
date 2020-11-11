#pragma once
#include <gazebo/common/common.hh>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include "mshw_geolib.h"

typedef ignition::math::Quaternion<double> Quaternion;
typedef ignition::math::Vector3d Vector;
typedef ignition::math::Pose3d  Position;

typedef Eigen::Matrix<double, 4, 4>  eMatrix44;
typedef Eigen::Matrix<double, 4, 3>  eMatrix43;
typedef Eigen::Matrix<double, 3, 3>  eMatrix33;
typedef Eigen::Matrix<double, 1, 3>  eMatrix13;
typedef Eigen::Matrix<double, 1, 4>  eMatrix14;
typedef Eigen::Matrix<double, 3, 1>  eVector3;
typedef Eigen::Matrix<double, 4, 1>  eVector4;
typedef Eigen::Matrix<double, 10, 1>  eVector10;

/*BEGIN quat*/

eVector3 quatRotate(const eVector4& q, const eVector3& v);

eVector4 quatMultiply(const eVector4& q, const eVector4& r);

eVector4 quatInverse(const eVector4& q);

/*End quat*/

double surf(double x, double y);

eVector3 surf_n(double x, double y);

eVector3 aim_traj(double t);

std::vector<eVector3> waypoints();

eVector3 aim_traj_dot_numeric(double t);

double yaw_on_aim_traj(double t);

eVector4 quatFromDirAndYaw(const eVector3& n, double yaw);

std::vector<eMatrix43> splineCoeffs(const std::vector<eVector3>& points);

double calc_ctrl(const eVector3& pos, const eVector4& q, const std::vector<eMatrix43>& splineCfsList, double v);

class SurfMotModel
{
public:
    
    SurfMotModel(double x, double y, double yaw);
    
    void calcNext(double u, double v, double dt);
    
    eVector10 getState();
    
    Position getPosition();
    
    Vector getLinearVel();
    
    Vector getAcc();
    
    Vector getRotVel();
    
private:
    double yaw0;
    eVector10 state;
    eVector3 acc;
    eVector3 rotVel;
    
    double Kv;
};

class ImuMeasurmentsModel
{
public:
    
    ImuMeasurmentsModel();
    ImuMeasurmentsModel(double accRms_, double angvelRms_, const eVector3& g_);
    void setParams(double accRms_, double angvelRms_, const eVector3& g_);
    
    eVector3 getImuAcc(const eVector3& acc, const eVector4& q);
    eVector3 getImuAngVel(const eVector3& angVel);
    
private:
    std::default_random_engine random_generator;
    std::normal_distribution<double> normal_distribution_acc;
    std::normal_distribution<double> normal_distribution_rotvel;
    eVector3 g;
};


class GnnsTripletMeasurmentsModel
{
public:
    
    GnnsTripletMeasurmentsModel();
    GnnsTripletMeasurmentsModel(const eVector3& refLatLonAlt_, double posBaseRms_, double velBaseRms_, double posSlavesRms_, const eVector3& dr_base_, const eVector3& dr_slave1_, const eVector3& dr_slave2_);
    void setParams(const eVector3& refLatLonAlt_, double posBaseRms_, double velBaseRms_, double posSlavesRms_, const eVector3& dr_base_, const eVector3& dr_slave1_, const eVector3& dr_slave2_);
    
    eVector3 getBasePosEnu(const eVector3& pos, const eVector4& q);
    eVector3 getBaseVelEnu(const eVector3& vel, const eVector4& q, const eVector3& angVel);
    eVector3 getSlave1DrEnu(const eVector4& q);
    eVector3 getSlave2DrEnu(const eVector4& q);
    
    eVector3 getBasePosEcef(const eVector3& pos, const eVector4& q);
    eVector3 getBaseVelEcef(const eVector3& vel, const eVector4& q, const eVector3& angVel);
    eVector3 getSlave1DrEcef(const eVector4& q);
    eVector3 getSlave2DrEcef(const eVector4& q);
    
private:
    mswhgeo::Geo geo;
    eVector3 refLatLonAlt;
    eVector3 refEcefXYZ;
    
    std::default_random_engine rg;
    std::normal_distribution<double> nd_base_pos;
    std::normal_distribution<double> nd_base_vel;
    std::normal_distribution<double> nd_slaves_pos;

    eVector3 dr_base;
    eVector3 dr_slave1;
    eVector3 dr_slave2;
};


/*BEGIN splain operations*/

eVector3 getSplinePoint(const eMatrix43& cfs, double s);

eVector3 getSplinePointDerv(const eMatrix43& cfs, double s);

eVector3 getSplinePointDDerv(const eMatrix43& cfs, double s);

double sstarCriteria(const eMatrix43& cfs, double s, const eVector3& y);

double sstarCriteriaDerv(const eMatrix43& cfs, double s, const eVector3& y);

double calcSstarNumeric(const eVector3& y, const eMatrix43& cfs);


/*END splain operations*/
