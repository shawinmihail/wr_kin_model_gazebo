#include <GL/gl.h>
#include "curve_math.h"
#include <cmath>
#include "surf_fcn.h"
#include "surf_fcn_terminate.h"
#include <iostream>

eVector4 quatFromEul(const eVector3& eul)
{
	 float roll = eul[0];
	 float pitch = eul[1];
	 float yaw = eul[2];

	 float cy = cos(yaw * 0.5f);
	 float sy = sin(yaw * 0.5f);
	 float cr = cos(roll * 0.5f);
	 float sr = sin(roll * 0.5f);
	 float cp = cos(pitch * 0.5f);
	 float sp = sin(pitch * 0.5f);

	 float q0 = cy * cr * cp + sy * sr * sp;
	 float q1 = cy * sr * cp - sy * cr * sp;
	 float q2 = cy * cr * sp + sy * sr * cp;
	 float q3 = sy * cr * cp - cy * sr * sp;

	 return eVector4(q0, q1, q2 ,q3);
}

eVector4 quatMultiply(const eVector4& q, const eVector4& r)
{
	eVector4 p;
	p[0] = r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3];
	p[1] = r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2];
	p[2] = r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1];
	p[3] = r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0];
	return p;
}

eVector4 quatInverse(const eVector4& q)
{
	eVector4 qDual (q[0], -q[1], -q[2], -q[3]);
	return qDual;
}

eVector3 quatRotate(const eVector4& q, const eVector3& v)
{
	eVector4 qv(0.0, v[0], v[1], v[2]);

	eVector4 qDual = quatInverse(q);
	eVector4 qv1 = quatMultiply(qv, qDual);
	eVector4 qv2 = quatMultiply(q, qv1);

	return eVector3(qv2[1], qv2[2], qv2[3]);
}

double surf(double x, double y)
{
    //double z = 2.0 * sin (x * 2.0 * 3.1415 / 25.0) +  3.0 *sin(y * 2.0* 3.1415 / 35.0);
    double out[4];
    surf_fcn(x, y, out);
    return out[0];
};

eVector3 surf_n(double x, double y)
{
    //double xn = (2.0 * 3.1415 / 25.0) * 2.0 * cos (x * 2.0 * 3.1415 / 25.0);
    //double yn = (2.0* 3.1415 / 35.0) * 3.0 * cos(y * 2.0* 3.1415 / 35.0);
    //double zn = 1;
    
    double out[4];
    surf_fcn(x, y, out);
    eVector3 n(out[1], out[2], out[3]);
    //std::cout << n << std::endl;
    return n.normalized();
};

eVector3 aim_traj(double t)
{
    double x = 30.0 * cos (t * 2.0 * 3.1415 / 100) - 30.0;
    double y = 30.0 * sin (t * 2.0 * 3.1415 / 100);
    double z = surf(x, y);
    return eVector3(x, y, z);
};

std::vector<eVector3> waypoints()
{
    std::vector<eVector3> wps;
    double t_step = 3;
    double t_lim = 100;
    double eps = 1e-6;
    for (double t = -t_step; t < t_lim; t += t_step-eps)
    {
       eVector3 wp = aim_traj(t);
       wps.push_back(wp);
    }
    return wps;
}

eVector3 aim_traj_dot_numeric(double t)
{
    double eps = 1e-6;
    eVector3 p1 =  aim_traj(t);
    eVector3 p2 =  aim_traj(t + eps);
    eVector3 dpdt = (p2 - p1) / eps;
    return dpdt;
};

double yaw_on_aim_traj(double t)
{
    eVector3 dpdt = aim_traj_dot_numeric(t);
    dpdt = eVector3(dpdt[0], dpdt[1], 0.0);
    dpdt = dpdt.normalized();
    eVector3 x(1.0, 0.0, 0.0);
    double cosA = x.dot(dpdt);
    double yaw = acos(cosA);
    if (dpdt[1] < 0.0)
    {
        yaw = -yaw;
    }
    return yaw;
};


eVector4 quatFromDirAndYaw(const eVector3& n, double yaw)
{
    double eps = 1e-6;
    
    eVector4 qYaw(cos(yaw/2.0), 0.0, 0.0, sin(yaw/2.0));
    if (n.norm() < eps)
    {
        return qYaw;
    }
    eVector3 z(0.0, 0.0, 1.0);
    eVector3 pin = z.cross(n);
    if (pin.norm() < eps)
    {
        return qYaw;
    }
    pin = pin.normalized();
    double cosA = n.dot(z);
    double A = acos(cosA);
    if(n[2] > 0){
        A = -A;
    }
    double sinHalfA = sin(A / 2.0);
    eVector4 qBow = eVector4(cos(A/2.0), sinHalfA*pin[0], sinHalfA*pin[1], sinHalfA*pin[2]);
    return quatMultiply(qBow, qYaw);
};


std::vector<eMatrix43> splineCoeffs(const std::vector<eVector3>& points)
{
    std::vector<eMatrix43> As;
    
    for (int k = 0; k < points.size() - 3; k ++)
    {
        eMatrix43 ps;
        for (int i = 0; i < 4; i ++)
        {
            eMatrix13 row;
            row << points[k+i][0], points[k+i][1], points[k+i][2];
            ps.row(i) = row;
        }
        
        eMatrix44 M;
        eMatrix14 M1;
        eMatrix14 M2;
        eMatrix14 M3;
        eMatrix14 M4;
        M1 << -1,  3, -3, 1;
        M2 <<  3, -6,  3, 0;
        M3 << -3,  0,  3, 0;
        M4 <<  1,  4,  1, 0;
        M << M1, M2, M3, M4;
        
        eMatrix43 A = 1.0 / 6.0 * M * ps;
        As.push_back(A);
    }
    
    return As;
}

double calc_ctrl(const eVector3& pos, const eVector4& q, const std::vector<eMatrix43>& splineCfsList, double v)
{
    double lambda = 4.0;
    
    //sstar, delta
    double sstar = -1;
    double delta = -1;
    eVector3 Delta;
    eMatrix43 cfs;
    for (int i = 0; i < splineCfsList.size(); i++)
    {
        cfs =  splineCfsList.at(i);
        double s  = calcSstarNumeric(pos, cfs);
        
        if(s>0){
            sstar = s;
            Delta = pos - getSplinePoint(cfs, sstar);
            delta = Delta.norm();
            break;
        }
    }
    
    if (sstar < 0){
        std::cout << "BAD CURVE DISTANCE CALCULATION" << std::endl;
        return 0;
    }
    
    eVector3 dp = getSplinePointDerv(cfs, sstar);
    eVector3 ddp = getSplinePointDDerv(cfs, sstar);

    eVector3 D = quatRotate(quatInverse(q), Delta);
    eVector3 Ctr_dp = quatRotate(quatInverse(q), dp);
    eVector3 C_ex = quatRotate(q, eVector3(1,0,0));
    
    double sstar_dot_top = v * eVector3(1,0,0).transpose() * Ctr_dp; // !!!!
    double sstar_dot_bot = (dp.norm() * dp.norm() - Delta.transpose() * ddp);
    double sstar_dot = sstar_dot_top / sstar_dot_bot * 10;
    double u = (-lambda*lambda*delta*delta - 2*v*lambda*D[0] - v*v + v*sstar_dot*dp.transpose() * C_ex + (v*D[0]/ delta)*(v*D[0]/ delta)) / (v*v * D[1]);

    double lim = 0.5;
    if (u > lim)
    {
        u = lim;
        
    }
    else if( u < -lim)
    {
        u = -lim;
    }
    //double u = min<double>(lim, max<double>(-lim, u)); !!!
    
    return u;
}


/* BEGIN class SurfMotModel */

SurfMotModel::SurfMotModel(double x, double y, double yaw) :
yaw0(yaw)
,Kv(2.0)
{
    state << x, y, surf(x, y), 0, 0, 0, 1, 0, 0, 0;
}
    
void SurfMotModel::calcNext(double u, double v, double dt)
{
    eVector3 r0 = state.segment(0,3);
    eVector3 v0 = state.segment(3,3);
    eVector4 q0 = state.segment(6,4);
        
    // r
    eVector3 r1 = r0 + v0 * dt;
    
    // q
    eVector3 n1 = surf_n(r1[0], r1[1]);
    //eVector3 en0(vn.X(), vn.Y(), vn.Z());
    
    double yaw1 = yaw0 + v * u * dt;
    eVector4 q1 = quatFromDirAndYaw(n1, yaw1);
    
    // v
    double v_curr = v0.norm();
    double a = Kv * (fabs(v) - v_curr);
    
    eVector3 v1;
    if (v > 0)
    {
        v1 = quatRotate(q0, eVector3(v_curr + a * dt, 0, 0));
    }
    else
    {
        v1 = quatRotate(q0, -eVector3(v_curr + a * dt, 0, 0));
    }
    
    // a w
    acc = (v1 - v0) / dt;

    eVector4 qdot = (q1 - q0)/ dt;
    eVector4 qw = 2 * quatMultiply(quatInverse(q0), qdot);
    rotVel = qw.segment(0,3);
    
    // state
    state << r1, v1, q1; 
    yaw0 = yaw1;
}

eVector10 SurfMotModel::getState()
{
    return state;
}
    
Position SurfMotModel::getIgnitionPosition()
{
    Position p(Vector(state[0], state[1], state[2]), Quaternion(state[6], state[7], state[8], state[9]));
    return p;
}

eVector3 SurfMotModel::getAcc()
{
    return acc;
}
    
eVector3 SurfMotModel::getRotVel()
{
    return rotVel;
}

/* END class SurfMotModel */

/*BEGIN splain operations*/

eVector3 getSplinePoint(const eMatrix43& cfs, double s)
{
    double x = cfs(0, 0) * s*s*s + cfs(1, 0) * s*s + cfs(2, 0) * s + cfs(3, 0);
    double y = cfs(0, 1) * s*s*s + cfs(1, 1) * s*s + cfs(2, 1) * s + cfs(3, 1);
    double z = cfs(0, 2) * s*s*s + cfs(1, 2) * s*s + cfs(2, 2) * s + cfs(3, 2);
    return eVector3(x, y, z);
}

eVector3 getSplinePointDerv(const eMatrix43& cfs, double s)
{
    double x = 3 * cfs(0, 0) * s*s + 2 * cfs(1, 0) * s + cfs(2, 0);
    double y = 3 * cfs(0, 1) * s*s + 2 * cfs(1, 1) * s + cfs(2, 1);
    double z = 3 * cfs(0, 2) * s*s + 2 * cfs(1, 2) * s + cfs(2, 2);
    return eVector3(x, y, z);
}

eVector3 getSplinePointDDerv(const eMatrix43& cfs, double s)
{
    double x = 6 * cfs(0, 0) * s + 2 * cfs(1, 0);
    double y = 6 * cfs(0, 1) * s + 2 * cfs(1, 1);
    double z = 6 * cfs(0, 2) * s + 2 * cfs(1, 2);
    return eVector3(x, y, z);
}

double sstarCriteria(const eMatrix43& cfs, double s, const eVector3& y)
{
    double res = (y - getSplinePoint(cfs ,s)).transpose() * getSplinePointDerv(cfs, s);
    return res;
}

double sstarCriteriaDerv(const eMatrix43& cfs, double s, const eVector3& y)
{
    double res1 = -getSplinePointDerv(cfs, s).transpose() * getSplinePointDerv(cfs ,s);
    double res2 = (y - getSplinePoint(cfs ,s)).transpose() * getSplinePointDDerv(cfs ,s);
    return res1 + res2;
}

double calcSstarNumeric(const eVector3& y, const eMatrix43& cfs)
{
    double eps = 1e-6;
    double slim = 1;
    double step = slim / 2.0 - eps;
    for (double s = 0 + eps; s < slim; s += step)
    {
        double x0 = s;
        double fx0 = 0;
        double fx0_dot = 0;
        for (int i = 0; i < 4; i++)
        {
            fx0 = sstarCriteria(cfs ,x0, y);
            fx0_dot = sstarCriteriaDerv(cfs ,x0, y);
            x0 = x0 - fx0/fx0_dot;
            if (x0 < 0 || x0 > slim)
            {
                break;
            }
        }
        if (fabs(fx0) < eps && x0 > 0 && x0 < slim)
        {
            return x0;
        }
    }
    return -1;
}

/*END splain operations*/
    
/*BEGIN class IMU mes model*/

ImuMeasurmentsModel::ImuMeasurmentsModel()
{
    
}

ImuMeasurmentsModel::ImuMeasurmentsModel(double accRms_, double angvelRms_, const eVector3& g_) :
 g(g_)
,normal_distribution_acc(0, accRms_)
,normal_distribution_rotvel(0, angvelRms_)
{
    
}

void ImuMeasurmentsModel::setParams(double accRms_, double angvelRms_, const eVector3& g_)
{
    g = g_;
    normal_distribution_acc = std::normal_distribution<double>(0, accRms_);
    normal_distribution_rotvel = std::normal_distribution<double>(0, angvelRms_);
}

eVector3 ImuMeasurmentsModel::getImuAcc(const eVector3& acc, const eVector4& q)
{
    eVector3 accImu = quatRotate(quatInverse(q), acc - g) + eVector3(normal_distribution_acc(random_generator), normal_distribution_acc(random_generator), normal_distribution_acc(random_generator));
    return accImu;
}

eVector3 ImuMeasurmentsModel::getImuAngVel(const eVector3& angVel)
{
    eVector3 angVelImu = angVel + eVector3(normal_distribution_rotvel(random_generator), normal_distribution_rotvel(random_generator), normal_distribution_rotvel(random_generator));
    return angVelImu;
}

/*END class IMU mes model*/


/*BEGIN class GNNS mes model*/
GnnsTripletMeasurmentsModel::GnnsTripletMeasurmentsModel():
geo(mswhgeo_constants::WGS84)
{
}

GnnsTripletMeasurmentsModel::GnnsTripletMeasurmentsModel(
    const eVector3& refLatLonAlt_, double posBaseRms_, double velBaseRms_, double posSlavesRms_, const eVector3& dr_base_, const eVector3& dr_slave1_, const eVector3& dr_slave2_):
dr_base(dr_base_)
,dr_slave1(dr_slave1)
,dr_slave2(dr_slave2)
,refLatLonAlt(refLatLonAlt_)
,geo(mswhgeo_constants::WGS84)
,nd_base_pos(0, posBaseRms_)
,nd_base_vel(0, velBaseRms_)
,nd_slaves_pos(0, posSlavesRms_)
{
    double x = 0, y = 0, z = 0;
    geo.Wgs2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], x, y, z);
    refEcefXYZ = eVector3(x, y, z);
}

void GnnsTripletMeasurmentsModel::setParams(
    const eVector3& refLatLonAlt_, double posBaseRms_, double velBaseRms_, double posSlavesRms_, const eVector3& dr_base_, const eVector3& dr_slave1_, const eVector3& dr_slave2_)
{
    dr_base = dr_base_;
    dr_slave1 = dr_slave1_;
    dr_slave2 = dr_slave2_;
    refLatLonAlt = refLatLonAlt_;
    
    nd_base_pos= std::normal_distribution<double>(0, posBaseRms_);
    nd_base_vel= std::normal_distribution<double>(0, velBaseRms_);
    nd_slaves_pos= std::normal_distribution<double>(0, posSlavesRms_);
    
    double x = 0, y = 0, z = 0;
    geo.Wgs2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], x, y, z);
    refEcefXYZ = eVector3(x, y, z);
}

eVector3 GnnsTripletMeasurmentsModel::getBasePosEnu(const eVector3& pos, const eVector4& q)
{
    eVector3 dr = quatRotate(q ,dr_base);
    eVector3 basePos = pos + dr + eVector3(nd_base_pos(rg), nd_base_pos(rg), nd_base_pos(rg));
    return basePos;
}

eVector3 GnnsTripletMeasurmentsModel::getBaseVelEnu(const eVector3& vel, const eVector4& q, const eVector3& angVel)
{
    eVector3 dv = quatRotate(q, angVel.cross(dr_base));
    eVector3 baseVel = vel + dv + eVector3(nd_base_vel(rg), nd_base_vel(rg), nd_base_vel(rg));
    return baseVel;
}

eVector3 GnnsTripletMeasurmentsModel::getSlave1DrEnu(const eVector4& q)
{
    eVector3 slavePos = quatRotate(q, dr_slave1) + eVector3(nd_slaves_pos(rg), nd_slaves_pos(rg), nd_slaves_pos(rg));
    return slavePos;
}

eVector3 GnnsTripletMeasurmentsModel::getSlave2DrEnu(const eVector4& q)
{
    eVector3 slavePos = quatRotate(q, dr_slave2) + eVector3(nd_slaves_pos(rg), nd_slaves_pos(rg), nd_slaves_pos(rg));
    return slavePos;
}

eVector3 GnnsTripletMeasurmentsModel::getBasePosEcef(const eVector3& pos, const eVector4& q)
{
    eVector3 basePosEnu = getBasePosEnu(pos, q);
    double x = 0, y = 0, z = 0;
    geo.Enu2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], basePosEnu[0], basePosEnu[1], basePosEnu[2], x, y, z);
    eVector3 basePos(x, y, z);
    return basePos + refEcefXYZ;
}

eVector3 GnnsTripletMeasurmentsModel::getBaseVelEcef(const eVector3& vel, const eVector4& q, const eVector3& angVel)
{
    eVector3 vEnu = getBaseVelEnu(vel, q, angVel);
    double x = 0, y = 0, z = 0;
    geo.Enu2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], vEnu[0], vEnu[1], vEnu[2], x, y, z);
    eVector3 velEcef(x, y, z);
    return velEcef;
}

eVector3 GnnsTripletMeasurmentsModel::getSlave1DrEcef(const eVector4& q)
{
    eVector3 slave1DrEnu = getSlave1DrEnu(q);
    double x = 0, y = 0, z = 0;
    geo.Enu2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], slave1DrEnu[0], slave1DrEnu[1], slave1DrEnu[2], x, y, z);
    eVector3 pos(x, y, z);
    return pos;
}

eVector3 GnnsTripletMeasurmentsModel::getSlave2DrEcef(const eVector4& q)
{
    eVector3 slave2DrEnu = getSlave2DrEnu(q);
    double x = 0, y = 0, z = 0;
    geo.Enu2Ecef(refLatLonAlt[0], refLatLonAlt[1], refLatLonAlt[2], slave2DrEnu[0], slave2DrEnu[1], slave2DrEnu[2], x, y, z);
    eVector3 pos(x, y, z);
    return pos;
}

/*END class GNNS mes model*/
