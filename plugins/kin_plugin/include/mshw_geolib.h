#pragma once
#include<iostream>
#include<cmath>

namespace mswhgeo
{
    
struct GeoEllipsoid
{
    double a;
    double f_inv;
    
    double f;
    double b;
    double e;
    double e_2;
    double hmin;
    
    GeoEllipsoid(double a_, double f_inv_) :
     a(a_)
    ,f_inv(f_inv_)
    {
        f = 1.0 / f_inv;
        b = a * (1.0 - f);
        e = sqrt(1.0 - b * b / a / a);
        e_2 = e * e;
        hmin = pow(e, 12) / 4.0;
    };
    
    GeoEllipsoid(const GeoEllipsoid& gE_) : a(gE_.a), f_inv(gE_.f_inv)
    {
        GeoEllipsoid(gE_.a, gE_.f_inv);
    };
};   

class Geo
{
public:
    
    Geo(GeoEllipsoid gE) : _gE(gE)
    {
    };
    
    int sign(double val)
    {
        return (double(0.0) < val) - (val < double(0.0));
    };
    
    void Wgs2Ecef(double lat, double lon, double alt, double& x, double& y, double& z)
    {
        double sLat = sin(lat);
        double cLat = cos(lat);
        double sLon = sin(lon);
        double cLon = cos(lon);
        double xi = sqrt(1.0 - _gE.e_2 * sLat * sLat);
        x = (_gE.a / xi + alt) * cLat * cLon;
        y = (_gE.a / xi + alt) * cLat * sLon;
        z = (_gE.a * (1.0 - _gE.e_2) / xi + alt) * sLat;
    };
    
    int Ecef2Wgs(double x, double y, double z, double& lat, double& lon, double& alt)
    {
        double a_2 = _gE.a * _gE.a;
    
    double w_2 = x*x + y*y;
    double l = _gE.e_2 / 2.0;
    double m = w_2 / a_2;
    double n = z * z * (1.0 - _gE.e_2) / a_2;
    double p = (m + n - 4.0 * l * l) / 6.0;
    double G = m * n * l * l;
    double H = 2.0 * p * p * p + G;
    if (H < _gE.hmin)
    {
        return -1;
    }
    double C = pow(H + G + 2.0 * sqrt(H * G), 1.0 / 3.0) / pow(2, (1.0 / 3.0));
    double i = -(2.0 * l * l + m + n) / 2.0;
    double P = p * p;
    double beta = i / 3.0 - C - P / C;
    double k = l * l * (l * l - m - n);
    double t = sqrt(sqrt(beta * beta - k) - (beta + i) / 2.0) - sign(m - n) * sqrt(abs(beta - i) / 2.0);
    double F = pow(t, 4.0) + 2.0 * i * t * t + 2.0 * l * (m - n) * t + k;
    double dFdt = 4.0 * t * t * t + 4.0 * i * t + 2 * l * (m - n);
    double Delta_t = -F / dFdt;
    double u = t + Delta_t + l;
    double v = t + Delta_t - l;
    double w = sqrt(w_2);
    double Delta_w = w * (1.0 - 1.0 / u);
    double Delta_z = z * (1.0 - (1.0 - _gE.e_2) / v);
    
    lat = atan2(z * u, w * v) ;
    alt = sign(u - 1) * sqrt(Delta_w * Delta_w + Delta_z * Delta_z);
    lon = atan2(y, x);
    
    return 0;
    };

    void Wgs2Enu(/*ref point*/double lat0, double lon0, double alt0, /*wgs point*/double lat, double lon, double alt, /*out enu point*/ double& e, double& n, double& u)
    {
        double x0 = 0;
        double y0 = 0;
        double z0 = 0;
        Wgs2Ecef(lat0, lon0, alt0, x0, y0, z0);
        
        double x = 0;
        double y = 0;
        double z = 0;
        Wgs2Ecef(lat, lon, alt, x, y, z);
        
        Ecef2Enu(lat0, lon0, alt0, x - x0, y - y0, z - z0,  e, n, u);
    };
    
    int Enu2Wgs(/*ref point*/double lat0, double lon0, double alt0, /*enu point*/double e, double n, double u, /*out wgs point*/ double& lat, double& lon, double& alt)
    {
        double x0 = 0;
        double y0 = 0;
        double z0 = 0;
        Wgs2Ecef(lat0, lon0, alt0, x0, y0, z0);
        
        double x = 0;
        double y = 0;
        double z = 0;
        Enu2Ecef(lat0, lon0, alt0, e, n, u, x, y, z);
        return Ecef2Wgs(x0 + x, y0 + y, z0 + z, lat, lon, alt);
    };
    
    void Ecef2Enu(/*ref point*/double lat0, double lon0, double alt0, /*ecef point*/double x, double y, double z,  /*out enu point*/double& e, double& n, double& u)
    {
        double sLat = sin(lat0);
        double cLat = cos(lat0);
        double sLon = sin(lon0);
        double cLon = cos(lon0);
        
        e = -sLon * x        + cLon * y;
        n = -sLat * cLon * x - sLat * sLon * y + cLat * z;
        u =  cLat * cLon * x + cLat * sLon * y + sLat * z;  
    };
    
    void Enu2Ecef(/*ref point*/double lat0, double lon0, double alt0, /*enu point*/double e, double n, double u,  /*out ecef point*/double& x, double& y, double& z)
    {
        double sLat = sin(lat0);
        double cLat = cos(lat0);
        double sLon = sin(lon0);
        double cLon = cos(lon0);
                
        x = -sLon * e - sLat * cLon * n + cLat * cLon * u;
        y =  cLon * e - sLat * sLon * n + cLat * sLon * u;
        z = n * cLat + u * sLat;
    }
    
private:
    GeoEllipsoid _gE;
};

}

namespace mswhgeo_constants{
    static const struct mswhgeo::GeoEllipsoid WGS84 = {6378137.0, 298.257223563};
    static const double PI = 3.1415926535898;
}
