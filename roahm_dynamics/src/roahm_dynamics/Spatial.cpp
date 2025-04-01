#include "roahm_dynamics/Spatial.hpp"

namespace Roahm
{
namespace Spatial
{

void jcalc(Mat6& Xj, 
           Vec6& S, 
           const int jtyp, 
           const double q) 
{
    Xj.setIdentity();
    S.setZero();

    double c = cos(q);
    double s = sin(q);

    if (jtyp < 0) { // reversed direction
        // c = cos(-q) = cos(q)
        s = -s; // s = sin(-q) = -sin(q)
    }

    switch (jtyp)
    {
        case 1: // revolute X axis 'Rx'
            Xj(1,1) = c;
            Xj(1,2) = s;
            Xj(2,1) = -s;
            Xj(2,2) = c;
            S(0) = 1;
            break;
        case 2: // revolute Y axis 'Ry'
            Xj(0,0) = c;
            Xj(0,2) = -s;
            Xj(2,0) = s;
            Xj(2,2) = c;
            S(1) = 1;
            break;
        case 3: // revolute Z axis 'Rz'
            Xj(0,0) = c;
            Xj(0,1) = s;
            Xj(1,0) = -s;
            Xj(1,1) = c;
            S(2) = 1;
            break;
        case -1: // reversed revolute X axis '-Rx'
            Xj(1,1) = c;
            Xj(1,2) = s;
            Xj(2,1) = -s;
            Xj(2,2) = c;
            S(0) = -1;
            break;
        case -2: // reversed revolute Y axis '-Ry'
            Xj(0,0) = c;
            Xj(0,2) = -s;
            Xj(2,0) = s;
            Xj(2,2) = c;
            S(1) = -1;
            break;
        case -3: // reversed revolute Z axis '-Rz'
            Xj(0,0) = c;
            Xj(0,1) = s;
            Xj(1,0) = -s;
            Xj(1,1) = c;
            S(2) = -1;
            break;
        case 4: // prismatic X axis 'Px'
            Xj(4,2) = q;
            Xj(5,1) = -q;
            S(3) = 1;
            break;
        case 5: // prismatic Y axis 'Py'
            Xj(3,2) = -q;
            Xj(5,0) = q;
            S(4) = 1;
            break;
        case 6: // prismatic Z axis 'Pz'
            Xj(3,1) = q;
            Xj(4,0) = -q;
            S(5) = 1;
            break;
        case -4: // reversed prismatic X axis '-Px'
            Xj(4,2) = -q;
            Xj(5,1) = q;
            S(3) = -1;
            break;
        case -5: // reversed prismatic Y axis '-Py'
            Xj(3,2) = q;
            Xj(5,0) = -q;
            S(4) = -1;
            break;
        case -6: // reversed prismatic Z axis '-Pz'
            Xj(4,2) = q;
            Xj(5,1) = -q;
            S(5) = -1;
            break;
        default:
            throw std::invalid_argument("spatial.hpp: jcalc(): unknown joint type!");
            break;
    }
    
    if (fabs(jtyp) <= 3) {
        Xj.bottomRightCorner(3,3) = Xj.topLeftCorner(3,3);
    }
}

void jcalc(Mat6& Xj, 
           Mat6& dXjdt,
           Vec6& S, 
           const int jtyp, 
           const double q,
           const double q_d) 
{
    Xj.setIdentity();
    dXjdt.setZero();
    S.setZero();

    double c = cos(q);
    double s = sin(q);
    double dcdt = -s * q_d;
    double dsdt = c * q_d;

    if (jtyp < 0) { // reversed direction
        // c = cos(-q) = cos(q)
        s = -s; // s = sin(-q) = -sin(q)
    }

    switch (jtyp)
    {
        case 1: // revolute X axis 'Rx'
            Xj(1,1) = c;
            Xj(1,2) = s;
            Xj(2,1) = -s;
            Xj(2,2) = c;
            dXjdt(1,1) = dcdt;
            dXjdt(1,2) = dsdt;
            dXjdt(2,1) = -dsdt;
            dXjdt(2,2) = dcdt;
            S(0) = 1;
            break;
        case 2: // revolute Y axis 'Ry'
            Xj(0,0) = c;
            Xj(0,2) = -s;
            Xj(2,0) = s;
            Xj(2,2) = c;
            dXjdt(0,0) = dcdt;
            dXjdt(0,2) = -dsdt;
            dXjdt(2,0) = dsdt;
            dXjdt(2,2) = dcdt;
            S(1) = 1;
            break;
        case 3: // revolute Z axis 'Rz'
            Xj(0,0) = c;
            Xj(0,1) = s;
            Xj(1,0) = -s;
            Xj(1,1) = c;
            dXjdt(0,0) = dcdt;
            dXjdt(0,1) = dsdt;
            dXjdt(1,0) = -dsdt;
            dXjdt(1,1) = dcdt;
            S(2) = 1;
            break;
        case -1: // reversed revolute X axis '-Rx'
            Xj(1,1) = c;
            Xj(1,2) = s;
            Xj(2,1) = -s;
            Xj(2,2) = c;
            dXjdt(1,1) = dcdt;
            dXjdt(1,2) = dsdt;
            dXjdt(2,1) = -dsdt;
            dXjdt(2,2) = dcdt;
            S(0) = -1;
            break;
        case -2: // reversed revolute Y axis '-Ry'
            Xj(0,0) = c;
            Xj(0,2) = -s;
            Xj(2,0) = s;
            Xj(2,2) = c;
            dXjdt(0,0) = dcdt;
            dXjdt(0,2) = -dsdt;
            dXjdt(2,0) = dsdt;
            dXjdt(2,2) = dcdt;
            S(1) = -1;
            break;
        case -3: // reversed revolute Z axis '-Rz'
            Xj(0,0) = c;
            Xj(0,1) = s;
            Xj(1,0) = -s;
            Xj(1,1) = c;
            dXjdt(0,0) = dcdt;
            dXjdt(0,1) = dsdt;
            dXjdt(1,0) = -dsdt;
            dXjdt(1,1) = dcdt;
            S(2) = -1;
            break;
        case 4: // prismatic X axis 'Px'
            Xj(4,2) = q;
            Xj(5,1) = -q;
            dXjdt(4,2) = q_d;
            dXjdt(5,1) = -q_d;
            S(3) = 1;
            break;
        case 5: // prismatic Y axis 'Py'
            Xj(3,2) = -q;
            Xj(5,0) = q;
            dXjdt(3,2) = -q_d;
            dXjdt(5,0) = q_d;
            S(4) = 1;
            break;
        case 6: // prismatic Z axis 'Pz'
            Xj(3,1) = q;
            Xj(4,0) = -q;
            dXjdt(3,1) = q_d;
            dXjdt(4,0) = -q_d;                                                                                                                                                                                                                                                                                                                                                                          
            S(5) = 1;
            break;
        case -4: // reversed prismatic X axis '-Px'
            Xj(4,2) = -q;
            Xj(5,1) = q;
            dXjdt(4,2) = -q_d;
            dXjdt(5,1) = q_d;
            S(3) = -1;
            break;
        case -5: // reversed prismatic Y axis '-Py'
            Xj(3,2) = q;
            Xj(5,0) = -q;
            dXjdt(3,2) = q_d;
            dXjdt(5,0) = -q_d;
            S(4) = -1;
            break;
        case -6: // reversed prismatic Z axis '-Pz'
            Xj(4,2) = q;
            Xj(5,1) = -q;
            dXjdt(4,2) = q_d;
            dXjdt(5,1) = -q_d;
            S(5) = -1;
            break;
        default:
            throw std::invalid_argument("spatial.hpp: jcalc(): unknown joint type!");
            break;
    }
    
    if (fabs(jtyp) <= 3) {
        Xj.bottomRightCorner(3,3) = Xj.topLeftCorner(3,3);
        dXjdt.bottomRightCorner(3,3) = dXjdt.topLeftCorner(3,3);
    }
}

Mat6 crm(const Vec6& v) 
{
    Mat6 vcross;
    vcross.setZero();

    // vcross = [  0    -v(3)  v(2)   0     0     0    ;
    //             v(3)  0    -v(1)   0     0     0    ;
    //             -v(2)  v(1)  0      0     0     0    ;
    //             0    -v(6)  v(5)   0    -v(3)  v(2) ;
    //             v(6)  0    -v(4)   v(3)  0    -v(1) ;
    //             -v(5)  v(4)  0     -v(2)  v(1)  0 ];

    vcross(0, 1) = -v(2);
    vcross(0, 2) = v(1);
    vcross(1, 0) = v(2);
    vcross(1, 2) = -v(0);
    vcross(2, 0) = -v(1);
    vcross(2, 1) = v(0);
    vcross.bottomRightCorner(3,3) = vcross.topLeftCorner(3,3);

    vcross(3, 1) = -v(5);
    vcross(3, 2) = v(4);
    vcross(4, 0) = v(5);
    vcross(4, 2) = -v(3);
    vcross(5, 0) = -v(4);
    vcross(5, 1) = v(3);

    return vcross;
}

Mat3 skew(const Vec3& v) 
{
    Mat3 skew;
    skew.setZero();

    skew(0, 1) = -v(2);
    skew(0, 2) = v(1);
    skew(1, 0) = v(2);
    skew(1, 2) = -v(0);
    skew(2, 0) = -v(1);
    skew(2, 1) = v(0);

    return skew;
}

Vec3 skew(const Mat3& m) 
{
    Vec3 res;
    res << m(2,1) - m(1,2), 
           m(0,2) - m(2,0), 
           m(1,0) - m(0,1);
    return res;
}

Mat6 plux(const Mat3& R, const Vec3& p) 
{
    Mat6 res;
    res << R,            Mat3::Zero(),
           -R * skew(p), R;
    return res;
}

} // namespace Spatial
} // namespace Roahm