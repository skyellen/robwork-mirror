#include "CG3IKSolver2D.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::graspplanning;

namespace {
    Vector3D<> toVector3D(const Vector2D<>& v){
        return Vector3D<>(v(0),v(1),0);
    }

    Vector2D<> toVector2D(const Vector3D<>& v){
        return Vector2D<>(v(0),v(1));
    }

    double getWFTipLen(const double x, const double y, const double z, const Vector3D<>& f1){
        const double f1x = f1(0);
        const double f1y = f1(1);
        return std::pow(-f1x + x,2) + std::pow(-f1y + y,2) + std::pow(z,2);
    }

    std::pair<double,double>
        calcFingerConfig(const Vector3D<>& fbPftcp,
                         const double a, const double b,
                         const State& state,
                         std::pair<Q,Q> bounds)
    {
        const Vector2D<> fbPftcp2d(fbPftcp(0),fbPftcp(1));
        double c = fbPftcp2d.norm2();

        // Test if the finger tcp can actually reach the position
        if( a+b-0.005 < c ){
            return std::make_pair(-1.0,-1.0);
        }
       //std::cout << "Vector: " << fbPftcp2d << std::endl;
        const double acosTmp = (a*a+b*b-c*c)/(2*a*b);
        if( acosTmp>=1 || acosTmp<=-1 )
            return std::make_pair(-1.0,-1.0);
        const double angleTip = acos( acosTmp );

        const double asinTmp = (sin(angleTip)*b)/c;
        if( asinTmp>=1 || asinTmp<=-1 )
            return std::make_pair(-1.0,-1.0);
        const double angleMid = asin( asinTmp )  ;

        const double angCorrection = atan2(fbPftcp2d(1),fbPftcp2d(0));
       //std::cout << "angleTip: " << angleTip*Rad2Deg << std::endl;
       //std::cout << "angleMid: " << angleMid*Rad2Deg << std::endl;
       //std::cout << "angCorrection: " << angCorrection*Rad2Deg << std::endl;

        double aMid = Pi/2 - angCorrection + angleMid;
        double j3Angle = Pi-fabs(angleTip);
        if( j3Angle<bounds.first(0) || bounds.second(0)<j3Angle )
            return std::make_pair(-1.0,-1.0);

        return std::make_pair(-aMid,j3Angle);
    }

    double angleToPlane(Vector3D<> v, Vector3D<> n){
        double dotProd = dot(n,v)/(v.norm2()*n.norm2());
        if( dotProd>=1 || dotProd<=-1 )
            return 0;
        return std::acos( dotProd )-Pi/2;
    }

}

CG3IKSolver2D::CG3IKSolver2D(const rw::models::TreeDevice& device, const rw::kinematics::State& state):
    _device(device),
    _maxWristToFingerTip(0.18),
    _minWristHeight(0.06)
{
    std::vector<Frame*> ends = _device.getEnds();
    _thumbBase = _device.getJoints()[0];
    _thumbMid = _device.getJoints()[1];
    _thumbTcp = ends[0];
    _f1Base = _device.getJoints()[3];
    _f1Mid = _device.getJoints()[4];
    _f1Tcp = ends[1];
    _f2Base = _device.getJoints()[5];
    _f2Mid = _device.getJoints()[6];
    _f2Tcp = ends[2];

    init(state);
}

void CG3IKSolver2D::init(const rw::kinematics::State& state){
    _aThum = Kinematics::frameTframe(_thumbBase,_thumbMid,state).P().norm2();
    _bThum = Kinematics::frameTframe(_thumbMid,_thumbTcp, state).P().norm2();
    _af1 = Kinematics::frameTframe(_f1Base,_f1Mid,state).P().norm2();
    _bf1 = Kinematics::frameTframe(_f1Mid,_f1Tcp, state).P().norm2();
    _af2 = Kinematics::frameTframe(_f2Base,_f2Mid,state).P().norm2();
    _bf2 = Kinematics::frameTframe(_f2Mid,_f2Tcp, state).P().norm2();
}


CG3IKSolver2D::IKResult CG3IKSolver2D::solve(
        const Transform3D<>& bTobj, const Grasp2D& grasp, const State& stateInit){
    State state = stateInit;
    IKResult result;
    std::vector<std::pair<double, std::pair<Transform3D<>,Q > > > semires;
    // some usefull definitions
    //Vector3D<> center = toVector3D(grasp.center);
    // conact points
    const Vector3D<> c1 = toVector3D(grasp.contacts[0].p);
    const Vector3D<> c2 = toVector3D(grasp.contacts[1].p);
    const Vector3D<> c3 = toVector3D(grasp.contacts[2].p);
    // approach vectors
    const Vector3D<> a1 = toVector3D(grasp.approach[0]);
    const Vector3D<> a2 = toVector3D(grasp.approach[1]);
    const Vector3D<> a3 = toVector3D(grasp.approach[2]);
    const Vector3D<> center = toVector3D(grasp.center);
    // angles for determining 6d positions
    //double a2Toa1_angle = angle(grasp.approach[1],grasp.approach[0]);
    //double a3Toa1_angle = angle(grasp.approach[2],grasp.approach[0]);

    // Start by finding a suitable wrist transform
    // by placing the hand with the thumb in the x-axis direction
    // find a suitable wrist transform
    // the wrist must be placed such that the thumb is placed on f3
    // the x-axis of the wrist point toward the thumb that is in a3 direction
    // and the z-axis of the wrist point toward the object center

    // place the coupled fingers acoording to phi
    Q q = _device.getQ(state);
    q(0) = 0;
    q(1) = 0;
    q(2) = grasp.phi;
    q(3) = 0;
    q(4) = 0;
    q(5) = 0;
    q(6) = 0;
    _device.setQ(q,state);
   //std::cout << "Grasp phi : " << q(2) << std::endl;

    // now come the triggy part, placing the fingers

    std::vector<Frame*> ends = _device.getEnds();
    const Frame *base = _device.getBase();
    const Transform3D<> baseTtbInv = inverse(Kinematics::frameTframe( base , _thumbBase,state));
    const Transform3D<> baseTf1bInv = inverse(Kinematics::frameTframe( base , _f1Base,state));
    const Transform3D<> baseTf2bInv = inverse(Kinematics::frameTframe( base , _f2Base,state));
   //std::cout << baseTtb << std::endl;
   //std::cout << baseTf1b << std::endl;
   //std::cout << baseTf2b << std::endl;

    // allow at most 10 deg of error on each finger
    const double maxQuality = Math::sqr(20*Deg2Rad)*3;

    // TODO: could be nice if the pose was not too fixed... say changing the pose a bit could perhaps
    // generate better solutions

    // now search for some height z above the grasp contact plane such that x and y coordinates
    // are acceptable (if they are too large then the wrist need to tilt too much)
    for(double z=_maxWristToFingerTip; z>_minWristHeight; z-=0.005 ){
        // project c2 onto a1 and take the half to get the x value as the resulting length of the vector
        //Vector3D<> center = (dot(a1,c2-c1)/Math::sqr(a1.norm2())*0.5)*a1+c1;

        Vector3D<> zAxis = normalize(Vector3D<>(0,0,-z));
        // project c3 onto zAxis to get the xAxis
        Vector3D<> zAxisProj = (dot(a1,zAxis)/Math::sqr(zAxis.norm2()))*zAxis;
        Vector3D<> xAxis = normalize( a1-zAxisProj );
        Vector3D<> yAxis = cross(zAxis,xAxis);

        Vector3D<> objPgrasp(center(0),center(1),0);
        Rotation3D<> objRgrasp(xAxis,yAxis,zAxis);
        Transform3D<> objTgrasp( objPgrasp, objRgrasp);
        Transform3D<> graspTwrist(Vector3D<>(0,0,-z),Rotation3D<>::identity());
        Transform3D<> objTwristInv = inverse(objTgrasp*graspTwrist);

        Vector3D<> c1tmp = baseTtbInv  * objTwristInv * c1;
        Vector3D<> c2tmp = baseTf1bInv * objTwristInv * c3;
        Vector3D<> c3tmp = baseTf2bInv * objTwristInv * c2;

        //Vector3D<> zComF1Tcp = baseTtb * Vector3D<>(0,0,1);
        //std::cout << "only x and y components: " << zComF1Tcp << std::endl;
        //Vector2D<> zCF1Tcp = toVector2D(baseTtb * Vector3D<>(0,0,1));
        //Vector2D<> zCF2Tcp = toVector2D(baseTf1b * Vector3D<>(0,0,1));
        //Vector2D<> zCF3Tcp = toVector2D(baseTf2b * Vector3D<>(0,0,1));

        // for each finger calculate the configurations
        std::pair<double,double> res;
        res = calcFingerConfig(c1tmp, _aThum, _bThum, state, _thumbMid->getBounds());
       //std::cout << "Thum: " << _aThum << " " << _bThum << std::endl;
        q(0) = res.first;
        q(1) = res.second;

        res = calcFingerConfig(c2tmp, _af1, _bf1, state, _f1Mid->getBounds() );
        q(3) = res.first;
        q(4) = res.second;

        res = calcFingerConfig(c3tmp, _af2, _bf2, state,  _f2Mid->getBounds());
        q(5) = res.first;
        q(6) = res.second;

        if(q(1)<0 || q(4)<0 || q(6)<0)
            continue;

        _device.setQ(q, state);

        const Frame *base = _device.getBase();
        const Transform3D<> f0AngV = Kinematics::frameTframe(base,_thumbTcp, state);
        double angF0 = angleToPlane( f0AngV*Vector3D<>(0,0,1),Vector3D<>(0,0,1));

        const Transform3D<> f1AngV = Kinematics::frameTframe(base,_f1Tcp, state);
        double angF1 = angleToPlane(f1AngV*Vector3D<>(0,0,1),Vector3D<>(0,0,1) );

        const Transform3D<> f2AngV = Kinematics::frameTframe(base,_f2Tcp, state);
        double angF2 = angleToPlane(f2AngV*Vector3D<>(0,0,1), Vector3D<>(0,0,1) );
        // check if
        double quality = angF0*angF0 + angF1*angF1 + angF2*angF2;
        if( quality>maxQuality )
            continue;

        //std::cout << "ANGLES: (" << angF0 << "," << angF1 << "," << angF2 << ")" << std::endl;
        std::pair<Transform3D<>,Q > resc(bTobj*objTgrasp*graspTwrist,q);
        semires.push_back( std::make_pair(quality, resc) );
    }
    //std::cout << "------- results: " << semires.size() << std::endl;
    if(semires.size()==0)
        return result;
    int bestIdx = 0;
    for(size_t i=1;i<semires.size();i++){
        if(semires[i].first<semires[bestIdx].first)
            bestIdx = i;
    }
    result.push_back(semires[bestIdx].second);

    return result;
}
