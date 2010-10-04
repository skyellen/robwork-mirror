#include "LuaMath.hpp"

using namespace rwlibs::lua;
#include <iostream>
using namespace std;
#include <sstream>

#define NS rw::math

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

/*

Q::Q(int n, double vals[]):rw::math::Q(n,vals){}
Q::Q(const rw::math::Q& q):rw::math::Q(q){}

Q Q::operator-() const {return Q(- *((NS::Q*)this)); };
Q Q::operator-(const Q& b){ return Q(*((NS::Q*)this)-b); };
Q Q::operator+(const Q& b){ return Q(*((NS::Q*)this)+b); };
Q Q::operator*(double s){ return Q(*((NS::Q*)this)*s); };
Q Q::operator/(double s){ return Q(*((NS::Q*)this)/s); };
bool Q::operator==(const Q& q){return *((NS::Q*)this)== q; }
std::string Q::__tostring() const{
	return toString(*this);
};
*/

Vector3D::Vector3D(double x,double y, double z):NS::Vector3D<double>(x,y,z){};
Vector3D::Vector3D(const NS::Vector3D<double>& v):NS::Vector3D<double>(v){};

Vector3D Vector3D::operator*(double scale) const { return (*((NS::Vector3D<>*)this))* scale;};
Vector3D Vector3D::operator/(double s){ return (*((NS::Vector3D<>*)this))/s;};
Vector3D Vector3D::operator+(const Vector3D& other) const{ return (*((NS::Vector3D<>*)this))+other;};
Vector3D Vector3D::operator-(const Vector3D& other) const{ return (*((NS::Vector3D<>*)this))-other;};
bool Vector3D::operator==(const Vector3D& q){ return (*((NS::Vector3D<>*)this))==q;};
std::string Vector3D::__tostring() const{ return toString(*this); };



Rotation3D::Rotation3D(double vals[9]):
    NS::Rotation3D<double>(
            vals[0],vals[1],vals[2],
            vals[3],vals[4],vals[5],
            vals[6],vals[7],vals[8]){};
Rotation3D::Rotation3D(const Vector3D& i, const Vector3D& j, const Vector3D& k):
    NS::Rotation3D<double>(i,j,k){};
Rotation3D::Rotation3D(const rw::math::Rotation3D<double>& R):
    NS::Rotation3D<double>(R){};

Rotation3D Rotation3D::operator*(const Rotation3D& other) const{return (*((NS::Rotation3D<>*)this))*other;};
Vector3D Rotation3D::operator*(const Vector3D& vec) const{return (*((NS::Rotation3D<>*)this))*vec;};
Rotation3D Rotation3D::inverse() const{return NS::inverse(*this);};
EAA Rotation3D::operator*(const EAA& other) const{ return (*((NS::Rotation3D<>*)this))*other; };
//bool Rotation3D::operator==(const Rotation3D &rhs) const{return NS::operator ==(*this,rhs);};
//bool Rotation3D::equal(const Rotation3D& rot, double precision)
std::string Rotation3D::__tostring() const{ return toString(*this); };



EAA::EAA(const rw::math::EAA<double>& eaa): NS::EAA<double>(eaa){};
EAA::EAA(const rw::math::Rotation3D<double>& rot):NS::EAA<double>(rot){};
EAA::EAA(double vals[3]):NS::EAA<double>(vals[0],vals[1],vals[2]){};
EAA::EAA(const Vector3D& v1, const Vector3D& v2):NS::EAA<double>(v1,v2){};

bool EAA::operator==(const EAA &rhs) const{
	return fabs((*this)[0]-rhs[0])<0.0000001 && fabs((*this)[1]-rhs[1])<0.0000001 && fabs((*this)[2]-rhs[2])<0.0000001;
};
std::string EAA::__tostring() const{ return toString(*this); };




RPY::RPY(const rw::math::RPY<double>& eaa): NS::RPY<double>(eaa){};
RPY::RPY(const rw::math::Rotation3D<double>& rot):NS::RPY<double>(rot){};
RPY::RPY(double vals[3]):NS::RPY<double>(vals[0],vals[1],vals[2]){};

bool RPY::operator==(const RPY &rhs) const{
	return fabs((*this)(0)-rhs(0))<0.0000001 && fabs((*this)(1)-rhs(1))<0.0000001 && fabs((*this)(2)-rhs(2))<0.0000001;
};
std::string RPY::__tostring() const{ return toString(*this); };




Quaternion::Quaternion(const rw::math::Quaternion<double>& eaa): NS::Quaternion<double>(eaa){};
Quaternion::Quaternion(const rw::math::Rotation3D<double>& rot):NS::Quaternion<double>(rot){};
Quaternion::Quaternion(double vals[4]):NS::Quaternion<double>(vals[0],vals[1],vals[2],vals[4]){};

Quaternion Quaternion::operator*(const Quaternion& other) const{
    return NS::Quaternion<>((*(NS::Quaternion<>*)this) * *((NS::Quaternion<>*)&other));
};
Quaternion Quaternion::operator*(double s){return (*((NS::Quaternion<>*)this))*s;};
bool Quaternion::operator==(const Quaternion &rhs) const{
	return fabs((*this)(0)-rhs(0))<0.0000001 && fabs((*this)(1)-rhs(1))<0.0000001 && fabs((*this)(2)-rhs(2))<0.0000001 && fabs((*this)(3)-rhs(3))<0.0000001;
};
std::string Quaternion::__tostring() const{ return toString(*this); };



Transform3D::Transform3D(const rw::math::Transform3D<double>& t3d):
    NS::Transform3D<double>(t3d){};
Transform3D::Transform3D(
    const Vector3D& position,
    const Rotation3D& rotation):
        NS::Transform3D<double>(position, rotation)
{}

Transform3D Transform3D::operator*(const Transform3D& other) const{
    return (*((NS::Transform3D<>*)this)) *other;
}
Vector3D Transform3D::operator*(const Vector3D& other) const{
    return (*((NS::Transform3D<>*)this))*other;
}
Transform3D Transform3D::inverse() const{
    return NS::inverse(*this);
}
//Vector3D Transform3D::P() const;
//Rotation3D Transform3D::R() const;
std::string Transform3D::__tostring() const{ return toString(*this); };




Pose6D::Pose6D(const rw::math::Pose6D<double>& p6d):NS::Pose6D<double>(p6d){};
Pose6D::Pose6D(const Vector3D& position,const EAA& rotation):NS::Pose6D<double>(position, rotation){};
Pose6D::Pose6D(const rw::math::Transform3D<double>& t3d):NS::Pose6D<double>(t3d){};

//Transform3D Pose6D::toTransform3D();
//Vector3D P() const;
//EAA R() const;
std::string Pose6D::__tostring() const{ return toString(*this); };

Rotation3D rwlibs::lua::inverse(const Rotation3D& val){return val.inverse();}
Transform3D rwlibs::lua::inverse(const Transform3D& val){return val.inverse();}
