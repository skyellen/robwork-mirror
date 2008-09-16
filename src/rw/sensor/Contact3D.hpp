#ifndef CONTACT3D_HPP_
#define CONTACT3D_HPP_

#include <rw/math/Vector3D.hpp>

class Contact3D {
public:
    Contact3D():mu(0.6){}
    Contact3D(rw::math::Vector3D<> tp,
              rw::math::Vector3D<> tf,
              rw::math::Vector3D<> tn):p(tp),n(tn),f(tf),mu(0.6)
    {
         normalForce =  dot(f, n);
    }
    rw::math::Vector3D<> p; // Contact position
    rw::math::Vector3D<> n; // Surface contact normal
    rw::math::Vector3D<> f; // the actual force
    double normalForce;

    // hmm, dunno about 3d curvature
    double curvature; // surface curvature
    double mu; // coulomb friction coefficient
};


#endif /*CONTACT3D_HPP_*/

