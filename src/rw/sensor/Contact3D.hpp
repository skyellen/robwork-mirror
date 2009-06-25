/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_SENSOR_CONTACT3D_HPP
#define RW_SENSOR_CONTACT3D_HPP

#include <rw/math/Vector3D.hpp>

class Contact3D {
public:
    Contact3D():mu(0.6){}

    Contact3D(rw::math::Vector3D<> tp,
    		  rw::math::Vector3D<> tn,
    		  double normalf
              ):p(tp),n(tn),f(n*normalf),normalForce(normalf),mu(0.6)
    {
    }

    Contact3D(rw::math::Vector3D<> tp,
    		  rw::math::Vector3D<> tn,
    		  rw::math::Vector3D<> tf
              ):p(tp),n(tn),f(tf),mu(0.6)
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


#endif /*RW_SENSOR_CONTACT3D_HPP*/

