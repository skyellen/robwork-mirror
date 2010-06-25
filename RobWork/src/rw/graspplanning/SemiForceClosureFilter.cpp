/*
 * PlaneClearanceFilter.hpp
 *
 *  Created on: 04-07-2009
 *      Author: jimali
 */

#include "SemiForceClosureFilter.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::math;
using namespace rw::graspplanning;

bool SemiForceClosureFilter::isValid(const Grasp3D& grasp){
    if(_nrContacts!=grasp.contacts.size())
        RW_THROW("The number of contacts does not match!");
    // calculate the average of all contact normals and take the opposite
    // as guess for Fext
    Vector3D<> fext2;
    for(size_t i=0; i<_nrContacts;i++){
        fext2 += grasp.contacts[i].n;
    }
    fext2 = -(fext2*_avgScale);

    // now check if the approximated fext is able to break the force-closure
    for(size_t i=0; i<_nrContacts;i++){
        double angle = acos( dot(grasp.contacts[i].n,fext2) );
        //std::cout << "fabs(angle) < Pi/2+atanMU  --> " << fabs(angle) <<" < "<<  Pi/2+atanMU << std::endl;
        if( fabs(angle) < Pi/2+atan( grasp.contacts[i].mu ) ){
            return true;
        }
    }
    return false;
}
