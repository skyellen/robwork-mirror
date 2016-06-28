/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SemiForceClosureFilter.hpp"
#include "Grasp3D.hpp"

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
