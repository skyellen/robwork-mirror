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

#ifndef RW_GEOMETRY_BVDISTANCECALC_HPP_
#define RW_GEOMETRY_BVDISTANCECALC_HPP_

#include <rw/math/Transform3D.hpp>

template<class COLLIDER, class BVTYPE>
class BVDistanceCalc {
public:
    typedef BVTYPE BVType;
    typedef typename BVTYPE::value_type value_type;

    BVDistanceCalc(){};

    virtual ~BVDistanceCalc(){};

    inline value_type distance(const BVTYPE& a, const BVTYPE& b, const rw::math::Transform3D<>& aTb){
        return distance(a,b,aTb.P());
    }

    inline value_type distance(const BVTYPE& a, const BVTYPE& b, const rw::math::Vector3D<>& aTb){
        return ((COLLIDER*)this)->distance(a,b,aTb);
    }

    inline value_type distance(const BVTYPE& a, const BVTYPE& b){
        return ((COLLIDER*)this)->distance(a,b);
    }

};


#endif /* BVCOLLIDER_HPP_ */
