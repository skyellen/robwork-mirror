/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_GEOMETRY_OBVDISTANCECALC_HPP_
#define RW_GEOMETRY_OBVDISTANCECALC_HPP_


template<class COLLIDER, class BVTYPE>
class OBVDistanceCalc {
public:
    typedef BVTYPE BVType;
    typedef typename BVTYPE::value_type value_type;

    OBVDistanceCalc(){};

    virtual ~OBVDistanceCalc(){};

    inline value_type distance(const BVTYPE& a, const BVTYPE& b, const rw::math::Transform3D<>& aTb){
        return ((COLLIDER*)this)->distance(a,b,aTb);
    }

    inline value_type distance(const BVTYPE& a, const BVTYPE& b){
        return ((COLLIDER*)this)->distance(a,b);
    }

};


#endif /* BVCOLLIDER_HPP_ */
