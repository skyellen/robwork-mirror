/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef BVDISTANCECALC_HPP_
#define BVDISTANCECALC_HPP_


template<class COLLIDER, class BVTYPE>
class BVDistanceCalc {
public:
    typedef BVTYPE BVType;
    typedef typename BVTYPE::value_type value_type;

    BVDistanceCalc(){};

    virtual ~BVDistanceCalc(){};

    virtual double distance(const BVTYPE& a, const BVTYPE& b,
                                const rw::math::Transform3D<>& aTb) = 0;

};


#endif /* BVCOLLIDER_HPP_ */
