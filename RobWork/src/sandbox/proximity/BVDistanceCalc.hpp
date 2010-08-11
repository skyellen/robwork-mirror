/*
 * BVCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef BVDISTANCECALC_HPP_
#define BVDISTANCECALC_HPP_

#include "BV.hpp"

class BVDistanceCalc {
public:
    BVDistanceCalc(){};

    virtual ~BVDistanceCalc(){};

    virtual double calcDistance(const BV& a, const BV& b,
                                const rw::math::Transform3D<>& aTb) = 0;

};


#endif /* BVCOLLIDER_HPP_ */
