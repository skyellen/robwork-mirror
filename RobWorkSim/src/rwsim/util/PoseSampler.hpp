/*
 * PoseSampler.hpp
 *
 *  Created on: Aug 23, 2011
 *      Author: jimali
 */

#ifndef POSESAMPLER_HPP_
#define POSESAMPLER_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>

class PoseSampler {
public:
    typedef rw::common::Ptr<PoseSampler> Ptr;

    virtual rw::math::Transform3D<> sample() = 0;
};

#endif /* POSESAMPLER_HPP_ */
