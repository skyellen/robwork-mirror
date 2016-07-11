/*
 * BoxPoseSampler.hpp
 *
 *  Created on: Aug 23, 2011
 *      Author: jimali
 */

#ifndef BOXPOSESAMPLER_HPP_
#define BOXPOSESAMPLER_HPP_

#include "PoseSampler.hpp"

#include <rw/math/Math.hpp>

class BoxPoseSampler: public PoseSampler {
public:

    BoxPoseSampler(rw::math::Vector3D<> minPos, rw::math::Vector3D<> maxPos):
        _minPos(minPos),_maxPos(maxPos),
        _randomRotation(true)
    {
    }
    BoxPoseSampler(rw::math::Vector3D<> minPos,
                   rw::math::Vector3D<> maxPos,
                   rw::math::RPY<> minRpy,
                   rw::math::RPY<> maxRpy):
                       _minPos(minPos),_maxPos(maxPos),
                       _minRpy(minRpy),_maxRpy(maxRpy),
                       _randomRotation(false)
    {
    }

    rw::math::Transform3D<> sample(){
        using namespace rw::math;
        Vector3D<> position(Math::ran(_minPos[0],_maxPos[0]), Math::ran(_minPos[1],_maxPos[1]),Math::ran(_minPos[2],_maxPos[2]));
        if(_randomRotation){
            Transform3D<> target( position, Math::ranRotation3D<double>());
            return target;
        } else {
            RPY<> rpy(Math::ran(_minRpy(0),_maxRpy(0)), Math::ran(_minRpy(1),_maxRpy(1)),Math::ran(_minRpy(2),_maxRpy(2)));
            Transform3D<> target( position, rpy.toRotation3D());
            return target;
        }
    }


private:
    rw::math::Vector3D<> _minPos, _maxPos;
    rw::math::RPY<> _minRpy, _maxRpy;
    bool _randomRotation;
};


#endif /* BOXPOSESAMPLER_HPP_ */
