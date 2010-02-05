/*
 * FiniteStateSampler.hpp
 *
 *  Created on: 23-09-2009
 *      Author: jimali
 */

#ifndef SPHEREPOSESAMPLER_HPP_
#define SPHEREPOSESAMPLER_HPP_

#include "StateSampler.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/MovableFrame.hpp>

/**
 * @brief samples poses of a movable frame, such that the frame is always
 * positioned on a sphere around some specified center. Random deviations
 * to the position of the frame can be added.
 *
 * This StateSampler will never become empty
 *
 */
class SpherePoseSampler: public StateSampler
{
public:

    /**
     * @brief
     *
     * @param mframe
     * @param wPc
     * @param initState [in] the initial state
     * @return
     */
    SpherePoseSampler(rw::kinematics::MovableFrame* mframe,
                      const rw::math::Vector3D<>& wPc,
                      rw::kinematics::State& initState);

    virtual ~SpherePoseSampler();

    bool sample(rw::kinematics::State& state);

    bool empty() const{ return false; };

private:
    rw::kinematics::MovableFrame* _mframe;
    rw::math::Vector3D<> _wPc;
    rw::kinematics::State _initState;
};

#endif /* FINITESTATESAMPLER_HPP_ */
