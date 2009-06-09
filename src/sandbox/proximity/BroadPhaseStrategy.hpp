/*
 * BroadPhaseDetector.hpp
 *
 *  Created on: 24-03-2009
 *      Author: jimali
 */

#ifndef BROADPHASEDETECTOR_HPP_
#define BROADPHASEDETECTOR_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include "CollisionSetup.hpp"

namespace rw { namespace proximity { namespace sandbox {



/**
 * @brief describe the interphase of a broad phase proximity strategy or proximity culler.
 *
 *
 */
class BroadPhaseStrategy {
public:

	/**
	 * @brief
	 */
	virtual void reset(const rw::kinematics::State& state) = 0;

	/**
	 * @brief
	 */
	virtual void update(const rw::kinematics::State& state) = 0;

	virtual const rw::kinematics::FramePair& next() = 0;

	virtual bool hasNext() = 0;

	virtual CollisionSetup& getCollisionSetup() = 0;

};

typedef rw::common::Ptr<BroadPhaseStrategy> BroadPhaseStrategyPtr;

}
}
}

#endif /* BROADPHASEDETECTOR_HPP_ */
