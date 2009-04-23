/*
 * BroadPhaseDetector.hpp
 *
 *  Created on: 24-03-2009
 *      Author: jimali
 */

#ifndef BROADPHASEDETECTOR_HPP_
#define BROADPHASEDETECTOR_HPP_

namespace rw { namespace proximity {


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
	virtual void update(double dt, const rw::kinematics::State& state) = 0;

	virtual const FramePair& next() = 0;

	virtual bool hasNext() = 0;

};

}
}

#endif /* BROADPHASEDETECTOR_HPP_ */
