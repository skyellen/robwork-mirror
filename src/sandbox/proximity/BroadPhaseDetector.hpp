/*
 * BroadPhaseDetector.hpp
 *
 *  Created on: 24-03-2009
 *      Author: jimali
 */

#ifndef BROADPHASEDETECTOR_HPP_
#define BROADPHASEDETECTOR_HPP_

namespace rw { namespace proximity {



class BroadPhaseDetector {
public:
	typedef std::iterator<std::forward_iterator_tag, rw::common::FramePair > Iterator;


	virtual void update(const rw::kinematics::State& state);

	Iterator begin();

	Iterator end();

};

}
}

#endif /* BROADPHASEDETECTOR_HPP_ */
