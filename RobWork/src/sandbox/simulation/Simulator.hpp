#ifndef _SIMULATOR_HPP_
#define _SIMULATOR_HPP_

#include <rw/kinematics/State.hpp>

namespace rwlibs {
namespace simulation {


class Simulator
{
public:

	/**
	 * @brief
	 */
	virtual ~Simulator(){};

	/**
	 * @brief Performs a step and updates the state
	 */
	virtual void step(double dt, rw::kinematics::State& state) = 0;

	/**
	 * @brief reset velocity and acceleration of all bodies to 0. And sets the position of all bodies
	 * to that described in state
	 */
	virtual void reset(rw::kinematics::State& state) = 0;

	/**
	 * @brief initialize simulator physics with state
	 */
	virtual void init(rw::kinematics::State& state) = 0;

	/**
	 * @brief gets the the current simulated time
	 */
	virtual double getTime() = 0;

	/**
	 * Enables or disables simulation of a frame
	 * @param frame
	 * @param enabled
	 */
	virtual void setEnabled(rw::kinematics::Frame* frame, bool enabled) = 0;

	/**
	 * @brief
	 */
	virtual rw::common::PropertyMap& getPropertyMap() = 0;

	/**
	 * @brief
	 */
	virtual void emitPropertyChanged() = 0;

};

}
}

#endif /*Simulator_HPP_*/

