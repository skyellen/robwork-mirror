#ifndef DYNAMICS_BODYCONTROLLER_HPP_
#define DYNAMICS_BODYCONTROLLER_HPP_

#include <rw/kinematics/State.hpp>

#include "Body.hpp"

namespace dynamics {

    /**
     * @brief The body controller is a pure interface through which bodies are controlled
     */
    class BodyController
    {
    public:

    	/**
    	 * @brief add external forces to the bodies that this
    	 * BodyManipulator controls.
    	 */
    	virtual void addForces(rw::kinematics::State &state, double time) = 0;

    	/**
    	 * @brief resets the state of the body controller to \b state
    	 */
    	virtual void reset(rw::kinematics::State &state) = 0;

    	/**
    	 * @brief add external impulses to the bodies that this BodyController controls
    	 */
    	//virtual void addImpulses(rw::kinematic::State &state, double time) = 0;

    	/**
    	 * @brief return the list of bodies that this controller controls
    	 */
    	//virtual std::vector<Body*>& getBodies() = 0;

    };
}
#endif /*BODYCONTROLLER_HPP_*/
