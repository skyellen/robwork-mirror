#ifndef RWSIM_SIMULATOR_EULERINTEGRATOR_HPP_
#define RWSIM_SIMULATOR_EULERINTEGRATOR_HPP_

#include "BodyIntegrator.hpp"

#include <rw/math/EAA.hpp>

#include "RWBody.hpp"

namespace rwsim {
namespace simulator {

	/**
	 * @brief body motion integrator that use an implicit Euler formulation to calculate
	 * the motion of a body
	 */
    class EulerIntegrator : public BodyIntegrator
    {
    public:

    	/**
    	 * @brief constructor
    	 */
        EulerIntegrator(RWBody *body);

        //! destructor
    	virtual ~EulerIntegrator(){};

    	//!@copydoc BodyIntegrator::updatePosition
        void updatePosition(double h, rw::kinematics::State& state);

    	//!@copydoc BodyIntegrator::updateVelocity
        void updateVelocity(double h, rw::kinematics::State& state);

    private:
        RWBody *_body;
    };

}
}

#endif /*EULERINTEGRATOR_HPP_*/
