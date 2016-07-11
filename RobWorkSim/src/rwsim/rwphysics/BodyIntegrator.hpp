#ifndef RWSIM_SIMULATOR_BODYINTEGRATOR_HPP_
#define RWSIM_SIMULATOR_BODYINTEGRATOR_HPP_

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {

	/**
	 * @brief abstract class for describing how the motion of a body behaves
	 * during a timestep
	 */
    class BodyIntegrator
    {
    protected:
        BodyIntegrator(){};

    public:

        /**
         * @brief destructor
         * @return
         */
    	virtual ~BodyIntegrator(){};

        /**
         * @brief integrates velocity over timestep h to update the position of the body
         */
        virtual void updatePosition(double h, rw::kinematics::State& state) = 0;

        /**
         * @brief integrates forces over timestep h to update the velocity of the body
         */
        virtual void updateVelocity(double h, rw::kinematics::State& state) = 0;

    };

}
}
#endif /*BODYINTEGRATOR_HPP_*/
