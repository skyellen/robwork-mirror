#ifndef BODYINTEGRATOR_HPP_
#define BODYINTEGRATOR_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/InertiaMatrix.hpp>
#include <rw/kinematics/State.hpp>

namespace dynamics {

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
#endif /*BODYINTEGRATOR_HPP_*/
