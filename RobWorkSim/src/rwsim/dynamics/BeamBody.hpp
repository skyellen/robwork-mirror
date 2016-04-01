#ifndef RWSIM_DYNAMICS_BEAMBODY_HPP_
#define RWSIM_DYNAMICS_BEAMBODY_HPP_

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
    //! @addtogroup rwsim_dynamics
    //! @{

    /**
     * @brief A deformable body composed of a sequence of connected rigid bodies.
     *
     * The beam body models deformations that are approximated with relatively stiff
     * joints and small deformations. eg. the bending of a long metal beam due to
     * gravitational effects.
     *
     */
    class BeamBody: public Body {
    public:
        /**
         * @brief calculates the relative velocity of a point p on the body
         * described in world frames.
         */
        virtual rw::math::Vector3D<>
        	getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const = 0;

        /**
         * @brief reset the state variables of this body
         */
        virtual void reset(rw::kinematics::State &state) = 0;

        //! @copydoc Body::calcEnergy
        double calcEnergy(const rw::kinematics::State& state,
        		const rw::math::Vector3D<>& gravity = rw::math::Vector3D<>::zero(),
				const rw::math::Vector3D<>& potZero = rw::math::Vector3D<>::zero()) const { return 0; };

        /**
         * @brief Sets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state) = 0;

        /**
         * @brief Gets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const = 0;

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state) = 0;

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in body frame
         */
        virtual void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state) = 0;

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state) = 0;

        /**
         * @brief returns torque described in body frame
         */
        virtual rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const = 0;


    private:
        std::vector<rw::math::Vector3D<> > _segments;
    };
    //! @}
}
}
#endif /* SOFTBODY_HPP_ */
