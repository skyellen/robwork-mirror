#ifndef RW_INVERSEKINEMATIOCS_IKMETASOLVER_HPP
#define RW_INVERSEKINEMATIOCS_IKMETASOLVER_HPP

#include "IterativeIK.hpp"

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace invkin {

	/** \addtogroup invkin */
	/*@{*/

	/**
     * @brief Solve the inverse kinematics problem with respect to joint limits and
     * collisions.
     *
     * Given an arbitrary iterative inverse kinematics solver, the IKMetaSolver
     * attempts to find a collision free solution satisfying joint limits. It
     * repeatingly calls the iterative solver with new random start configurations
     * until either a solution is found or a specified max attempts has been
     * reached.
     */
    class IKMetaSolver: public IterativeIK
    {
    public:
        /**
         * @brief Constructs IKMetaSolver
         *
         * The IKMetaSolver takes ownership of the \b iksolver. The IKMetaSolver
         * does NOT take ownership of the \b collisionDetector. To skip testing for
         * collision use null as collision detector
         *
         * @param iksolver [in] The basic iksolver to use. Ownership is taken
         * @param device [in] Device to solve for
         *
         * @param collisionDetector [in] CollisionDetector to use. If null no
         * collision detection used.
         */
        IKMetaSolver(
            IterativeIK* iksolver,
            rw::models::Device* device,
            rw::proximity::CollisionDetector* collisionDetector);

        /**
         * @brief Descrutor
         */
        virtual ~IKMetaSolver();

        /**
         * @copydoc IterativeIK::solve
         *
         * Searches for a valid solution using the parameters set in the IKMetaSolver
         */
        std::vector<rw::math::Q> solve(
            const math::Transform3D<>& baseTend,
            const kinematics::State& state) const;

        /**
         * @brief Sets up the maximal number of attempts
         * @param maxAttempts [in] Maximal number of attempts
         */
        void setMaxAttempts(size_t maxAttempts);

        /**
         * @brief Sets whether to stop searching after the first solution
         * @param stopAtFirst [in] True to stop after first solution has been found
         */
        void setStopAtFirst(bool stopAtFirst);

        /**
         * @brief Solves the inverse kinematics problem
         *
         * Tries to solve the inverse kinematics problem using the iterative
         * inverse kinematics solver provided in the constructor. It tries at
         * most \b cnt times and can either be stopped after the first solution
         * is found or continue to search for solutions. If multiple solutions
         * are returned there might be duplicates in the list.
         *
         * @param baseTend [in] Desired base to end transform
         * @param state [in] State of the workcell
         * @param cnt [in] Maximal number of attempts
         *
         * @param stopatfirst [in] If true the method will return after the first
         * solution is found. If false it will continue searching for more solution
         * until the maximal number of attemps is met.
         */
        std::vector<rw::math::Q> solve(
            const math::Transform3D<>& baseTend,
            const kinematics::State& state,
            size_t cnt,
            bool stopatfirst) const;

    private:
        IterativeIK* _iksolver;
        rw::proximity::CollisionDetector* _collisionDetector;
        rw::models::Device* _device;
        std::pair<rw::math::Q, rw::math::Q> _bounds;
        size_t _dof;

        size_t _maxAttempts;
        bool _stopAtFirst;

        bool betweenLimits(const rw::math::Q& q) const;

        rw::math::Q getRandomConfig() const;
    };

	/*@}*/

}} // end namespaces

#endif // end include guard
