/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_IKQPSOLVER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_IKQPSOLVER_HPP

/**
 * @file IKQPSolver.hpp
 */

#include "QPController.hpp"

#include <rw/invkin/IterativeIK.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>

namespace rw { namespace models {
    class SerialDevice;
}} // end namespaces

namespace rwlibs { namespace algorithms {

    /** \addtogroup algorithms */
    /*@{*/

    /**
     * @brief Iterative inverse kinematics solved based on the QPController
     *
     * The IKQPSolver works very similar to the SimpleSolver. However, instead
     * of using a simple inverse Jacobian it uses the rw::algorithms::QPController
     * making it robust to singularities and ensuring joint limits.
     *
     * Usually the IKQPSolver runs order of magnitudes slower than the standard
     * SimpleSolver. Solutions returned by the IKQPSolver might not be exact
     * solutions, but are least square fits. Given a target outside the robot workspace
     * the IKQPSolver will, given enough iterations, return the least square solution.
     *
     * Notice that the IKQPSolver is a local method. It there is thus no guarantee that
     * the solution returned will be the global least square.
     */
    class IKQPSolver: public rw::invkin::IterativeIK {
    public:
        /**
         * @brief Constructs IKQPSolver for device
         *
         * It is required that the device has correct joint position, velocity
         * and acceleration limits.
         *
         * @param device [in] Device to solve for         * @param state [in] State of the workcell
         */
        IKQPSolver(rw::models::SerialDevice* device,
                   const rw::kinematics::State& state);

        /**
         * @copydoc rw::inversekinematics::IterativeIK::solve
         */
        std::vector<rw::math::Q> solve(const rw::math::Transform3D<>& baseTend,
                                       const rw::kinematics::State& state) const;

    private:
        QPController _qpcontroller;
        const rw::models::SerialDevice* _device;
        rw::common::PropertyMap _properties;
        rw::kinematics::State _state;
        double _maxQuatStep;

        bool performLocalSearch(const rw::models::SerialDevice *device,
                       			const rw::math::Transform3D<> &bTed,
                       			double maxError,
                       			rw::kinematics::State &state,
                       			unsigned int maxIter) const;
    };

}} // end namespaces

#endif // end include guard
