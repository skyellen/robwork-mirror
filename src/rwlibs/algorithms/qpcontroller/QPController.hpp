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

#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_QPCONTROLLER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_QPCONTROLLER_HPP

/**
 * @file QPController.hpp
 */

#include <cmath>
#include <boost/numeric/ublas/vector.hpp>

#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>

namespace rwlibs { namespace algorithms {

    /** @addtogroup algorithms */
    /* @{*/

    /**
     * The QPController provides an visual servoing control scheme, which is based on
     * quadratic optimization. The method computes the \f$\mathbf{\dot{q}}\f$ minimizing
     * \f$\|\mathbf{J}\mathbf{\dot{q}}-\mathbf{\dot{x}}\|^2\f$, subject to the joint position,
     * velocity and acceleration limits.
     *
     * See the paper: L.-P. Ellekilde, P. Favrhold, M. Paulin and H.G. Petersen, "Robust
     * Control for High-Speed Visual Servoing Applications", To appear in International
     * Journal for Advanced Robotic Systems, vol. 4, no. 3, 2007.
     */
    class QPController {
    public:
        /**
         * @brief Construct QPController object
         * @param h [in] step-size to use
         * @param state [in] state of the workcell
         * @param device [in] device to control
         */
        QPController(double h,
                     const rw::kinematics::State& state,
                     rw::models::Device* device);

        /**
         * @brief destructor
         */
        virtual ~QPController();

        /**
         * @brief Computes joint velocities for a tool velocity
         *
         * The method provides the, in a least square sense, optimal joint
         * velocity for the specified tool velocity screw. That is the
         * \f$\mathbf{\dot{q}}\f$ minimizing \f$\|\mathbf{J}\mathbf{\dot{q}}-\mathbf{\dot{x}}\|^2\f$,
         * subject to the joint position, velocity and acceleration limits.
         *
         * @param q [in] current device joint configuration
         * @param dq [in] current device joint velocity
         * @param tcpscrew [in] desired velocity screw of tcp
         *
         * @return the optimal joint velocity
         */
        rw::math::Q solve(const rw::math::Q& q,
                          const rw::math::Q& dq,
                          const rw::math::VelocityScrew6D<>& tcpscrew);

    private:
        typedef rw::math::Q::Base QBase;

        QBase inequalitySolve(const boost::numeric::ublas::matrix<double>& G,
        				      const QBase& b,
        				      const QBase& lower,
        				      const QBase& upper);

        void calculateVelocityLimits(QBase& lower,
        							 QBase& upper,
        							 const QBase& q,
        							 const QBase& dq);

        double _h;
        size_t _n;
        size_t _cnt;
        rw::models::Device* _device;
        rw::kinematics::State _state;

        QBase _qmin;
        QBase _qmax;

        QBase _vmin;
        QBase _vmax;

        QBase _amin;
        QBase _amax;

        //Stuff for stats
        char* _lowerLimitType;
        char* _upperLimitType;
        char* _statusArray;
        int* _accLimit;
        int* _velLimit;
        int* _posLimit;
    };

    /**@}*/

}} // end namespaces

#endif // end include guard
