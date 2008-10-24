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

#ifndef RW_MODELS_PARALLELDEVICE_HPP
#define RW_MODELS_PARALLELDEVICE_HPP

/**
 * @file ParallelDevice.hpp
 */

#include "JointDevice.hpp"

#include <vector>
#include <string>

namespace rw { namespace math { class Jacobian; }}
namespace rw { namespace kinematics { class State; class Frame; }}

namespace rw { namespace models {

    class Joint;
    class ParallelLeg;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief This class defines the interface for Parallel devices.
     */
    class ParallelDevice : public JointDevice
    {
    public:
        /**
         * @brief Constructor
         *
         * @param legs [in] the serial legs connecting the endplate to the base.
         * The base of each serial Leg must be the same frame. Likewise, the endeffector
         * (last frame) of each Leg must transform to the same transform as each of the
         * other legs
         * @param name [in] name of device
         * @param state [in] the state for the assembly mode
         */
        ParallelDevice(
            const std::vector<ParallelLeg*>& legs,
            const std::string name,
            const kinematics::State& state);

        /** @brief Destructor */
        ~ParallelDevice();

        /**
         * @copydoc Device::setQ
         *
         * The configuration \b q is the configuration for the actuated joints
         * of the parallel device. Based on the value of \b q the setQ() method
         * automatically computes the values for the unactuated (passive)
         * joints.
         */
        virtual void setQ(const math::Q& q, kinematics::State& state) const;

        /** @copydoc Device::baseJframe */
        math::Jacobian baseJframe(
            const kinematics::Frame* frame,
            const kinematics::State& state) const;

        /** @copydoc Device::baseJend */
        math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @brief The legs of the parallel device.
         */
        virtual std::vector<ParallelLeg*> getLegs() const { return _legs;}

        /**
         * @brief The active joints of the parallel device.
         */
        virtual std::vector<models::Joint*> getActiveJoints()
        { return _actuatedJoints; }

    private:
        std::vector<ParallelLeg*> _legs;
        std::vector<kinematics::Frame*> _secondaryRef;

        std::vector<models::Joint*> _actuatedJoints; // list of actuated joints
        std::vector<models::Joint*> _unActuatedJoints; // list of unactuated joints

        boost::numeric::ublas::matrix<double>* _aJointJ; // the actuated joint jacobian
        boost::numeric::ublas::matrix<double>* _uaJointJ; // the unactuated joint jacobian

        math::Q* _lastAJVal; // current actuated joint values
        math::Q* _lastUAJVal;// current unactuated joint values
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
