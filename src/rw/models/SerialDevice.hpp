/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rw_models_SerialDevice_HPP
#define rw_models_SerialDevice_HPP

/**
 * @file SerialDevice.hpp
 */

#include "DeviceModel.hpp"
#include "BasicDevice.hpp"
#include "BasicDeviceJacobian.hpp"

#include <rw/math/Transform3D.hpp>

#include <vector>
#include <map>
#include <memory>

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Serial Chain device
     *
     * @dot
     * digraph serialdevice {
     *  node [shape=record, fontname=Helvetica, fontsize=10, style=filled];
     *  Base [ fillcolor="red"];
     *  Link1 [ label="Link1\n<Link>", fillcolor="red"];
     *  Axis1 [ label="Axis1\n<Joint>", fillcolor="red"];
     *  Link2 [ label="Link2\n<Link>",fillcolor="red"];
     *  Axis2 [ label="Axis2\n<Joint>",fillcolor="red"];
     *  Link3 [ label="Link3\n<Link>",fillcolor="red"];
     *  Axis3 [ label="Axis1\n<Joint>",fillcolor="red"];
     *  EndEffector [ fillcolor="red"];
     *
     * world -> object1;
     * world -> table;
     * table -> object2;
     * world -> Base;
     * Base -> Link1;
     * Link1 -> Axis1;
     * Axis1 -> Link2;
     * Link2 -> Axis2;
     * Axis2 -> Link3;
     * Link3 -> Axis3;
     * Axis3 -> EndEffector;
     * }
     * @enddot
     *
     * Example of usage:
     * @code
     * SerialDevice serialDevice(base, endEffector, activeJoints);
     * std::cout << "Jacobian: " << serialDevice.bJe() << std::endl;
     * @endcode
     *
     * @todo document this
     */
    class SerialDevice : public DeviceModel
    {
    public:
        /**
         * @brief Creates object
         *
         * @param first [in] the base frame of the robot
         * @param last [in] the default endeffector of the robot
         * @param name [in] name of device
         * @param state [in] the initial state of everything
         *
         * @pre links[0] must be located below first in the chain
         *
         * @pre links[links.size()-1] mst be the last in the chain. This will
         * represent the default endeffector.
         *
         * @pre joints must be a subset of the kinematic chain between the first
         * and last link
         *
         */
        SerialDevice(kinematics::Frame* first,
                     kinematics::Frame* last,
                     const std::string& name,
                     const kinematics::State& state);

        /**
         * @brief Creates object
         *
         * @param serialChain [in] a vector of connected frames. The base of
         * the serialDevice becomes the first frame in serialChain, the endeffector
         * becomes the last frame in the vector. All frames in the chain between 
         * base and end does not need to be located in the serialChain. If they are 
         * not, these will not be considered as joints for this device. 
         *
         * @param name [in] name of device
         *
         * @param state [in] the initial state of everything
         *
         */
        SerialDevice(const std::vector<kinematics::Frame*>& serialChain,
                     const std::string& name,
                     const kinematics::State& state);

        /**
         * @brief destructor
         */
        virtual ~SerialDevice();

        /**
         * @copydoc DeviceModel::setQ
         *
         * @pre q.size() == activeJoints.size()
         */
        void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getQ
         */
        math::Q getQ(const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getDOF
         */
        size_t getDOF() const{
            return _activeJoints.size();
        }

        /**
         * @copydoc DeviceModel::getBounds
         */
        std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @copydoc DeviceModel::setBounds
         */
        void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @copydoc DeviceModel::getVelocityLimits
         */
        math::Q getVelocityLimits() const;

        /**
         * @copydoc DeviceModel::setVelocityLimits
         */
        void setVelocityLimits(const math::Q& vellimits);

        /**
         * @brief DeviceModel::getAccelerationLimits
         */
        math::Q getAccelerationLimits() const;

        /**
         * @brief DeviceModel::setAccelerationLimits
         */
        void setAccelerationLimits(const math::Q& acclimits);

        /**
         * @brief Returns pointer to active joint
         * @param index [in] joint index
         * @return a joint
         */
        Joint* getActiveJoint(size_t index) const
        {
            assert(index < _activeJoints.size());
            return _activeJoints[index];
        }

        /**
         * @brief Returns reference to kinematic chain
         *
         * @return a reference to the list of frames that defines the kinematic
         * chain of the serial robot
         */
        const std::vector<kinematics::Frame*>& frames() const{
            return _kinematicChain;
        }

        /**
         * @copydoc DeviceModel::getBase
         */
        kinematics::Frame* getBase(){
            return _base;
        };

        /**
         * @copydoc DeviceModel::getBase
         */
        const kinematics::Frame* getBase() const {
            return _base;
        };

        /**
         * @copydoc DeviceModel::getEnd()
         */
        virtual kinematics::Frame* getEnd() {
            return _end;
        };

        /**
         * @copydoc DeviceModel::getEnd() const
         */
        virtual const kinematics::Frame* getEnd() const {
            return _end;
        };

        /**
         * @copydoc DeviceModel::baseJend
         */
        math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::baseJframe
         */
        math::Jacobian baseJframe(
            const kinematics::Frame *frame,
            const kinematics::State& state) const;

        /**
         * @brief like baseJframe but with a jacobian calculated for a
         * list of end frames.
         * @return a BasicDeviceJacobian that can calculate the jacobian
         * using get(..) method
         */
        boost::shared_ptr<BasicDeviceJacobian> baseJframes(
            const std::vector<kinematics::Frame*>& frames,
            const kinematics::State& state) const;        
        
    private:
        kinematics::Frame* _base;

        kinematics::Frame* _end;

        std::vector<kinematics::Frame*> _kinematicChain;

        std::vector<Joint*> _activeJoints;

        // getQ(), getBounds(), etc. are forwarded to this object.
        BasicDevice _basicDevice;

        // Base to end Jacobians are computed here.
        BasicDeviceJacobian _dj;
    };

    /*@}*/
}} // end namespaces

/*
 * @example workcell/PUMA560.cpp
 *
 * This is an example of how to use the SerialDevice class to model a robot
 *
 * The example models an Unimation PUMA 560 robot and calculates its forward
 * kinematics at 2 different configurations
 */

#endif // end include guard
