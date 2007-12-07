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

#ifndef rw_models_ParallelDevice_HPP
#define rw_models_ParallelDevice_HPP

/**
 * @file ParallelDevice.hpp
 */

#include "DeviceModel.hpp"
#include "ParallelLeg.hpp"
#include "BasicDevice.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>

#include <rw/models/SerialDevice.hpp>

#include <memory>
#include <vector>
#include <string>

namespace rw { namespace math {
    class Jacobian;
}} // end namespaces

namespace rw { namespace kinematics {
    class State;
    class Frame;
}} // end namespaces

namespace rw { namespace models {

    class Joint;

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief This class defines the interface for Parallel devices.
     */
    class ParallelDevice : public DeviceModel
    {
    public:
        /**
         * @brief default constructor
         *
         * @param legs [in] the serial legs connecting the endplate to the base.
         * The base of each serial Leg must be the same frame. Likewise, the endeffector
         * (last frame) of each Leg must transform to the same transform as each of the
         * other legs
         * @param name [in] name of device
         * @param state [in] the state for the assembly mode
         */
        ParallelDevice(
            std::vector< ParallelLeg* > legs,
            const std::string name,
            const kinematics::State& state);

        /**
         * @brief default deconstructor
         */
        virtual ~ParallelDevice();

        /**
         * @copydoc DeviceModel::setQ
         * @note When setting the configuration of the parallel robot, only the
         * actuated (active) joints are set. The setQinState automaticly calculates new
         * configurations for the unactuated (passive) joints.
         */
        virtual void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getQ
         */
        virtual math::Q getQ(const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::getBounds
         */
        virtual std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @copydoc DeviceModel::getDOF
         */
        virtual size_t getDOF() const{
            return _actuatedJoints.size();
        };

        /**
         * @copydoc DeviceModel::getBase
         */
        virtual kinematics::Frame* getBase(){
            return _legs.front()->getBase();
        };

        /**
         * @copydoc DeviceModel::getBase
         */
        virtual const kinematics::Frame* getBase() const{
            return _legs.front()->getBase();
        };


        /**
         * @copydoc DeviceModel::getEnd()
         */
        virtual kinematics::Frame* getEnd() {
            return _legs.front()->getEnd();
        };

        /**
         * @copydoc DeviceModel::getEnd() const
         */
        virtual const kinematics::Frame* getEnd() const {
            return _legs.front()->getEnd();
        };

        /**
         * @copydoc DeviceModel::baseJframe
         */
        virtual math::Jacobian baseJframe(const kinematics::Frame* frame,
                                    const kinematics::State& state) const;

        /**
         * @copydoc DeviceModel::baseJend
         */
        virtual math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @brief Returns the legs of the ParallelDevice
         * @return list with the ParallelLeg's
         */
        virtual std::vector< ParallelLeg* > getLegs() const { return _legs;};

        /**
         * @copydoc DeviceModel::setBounds()
         */
        void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @brief Returns list with the active joint for the ParallelDevice
         * @return list of active Joints
         */
        virtual std::vector< models::Joint* > getActiveJoints(){ return _actuatedJoints;};

        /**
         * @brief Returns the velocity limits
         * @return velocity limits
         */
        math::Q getVelocityLimits() const;

        /**
         * @brief Sets the velocity limits
         * @param vellimits [in] the new velocity limits
         */
        void setVelocityLimits(const math::Q& vellimits);

        /**
         * @brief Returns the acceleration limits
         * @return acceleration limits
         */
        math::Q getAccelerationLimits() const;

        /**
         * @brief Sets the acceleration limits
         * @param acclimits [in] the new acceleration limits
         */
        void setAccelerationLimits(const math::Q& acclimits);

    private:
        std::vector< ParallelLeg* > _legs;
        std::vector<kinematics::Frame*> _secondaryRef;
        // kinematics::Frame *_baseFrame;
        // kinematics::Frame *_endplate;

        typedef std::pair<
            rw::math::Vector3D<double>,
            rw::math::Quaternion<double> > QPose;

        std::vector<models::Joint*> _actuatedJoints; // list of actuated joints
        std::vector<models::Joint*> _unActuatedJoints; // list of unactuated joints

        boost::numeric::ublas::matrix<double>* _aJointJ; // the actuated joint jacobian
        boost::numeric::ublas::matrix<double>* _uaJointJ; // the unactuated joint jacobian

        //std::vector<QPose >* _currY; // the vector containing the result
        math::Q* _lastAJVal; // current actuated joint values
        math::Q* _lastUAJVal;// current unactuated joint values

        // getQ(), getBounds(), etc. are forwarded to this object.
        std::auto_ptr<BasicDevice> _basicDevice;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
