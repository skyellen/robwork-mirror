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

#ifndef rw_models_CompositeDevice_HPP
#define rw_models_CompositeDevice_HPP

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
     */
    class CompositeDevice : public DeviceModel
    {
    public:
        /**
         * @brief Creates object
         */
        CompositeDevice(rw::kinematics::Frame *base,
                        std::vector<DeviceModel*> models,
                        rw::kinematics::Frame *end,
                        const std::string& name,
                        const kinematics::State& state);

        /**
         * @brief destructor
         */
        virtual ~CompositeDevice();

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
            //std::cout << "getDOF" << std::endl;

            size_t dof = 0;
            for(size_t i=0; i<_devices.size(); i++){
                dof += _devices[i]->getDOF();
            }
            return dof;
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
        std::vector<DeviceModel*> _devices;
        
        kinematics::Frame* _base;

        kinematics::Frame* _end;

        //std::vector<Joint*> _activeJoints;

        // getQ(), getBounds(), etc. are forwarded to this object.
        //BasicDevice _basicDevice;

        // Base to end Jacobians are computed here.
        //BasicDeviceJacobian _dj;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
