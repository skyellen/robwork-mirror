#ifndef RW_MODELS_MOBILEDEVICE_HPP
#define RW_MODELS_MOBILEDEVICE_HPP

#include "DeviceModel.hpp"

#include "BasicDevice.hpp"
#include "RevoluteJoint.hpp"

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/math/Q.hpp>

namespace rw {
namespace models {
    /** @addtogroup models */
    /*@{*/

    /**
     * @brief Provides a differential controlled mobile device
     * 
     * The MobileDevice class provides a differential controlled mobile device with non-holonomic constraints.
     * The \f$x\f$ direction is defined as straight forward and \f$z\f$ vertically up. The wheels are assumed 
     * to be positioned summetrically around the base and have \f$0\f$ \f$x\f$ component.
     * 
     * When using setQ it takes 2 parameters, which corresponds to the distances travelled by the wheels.
     * Based on this input and the current pose of the device it calcualtes a new pose as.
     */
    class MobileDevice : public rw::models::DeviceModel
    {
    public:
        /**
         * @brief Constructs a mobile device
         * @param base [in] the base of the device
         * @param wheel1 [in] the left wheel
         * @param wheel2 [in] the right wheel
         * @param state [in] the state of the device
         * @param name [in] name of device 
         */
        MobileDevice(rw::kinematics::MovableFrame* base,
                     RevoluteJoint* wheel1,
                     RevoluteJoint* wheel2,
                     rw::kinematics::State& state,
                     const std::string& name);
        
        /**
         * @brief Destructor
         */
        virtual ~MobileDevice();
        
        
        /**
         * @brief Sets the position and orientation of the base
         * 
         * This operation moves the base of the robot, without considering 
         * the non-holonomic constraints of the device
         * @param transform [in] new base transform
         * @param state [in] state to write change to
         */
        void setDevicePose(const rw::math::Transform3D<>& transform, kinematics::State& state);
        
        /**
         * @copydoc DeviceModel::setQ
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
         * @copydoc DeviceModel::setBounds
         */
        virtual void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @copydoc DeviceModel::getVelocityLimits
         */
        virtual math::Q getVelocityLimits() const;

        /**
         * @copydoc DeviceModel::setVelocityLimits
         */
        virtual void setVelocityLimits(const math::Q& vellimits);

        /**
         * @copydoc DeviceModel::getAccelerationLimits
         */
        virtual math::Q getAccelerationLimits() const;

        /**
         * @copydoc DeviceModel::setAccelerationLimits
         */
        virtual void setAccelerationLimits(const math::Q& acclimits);

        /**
         * @copydoc DeviceModel::getDOF
         */
        virtual size_t getDOF() const;

        /**
         * @copydoc DeviceModel::getBase()
         */
        virtual kinematics::Frame* getBase();

        /**
         * @copydoc DeviceModel::getBase() const
         */
        virtual const kinematics::Frame* getBase() const;

        /**
         * @copydoc DeviceModel::getEnd()
         */
        virtual kinematics::Frame* getEnd();

        /**
         * @copydoc DeviceModel::getEnd() const
         */
        virtual const kinematics::Frame* getEnd() const;

        /**
         * @copydoc DeviceModel::baseJend
         */
        virtual math::Jacobian baseJend(const kinematics::State& state) const;
        
        /**
         * @copydoc DeviceModel::baseJframe
         */
        virtual math::Jacobian baseJframe(const kinematics::Frame* frame,
                                          const kinematics::State& state) const;

    private:
        rw::kinematics::MovableFrame* _base;
        std::vector<rw::kinematics::Frame*> _axillaryFrames;
        RevoluteJoint* _wheel1;
        RevoluteJoint* _wheel2;
        double _width;
        
        std::pair<rw::math::Q, rw::math::Q> _posLimits;
        rw::math::Q _velLimits;
        rw::math::Q _accLimits;
        
        rw::models::BasicDevice _basicDevice;
        
    };
    
    /*@}*/
    
} //end namespace models
} //end namespace rw

#endif //#ifndef RW_MODELS_MOBILEDEVICE_HPP
