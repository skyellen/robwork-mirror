#ifndef RW_MODELS_CONVEYOR_HPP
#define RW_MODELS_CONVEYOR_HPP

#include "FixedJoint.hpp"
#include "Device.hpp"
#include "BasicDevice.hpp"
#include "ConveyorSegment.hpp"

#include <rw/kinematics/State.hpp>

namespace rw { namespace models {
	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief Provides a Conveyor models as a 1-DOF device
	 *
	 * A Conveyor consist of 1 or more ConveyorSegments joined to form the
	 * desired conveyor system. It is the responsibility of the user that
	 * ConveyorSegments are placed such that the paths of objects on the
	 * conveyor are continuous.
	 */
	class Conveyor: public Device
	{
	public:
		/**
		 * @brief constructs a conveyor consisting of a list of connected conveyor
		 * segments
		 * @param segments [in] vector of ConveyorSegments
		 */
		Conveyor(
            const std::string& name,
            FixedJoint* base,
            const std::vector<ConveyorSegment*>& segments);

	    /**
	     * @brief adds a dynamic attachable frame (DAF) to this conveyor. The time
	     * indicates the distance from the start of the conveyor to where the frame
	     * is attached.
	     * @param frame [in] pointer to DAF
	     * @param time [in] the distance from start of conveyor to DAF position
	     */
	    void addItem(ConveyorItem* item, double time, rw::kinematics::State& state);

        /**
		 * @copydoc Device::setQ
         */
        virtual void setQ(const math::Q& q, kinematics::State& state) const;

        /**
         * @copydoc Device::getQ
         */
        virtual math::Q getQ(const kinematics::State& state) const;

        /**
         * @copydoc Device::getBounds
         */
        virtual std::pair<math::Q, math::Q> getBounds() const;

        /**
         * @copydoc Device::setBounds
         */
        virtual void setBounds(const std::pair<math::Q, math::Q>& bounds);

        /**
         * @copydoc Device::getVelocityLimits
         */
        virtual math::Q getVelocityLimits() const;

        /**
         * @copydoc Device::setVelocityLimits
         */
        virtual void setVelocityLimits(const math::Q& vellimits);

        /**
         * @copydoc Device::getAccelerationLimits
         */
        virtual math::Q getAccelerationLimits() const;

        /**
         * @copydoc Device::setAccelerationLimits
         */
        virtual void setAccelerationLimits(const math::Q& acclimits);

        /**
         * @copydoc Device::getDOF
         */
        virtual size_t getDOF() const;

        /**
         * @copydoc Device::getBase
         */
        virtual kinematics::Frame* getBase();

        /**
         * @copydoc Device::getBase
         */
        virtual const kinematics::Frame* getBase() const;

        /**
         * @copydoc Device::getEnd
         */
        virtual kinematics::Frame* getEnd();

        /**
         * @copydoc Device::getEnd
         */
        virtual const kinematics::Frame* getEnd() const;

        /**
         * @copydoc Device::baseJend
         */
        virtual math::Jacobian baseJend(const kinematics::State& state) const;

        /**
         * @copydoc Device::baseJframe
         */
        virtual math::Jacobian baseJframe(
            const kinematics::Frame* frame, const kinematics::State& state) const;

	private:
	    std::vector<ConveyorSegment*> _segments;
	    FixedJoint* _base;
	    BasicDevice _basicDevice;

	    std::map<rw::kinematics::Frame*, ConveyorSegment*> _frame2segment;
	};

	/* @} */
}} // end namespaces

#endif // end include guard
