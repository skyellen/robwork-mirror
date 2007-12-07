#ifndef RW_MODELS_CONVEYOR_HPP
#define RW_MODELS_CONVEYOR_HPP


#include "FixedJoint.hpp"
#include "DeviceModel.hpp"
#include "BasicDevice.hpp"
#include "ConveyorSegment.hpp"
#include "FixedJoint.hpp"

#include <rw/kinematics/State.hpp>

namespace rw {
namespace models {
	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief Provides a Conveyor models as a 1-DOF device
	 * 
	 * A Conveyor consist of 1 or more ConveyorSegments joined to form the desired 
	 * conveyor system. It is the responsibility of the user that ConveyorSegments are placed 
	 * such that the paths of objects on the conveyor are continuous.
	 *
	 */
	class Conveyor: public DeviceModel
	{
	public:
		/**
		 * @brief constructs a conveyor consisting of a list of connected conveyor
		 * segments
		 * @param segments [in] vector of ConveyorSegments
		 */
		Conveyor(const std::string& name, 
		         FixedJoint* base, 
		         const std::vector<ConveyorSegment*>& segments);
		
		/**
		 * @brief Default destructor
		 */
		virtual ~Conveyor();
	
	    /**
	     * @brief adds a dynamic attachable frame (DAF) to this conveyor. The time
	     * indicates the distance from the start of the conveyor to where the frame 
	     * is attached.
	     * @param frame [in] pointer to DAF
	     * @param time [in] the distance from start of conveyor to DAF position
	     */
	    void addItem(ConveyorItem* item, double time, rw::kinematics::State& state);
	
	    
	    
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
         * @copydoc DeviceModel::getBase
         */
        virtual kinematics::Frame* getBase();

        /**
         * @copydoc DeviceModel::getBase
         */
        virtual const kinematics::Frame* getBase() const;

        /**
         * @copydoc DeviceModel::getEnd
         */
        virtual kinematics::Frame* getEnd();

        /**
         * @copydoc DeviceModel::getEnd
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
	    std::vector<ConveyorSegment*> _segments;	    
	    
	    std::map<rw::kinematics::Frame*, ConveyorSegment*> _frame2segment;
	    
	    BasicDevice _basicDevice;
	    FixedJoint* _base;
	};
	/* @} */
} //end namespace models
} //end namespace rw
	
#endif //#ifndef RW_MODELS_CONVEYOR_HPP
