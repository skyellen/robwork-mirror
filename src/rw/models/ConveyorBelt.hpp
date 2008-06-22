#ifndef RW_MODELS_CONVEYORBELT_HPP
#define RW_MODELS_CONVEYORBELT_HPP

#include "ConveyorSegment.hpp"
#include "ConveyorItem.hpp"

#include <rw/interpolator/Trajectory.hpp>

namespace rw { namespace models {

	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief A simple conveyor belt to be used with a Conveyor
	 *
	 * The ConveyerBelt provides a simple ConveyorSegment with 1 entry and 1 exit point. Associated
	 * with a belt is a trajectory, which is used to calculate the position of items.
	 */
	class ConveyorBelt: public ConveyorSegment
	{
	public:
		/**
		 * @brief Constructor
		 */
		ConveyorBelt(
            const rw::interpolator::Trajectory& trajectory,
            rw::kinematics::Frame* baseFrame,
            ConveyorSegment* next,
            ConveyorSegment* previous);

		/**
		 * @brief Destructor
		 */
		virtual ~ConveyorBelt();

		/**
		 * @copydoc ConveyorSegment::addItem
		 */
	    virtual void addItem(ConveyorItem* item, FramePosition position, rw::kinematics::State& state);

		/**
		 * @copydoc ConveyorSegment::move
		 */
	    virtual void move(ConveyorItem* item, double d, rw::kinematics::State& state);

		/**
		 * @copydoc ConveyorSegment::length
		 */
	    virtual double length() const;

		/**
		 * @copydoc ConveyorItem::getBaseFrame
		 */
	    virtual rw::kinematics::Frame* getBaseFrame();

		/**
		 * @brief Sets the previous segment
		 * @param segment [in] the segment to use as the previous
		 */
		virtual void setPreviousSegment(ConveyorSegment* segment);

		/**
		 * @brief Sets the next segment
		 * @param segment [in] the segment to use as the next
		 */
		virtual void setNextSegment(ConveyorSegment* segment);


	private:
	    /**
	     * Specifies
	     */
	    virtual void moveTo(ConveyorItem* item, double q, rw::kinematics::State& state);

	    rw::interpolator::Trajectory _trajectory;
	    rw::kinematics::Frame* _baseFrame;
        ConveyorSegment* _next;
	    ConveyorSegment* _previous;

	    std::map<rw::kinematics::Frame*, double> _frame2q;
	};
	/* @} */
}} // end namespaces

#endif //#ifndef RW_MODELS_CONVEYORBELT_HPP
