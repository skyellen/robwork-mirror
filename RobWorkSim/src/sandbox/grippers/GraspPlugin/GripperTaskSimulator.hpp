/**
 * @file GripperTaskSimulator.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <rw/models/WorkCell.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include "TaskDescription.hpp"



namespace rwsim {
	namespace simulator {
		
/**
 * @class GripperTaskSimulator
 * @brief Used to simulate tasks for a specific gripper and evaluate gripper.
 * 
 * Inherits from GraspTaskSimulator. Added behaviour is implemented by using callback after the grasp is finished.
 * After each grasp is completed, if it fails interference or wrench limit check, grasp status is changed from Success
 * to Interference or ObjectSlipped appriopriately.
 * Interference for a grasp is a sum of interferences of objects specified in TaskDescription. For each of the objects,
 * interference is a difference in position and angle from the initial pose is completed, and evaluated using weighted
 * metric.
 */
class GripperTaskSimulator : public GraspTaskSimulator
{
	public:
	// typedefs
		/// Smart pointer type.
		typedef rw::common::Ptr<GripperTaskSimulator> Ptr;

	// constructors
		/// Constructor.
		GripperTaskSimulator(TaskDescription::Ptr td) :
			GraspTaskSimulator(td->getDynamicWorkCell(), 1), _td(td) {}
			
		/// Destructor.
		virtual ~GripperTaskSimulator() {}
		
	protected:
	// methods
		/// Callback for when the grasp is finished.
		virtual void graspFinishedCB(SimState& sstate);
		
	private:
	// methods
		/// Measure interference.
		double measureInterference(SimState& sstate, const rw::kinematics::State& state);

	// data
		TaskDescription::Ptr _td;
};
}} // end namespaces
