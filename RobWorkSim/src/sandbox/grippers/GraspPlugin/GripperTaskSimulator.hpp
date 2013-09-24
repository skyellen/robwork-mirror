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
		/// @copydoc GraspTaskSimulator::graspFinished
		virtual void graspFinished(SimState& sstate);
		
		/// @copydoc GraspTaskSimulator::printGraspResult
		virtual void printGraspResult(SimState& sstate);
		
		/// @copydoc GraspTaskSimulator::simulationFinished
		virtual void simulationFinished(SimState& sstate);
		
	private:
	// methods
		/**
		 * @brief Returns interference measurement for given grasp.
		 * 
		 * This is a sum of individual object displacements compared to initial kinematic state.
		 * In addition to returning interference face value, function sets up vectors storing
		 * interference data in GraspResult. These are interferenceDistances, interferenceAngles
		 * and interferences vectors, storing respectively: linear and angular displacements, and
		 * the weighted metrics of these displacements for each individual object.
		 * 
		 * @param sstate [in] current simulation state
		 * @param state0 [in] initial kinematic state
		 */
		double getInterference(SimState& sstate, const rw::kinematics::State& state0);
		
		/**
		 * @brief Get wrench measurement for performed grasp.
		 * 
		 * This is the min. wrench of the grasp.
		 */
		double getWrench(SimState& sstate);

	// data
		TaskDescription::Ptr _td;
};
}} // end namespaces
