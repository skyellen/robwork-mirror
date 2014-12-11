/**
 * @file GripperTaskSimulator.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <rw/models/WorkCell.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include "TaskDescription.hpp"
#include "Gripper.hpp"



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
		/**
		 * @brief Constructor.
		 * 
		 * @param gripper [in] Gripper to perform tasks
		 * @param tasks [in] tasks to simulate
		 * @param samples [in] all samples created by task generator
		 * @param td [in] task description
		 */
		GripperTaskSimulator(rw::models::Gripper::Ptr gripper, rwlibs::task::GraspTask::Ptr tasks,
			rwlibs::task::GraspTask::Ptr samples, TaskDescription::Ptr td);
			
		/// Destructor.
		virtual ~GripperTaskSimulator() {}
		
	// methods
		/**
		 * @brief Load all samples created by grasp generator.
		 * 
		 * These samples are used for coverage calculation.
		 */
		void loadSamples(rwlibs::task::GraspTask::Ptr samples) { _samples = samples; }
		
		/**
		 * @brief Get the simulated gripper's quality
		 */
		rw::models::GripperQuality getGripperQuality() const { return _quality; }
		
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
		double calculateInterference(SimState& sstate, const rw::kinematics::State& state0);
		
		/**
		 * @brief Returns alignment index for all performed grasps
		 * 
		 * Performs a pose classification on tcpTobject poses after lifting objects.
		 * The model of a stable pose with the highest number of inliers is selected.
		 * For all the poses matching the model, an angle between pose before grasping, and a pose after grasping is calculated.
		 * The alignment index is a width of a histogram of pose angle changes. It is calculated as a standard variation of
		 * the angle values.
		 */
		 
		double calculateAlignment() const;
		
		/**
		 * @brief Get wrench measurement for performed grasp.
		 * 
		 * This is the min. wrench of the grasp.
		 */
		double calculateWrench(SimState& sstate) const;
		
		/**
		 * @brief Returns coverage measurement.
		 * 
		 * Coverage is calculated as a ratio of filtered succesful grasps to
		 * number of samples created by the generator.
		 * 
		 * @param actualRatio [in] ratio of grasps that didn't end up with simulation failure)
		 */
		double calculateCoverage(double actualRatio=1.0);
		
		/**
		 * @brief Calculates min., avg. and max. wrenches in all tested grasps.
		 */
		rw::math::Q calculateWrenchMeasurement() const;
		
		/**
		 * @brief Calculates gripper quality.
		 */
		double calculateQuality();
		
		/**
		 * @brief Calculates gripper shape quality.
		 * @todo Implement!
		 */
		double calculateShape();
		
		/**
		 * @brief Evaluates gripper.
		 * 
		 * Sets up GripperQuality structure in Gripper.
		 */
		void evaluateGripper();

	// data
		rw::models::Gripper::Ptr _gripper;
		TaskDescription::Ptr _td;
		rwlibs::task::GraspTask::Ptr _samples;
		
		rw::models::GripperQuality _quality; // quality of the simulated gripper
};
}} // end namespaces
