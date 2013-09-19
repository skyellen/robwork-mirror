/**
 * @file GripperTaskSimulator.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <rw/models/WorkCell.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>



namespace rwsim {
	namespace simulator {
		
		/**
		 * @brief Used to simulate tasks for a specific gripper and evaluate gripper
		 */
		class GripperTaskSimulator : public GraspTaskSimulator
		{
			public:
				//! smart pointer type
				typedef rw::common::Ptr<GripperTaskSimulator> Ptr;

			public:
				//! @brief Constructor
				GripperTaskSimulator(rwsim::dynamics::DynamicWorkCell::Ptr dwc);
				
			public:
				//! @brief Set objects for interference measurement
				void setInterferenceObjects(const std::vector<rw::models::Object::Ptr>& objects) { _objects = objects; }
				
				//! @brief Add object for interference measurement
				void addInterferenceObject(rw::models::Object::Ptr object) { _objects.push_back(object); }
				
				//! @brief Set interference limit
				void setInterferenceLimit(double limit) { _interferenceLimit = limit; }
				
				//! @brief Set wrench limit
				void setWrenchLimit(double limit) { _wrenchLimit = limit; }
				
			protected:
				/**
				 * @brief Callback for when the grasp is finished
				 */
				virtual void graspFinishedCB(SimState& sstate);
				
			private:
				//! @brief Measure interference
				double measureInterference(SimState& sstate, const rw::kinematics::State& state);
				
				rw::models::WorkCell::Ptr _wc;
				rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
				std::vector<rw::models::Object::Ptr> _objects;
				
				double _interferenceLimit;
				double _wrenchLimit;
		};
}} // end namespaces
