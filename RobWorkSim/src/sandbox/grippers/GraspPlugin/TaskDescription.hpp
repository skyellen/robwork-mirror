/**
 * @file TaskDescription.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>



/**
 * @class TaskDescription
 * 
 * @brief Contains description of a task.
 * 
 * Description includes:
 * - workcell & dynamic workcell
 * - which object is the target
 * - name of the gripper device
 * - ...
 */
class TaskDescription
{
	public:
	// constructors
		/// Constructor
		TaskDescription();
		
		/// Destructor
		virtual ~TaskDescription();
		
	// methods
		rw::models::WorkCell* getWorkCell() { return _wc; }
		void setWorkCell(rw::models::WorkCell* wc) { _wc = wc; }
		
		rwsim::dynamics::DynamicWorkCell::Ptr getDynamicWorkCell() { return _dwc; }
		void setDynamicWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc) { _dwc = dwc; }
		
		double getInterferenceLimit() const { return _interferenceLimit; }
		void setInterferenceLimit(double limit) { _interferenceLimit = limit; }
		
		double getWrenchLimit() const { return _wrenchLimit; }
		void setWrenchLimit(double limit) { _wrenchLimit = limit; }
		
		std::vector<rw::models::Object::Ptr> getInterferenceObjects() { return _interferenceObjects; }
		void addInterferenceObject(rw::models::Object::Ptr object) { _interferenceObjects.push_back(object); }
		
	// friends
		friend class TaskDescriptionLoader;
	
	protected:
	// data
		rw::models::WorkCell* _wc;
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::kinematics::State _initState;
		double _interferenceLimit;
		double _wrenchLimit;
		std::vector<rw::models::Object::Ptr> _interferenceObjects;
};



/**
 * @class TaskDescriptionLoader
 * @brief Loads & saves task description from/to XML file.
 */
class TaskDescriptionLoader
{
	public:
	
	protected:
};
