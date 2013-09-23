/**
 * @file TaskDescription.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <vector>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>



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
	// types
		/// Smart pointer to this type of class.
		typedef rw::common::Ptr<TaskDescription> Ptr;
		
	// constructors
		/// Constructor
		TaskDescription(rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/// Destructor
		virtual ~TaskDescription();
		
	// methods
		rw::models::WorkCell* getWorkCell() { return _wc; }
		void setWorkCell(rw::models::WorkCell* wc) { _wc = wc; _initState = wc->getDefaultState(); }
		
		rwsim::dynamics::DynamicWorkCell::Ptr getDynamicWorkCell() { return _dwc; }
		void setDynamicWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc) { _dwc = dwc; setWorkCell(dwc->getWorkcell().get()); }
		
		rw::kinematics::State& getInitState() { return _initState; }
		
		double getInterferenceLimit() const { return _interferenceLimit; }
		void setInterferenceLimit(double limit) { _interferenceLimit = limit; }
		
		double getWrenchLimit() const { return _wrenchLimit; }
		void setWrenchLimit(double limit) { _wrenchLimit = limit; }
		
		std::vector<rw::models::Object::Ptr> getInterferenceObjects() { return _interferenceObjects; }
		void addInterferenceObject(rw::models::Object::Ptr object) { _interferenceObjects.push_back(object); }
		
		rw::models::Object::Ptr getTargetObject() { return _targetObject; }
		void setTargetObject(rw::models::Object::Ptr object) { _targetObject = object; }
		
		const std::string& getGripperID() { return _gripperID; }
		
		rw::models::Device::Ptr getGripperDevice() { return _gripperDevice; }
		
		rw::kinematics::Frame* getGripperTCP() { return _gripperTCP; }
		
		rw::kinematics::MovableFrame* getGripperMovable() { return _gripperMovable; }
		
		const std::string& getControllerID() { return _controllerID; }
		
	// friends
		friend class TaskDescriptionLoader;
	
	protected:
	// data
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::models::WorkCell* _wc;
		rw::kinematics::State _initState;
		double _interferenceLimit;
		double _wrenchLimit;
		std::vector<rw::models::Object::Ptr> _interferenceObjects;
		rw::models::Object::Ptr _targetObject;
		std::string _gripperID;
		rw::models::Device::Ptr _gripperDevice;
		rw::kinematics::Frame* _gripperTCP;
		rw::kinematics::MovableFrame* _gripperMovable;
		std::string _controllerID;
};



/**
 * @class TaskDescriptionLoader
 * @brief Loads & saves task description from/to XML file.
 */
class TaskDescriptionLoader
{
	public:
	// static
		/// Load task description from XML file.
		static TaskDescription::Ptr load( const std::string& filename, rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/// Save task description to XML file.
		static void save(const std::string& filename);
	
	protected:
	// typedefs
		typedef boost::property_tree::ptree PTree;
		typedef PTree::iterator CI;
	
	// methods
		static TaskDescription::Ptr readTaskDescription(PTree& tree, rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		static void readTarget(PTree& tree, TaskDescription::Ptr task);
		static void readGripper(PTree& tree, TaskDescription::Ptr task);
		static void readInterferenceObjects(PTree& tree, TaskDescription::Ptr task);
		static void readLimits(PTree& tree, TaskDescription::Ptr task);
};
