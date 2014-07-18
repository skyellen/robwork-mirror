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
#include "XMLHelpers.hpp"
#include "Alignment.hpp"



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
		
		/// Used for storing baseline and weights for quality measurements.
		typedef struct { double coverage, success, wrench, topwrench, robustness, stress, volume; } Qualities;
		
	// constructors
		/// Constructor
		TaskDescription(rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/// Destructor
		virtual ~TaskDescription();
		
	// methods
		/// Is task description complete and consistent.
		bool isOk() { return _isOk; }
		
		rw::models::WorkCell* getWorkCell() { return _wc; }
		//void setWorkCell(rw::models::WorkCell* wc) { _wc = wc; _initState = wc->getDefaultState(); }
		
		rwsim::dynamics::DynamicWorkCell::Ptr getDynamicWorkCell() { return _dwc; }
		//void setDynamicWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc) { _dwc = dwc; setWorkCell(dwc->getWorkcell().get()); }
		
		rw::kinematics::State& getInitState() { return _initState; }
		
		double getInterferenceLimit() const { return _interferenceLimit; }
		void setInterferenceLimit(double limit) { _interferenceLimit = limit; }
		
		double getWrenchLimit() const { return _wrenchLimit; }
		void setWrenchLimit(double limit) { _wrenchLimit = limit; }
		
		double getStressLimit() const { return _stressLimit; }
		void setStressLimit(double limit) { _stressLimit = limit; }
		
		std::vector<rw::models::Object::Ptr> getInterferenceObjects() { return _interferenceObjects; }
		void addInterferenceObject(rw::models::Object::Ptr object) { _interferenceObjects.push_back(object); }
		
		bool hasHints() const { return _hints.size() > 0; }
		std::vector<rw::math::Transform3D<> >& getHints() { return _hints; }
		void addHint(rw::math::Transform3D<> hint) { _hints.push_back(hint); }
		
		bool hasAlignments() const { return _alignments.size() > 0; }
		std::vector<Alignment>& getAlignments() { return _alignments; }
		void addAlignment(const Alignment& alignment) { _alignments.push_back(alignment); }
		
		void setPrefilteringDistance(rw::math::Q dist) { _prefilteringDistance = dist; }
		rw::math::Q getPrefilteringDistance() const { return _prefilteringDistance; }
		
		void setCoverageDistance(rw::math::Q dist) { _coverageDistance = dist; }
		rw::math::Q getCoverageDistance() const { return _coverageDistance; }
		
		void setTeachDistance(rw::math::Q dist) { _teachDistance = dist; }
		rw::math::Q getTeachDistance() const { return _teachDistance; }
		
		rw::models::Object::Ptr getTargetObject() { return _targetObject; }
		rw::kinematics::Frame* getTargetFrame() { return _targetFrame; }
		void setTarget(rw::models::Object::Ptr object);
		void setTarget(std::string objectName) { setTarget(_wc->findObject(objectName)); }
		
		const std::string& getGripperID() { return _gripperID; }
		
		rw::models::TreeDevice::Ptr getGripperDevice() { return _gripperDevice; }
		rwsim::dynamics::RigidDevice::Ptr getGripperDynamicDevice() { return _gripperDynamicDevice; }
		rw::kinematics::Frame* getGripperTCP() { return _gripperTCP; }
		rw::kinematics::MovableFrame* getGripperMovable() { return _gripperMovable; }
		const std::string& getControllerID() { return _controllerID; }
		
		Qualities& getBaseline() { return _baseLine; }
		Qualities& getWeights() { return _weights; }
		
	// friends
		friend class TaskDescriptionLoader;
	
	protected:
	// data
		bool _isOk; // is task description consistent
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::models::WorkCell* _wc;
		rw::kinematics::State _initState;
		double _interferenceLimit;
		double _wrenchLimit;
		double _stressLimit;
		std::vector<rw::models::Object::Ptr> _interferenceObjects;
		rw::math::Q _prefilteringDistance; // distance used to filter samples at the start, so they are more uniform
		rw::math::Q _coverageDistance; // distance used for filtering grasps for coverage
		rw::math::Q _teachDistance;
		rw::models::Object::Ptr _targetObject;
		rw::kinematics::Frame* _targetFrame;
		std::string _gripperID;
		rw::models::TreeDevice::Ptr _gripperDevice;
		rw::kinematics::Frame* _gripperTCP;
		rw::kinematics::MovableFrame* _gripperMovable;
		std::string _controllerID;
		rwsim::dynamics::RigidDevice::Ptr _gripperDynamicDevice;
		Qualities _baseLine;
		Qualities _weights;
		std::vector<rw::math::Transform3D<> > _hints;
		std::vector<Alignment> _alignments;
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
		static TaskDescription::Ptr load(const std::string& filename, rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		
		/// Save task description to XML file.
		static void save(const TaskDescription::Ptr td, const std::string& filename);
	
	protected:
	// methods
		static TaskDescription::Ptr readTaskDescription(rwlibs::xml::PTree& tree, rwsim::dynamics::DynamicWorkCell::Ptr dwc);
		static void readTarget(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readGripper(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readInterferenceObjects(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readLimits(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readQualities(rwlibs::xml::PTree& tree, TaskDescription::Qualities& q);
		static void readHints(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readGrasp(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readAlignments(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
		static void readAlignment(rwlibs::xml::PTree& tree, TaskDescription::Ptr task);
};
