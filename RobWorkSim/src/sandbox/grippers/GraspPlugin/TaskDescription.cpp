#include "TaskDescription.hpp"

#include <boost/algorithm/string.hpp>
#include "XMLHelpers.hpp"

#define DEBUG rw::common::Log::debugLog()



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::dynamics;
using namespace boost::numeric;
using namespace boost::property_tree;
using boost::algorithm::trim;
using namespace rwlibs::xml;



TaskDescription::TaskDescription(rwsim::dynamics::DynamicWorkCell::Ptr dwc) :
	_isOk(false),
	_dwc(dwc),
	_wc(dwc->getWorkcell().get()),
	_initState(_wc->getDefaultState()),
	_interferenceLimit(0.0),
	_wrenchLimit(0.0),
	_targetObject(NULL)
{
	if (_wc == NULL || _dwc == NULL) {
		RW_THROW("NULL WC or DWC!");
	}
}



TaskDescription::~TaskDescription()
{
}



TaskDescription::Ptr TaskDescriptionLoader::readTaskDescription(PTree& tree, rwsim::dynamics::DynamicWorkCell::Ptr dwc)
{
	TaskDescription::Ptr task = ownedPtr(new TaskDescription(dwc));
	
	if (task->_wc == NULL) {
		RW_THROW(" NULL wc");
	}
	
	DEBUG << "Loading task description..." << endl;
	
	readTarget(tree.get_child("Target"), task);
	readGripper(tree.get_child("Gripper"), task);
	readInterferenceObjects(tree.get_child("InterferenceObjects"), task);
	readLimits(tree.get_child("Limits"), task);
	
	DEBUG << "- baseline" << endl;
	readQualities(tree.get_child("Baseline"), task->_baseLine);
	DEBUG << "- weights" << endl;
	readQualities(tree.get_child("Weights"), task->_weights);
	
	DEBUG << "- coverage distance: ";
	PTree& node = tree.get_child("CoverageDistance");
	task->_coverageDistance = XMLHelpers::readQ(node);
	task->_coverageDistance(6) *= Deg2Rad;
	DEBUG << task->_coverageDistance << endl;
	
	task->_isOk = true;
	
	return task;
}



void TaskDescriptionLoader::readTarget(PTree& tree, TaskDescription::Ptr task)
{
	string targetName = tree.get_value<string>();
	trim(targetName);
	
	DEBUG << "- target name: [" << targetName << "]" << endl;
	
	task->_targetObject = task->_wc->findObject(targetName);
	task->_targetFrame = task->_targetObject->getBase();
	
	if (!task->_targetObject) {
		RW_THROW("Cannot find target object!");
	}
}



void TaskDescriptionLoader::readGripper(PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "- gripper" << endl;
	
	// read name
	PTree& nameNode = tree.get_child("Name");
	string gripperName = nameNode.get_value<string>();
	trim(gripperName);
	DEBUG << "\t name: [" << gripperName << "]" << endl;
	task->_gripperID = gripperName;
	task->_gripperDevice = task->_wc->findDevice<TreeDevice>(gripperName);
	if (!task->_gripperDevice) {
		RW_THROW("Gripper device not found!");
	}
	task->_gripperDynamicDevice = task->_dwc->findDevice<RigidDevice>(gripperName);
	if (!task->_gripperDynamicDevice) {
		RW_THROW("Gripper dynamic device not found!");
	}
	
	// read tcp frame
	PTree& tcpNode = tree.get_child("TCP");
	string tcpName = tcpNode.get_value<string>();
	trim(tcpName);
	DEBUG << "\t tcp frame: [" << tcpName << "]" << endl;
	task->_gripperTCP = task->_wc->findFrame(tcpName);
	if (!task->_gripperTCP) {
		RW_THROW("Gripper TCP frame not found!");
	}
	
	// read movable frame
	PTree& movNode = tree.get_child("Movable");
	string movableName = movNode.get_value<string>();
	trim(movableName);
	DEBUG << "\t movable frame: [" << movableName << "]" << endl;
	task->_gripperMovable = task->_wc->findFrame<MovableFrame>(movableName);
	if (!task->_gripperMovable) {
		RW_THROW("Gripper movable frame not found!");
	}
		
	// read controller
	PTree& ctrlrNode = tree.get_child("Controller");
	string ctrlrName = ctrlrNode.get_value<string>();
	trim(ctrlrName);
	DEBUG << "\t controller: [" << ctrlrName << "]" << endl;
	task->_controllerID = ctrlrName;
}



void TaskDescriptionLoader::readInterferenceObjects(PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "- interference objects" << endl;
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Object") {
			string objName = p->second.get_value<string>();
			trim(objName);
			
			DEBUG << "\t object: [" << objName << "]" << endl;
			
			Object::Ptr obj = task->_wc->findObject(objName);
			if (obj == NULL) {
				RW_THROW("Interference object not found!");
			}
			
			task->addInterferenceObject(obj);
		}
	}
}



void TaskDescriptionLoader::readLimits(PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "- limits" << endl;
	
	DEBUG << "\tInterference limit: ";
	PTree& intChild = tree.get_child("Interference");
	task->_interferenceLimit = XMLHelpers::readDouble(intChild);
	DEBUG << task->_interferenceLimit << endl;
	
	DEBUG << "\tWrench limit: ";
	PTree& wrenchChild = tree.get_child("Wrench");
	task->_wrenchLimit = XMLHelpers::readDouble(wrenchChild);
	DEBUG << task->_wrenchLimit << endl;
}



void TaskDescriptionLoader::readQualities(PTree& tree, TaskDescription::Qualities& q)
{
	DEBUG << "\tShape: ";
	PTree& node1 = tree.get_child("Shape");
	q.shape = XMLHelpers::readDouble(node1);
	DEBUG << q.shape << endl;
	
	DEBUG << "\tCoverage: ";
	PTree& node2 = tree.get_child("Coverage");
	q.coverage = XMLHelpers::readDouble(node2);
	DEBUG << q.coverage << endl;
	
	DEBUG << "\tSuccess ratio: ";
	PTree& node3 = tree.get_child("SuccessRatio");
	q.success = XMLHelpers::readDouble(node3);
	DEBUG << q.success << endl;
	
	DEBUG << "\tWrench: ";
	PTree& node4 = tree.get_child("Wrench");
	q.wrench = XMLHelpers::readDouble(node4);
	DEBUG << q.wrench << endl;
}



TaskDescription::Ptr TaskDescriptionLoader::load(const std::string& filename, rwsim::dynamics::DynamicWorkCell::Ptr dwc)
{
	TaskDescription::Ptr task;
	
    try {
        PTree tree;
        read_xml(filename, tree);

        task = readTaskDescription(tree.get_child("TaskDescription"), dwc);      
    } catch (const ptree_error& e) {
        RW_THROW(e.what());
    }
    
	return task;
}



void TaskDescriptionLoader::save(const std::string& filename)
{
	RW_WARN("Task description saving NOT IMPLEMENTED");
}
