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
	_stressLimit(10.0), // arbitrary
	_targetObject(NULL)
{
	if (_wc == NULL || _dwc == NULL) {
		RW_THROW("NULL WC or DWC!");
	}
}



TaskDescription::~TaskDescription()
{
}



void TaskDescription::setTarget(rw::models::Object::Ptr object)
{
	_targetObject = object;
	_targetFrame = _targetObject->getBase();
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
	
	DEBUG << "- prefiltering distance: ";
	PTree& node1 = tree.get_child("PrefilteringDistance");
	task->_prefilteringDistance = XMLHelpers::readQ(node1);
	task->_prefilteringDistance(1) *= Deg2Rad;
	task->_prefilteringDistance(2) *= Deg2Rad;
	DEBUG << task->_prefilteringDistance << endl;
	
	DEBUG << "- coverage distance: ";
	PTree& node2 = tree.get_child("CoverageDistance");
	task->_coverageDistance = XMLHelpers::readQ(node2);
	//task->_coverageDistance(6) *= Deg2Rad;
	task->_coverageDistance(1) *= Deg2Rad;
	task->_coverageDistance(2) *= Deg2Rad;
	DEBUG << task->_coverageDistance << endl;
	
	DEBUG << "- teach distance: ";
	PTree& node3 = tree.get_child("TeachDistance");
	task->_teachDistance = XMLHelpers::readQ(node3);
	task->_teachDistance(3) *= Deg2Rad;
	task->_teachDistance(4) *= Deg2Rad;
	DEBUG << task->_teachDistance << endl;
	
	boost::optional<PTree&> node4 = tree.get_child_optional("HintGrasps");
	if (node4) {
		readHints(*node4, task);
	}
	
	boost::optional<PTree&> node5 = tree.get_child_optional("Alignments");
	if (node5) {
		readAlignments(*node5, task);
	}
	
	task->_isOk = true;
	
	return task;
}



void TaskDescriptionLoader::readTarget(PTree& tree, TaskDescription::Ptr task)
{
	string targetName = tree.get_value<string>();
	trim(targetName);
	
	DEBUG << "- target name: [" << targetName << "]" << endl;
	
	task->_targetObject = task->_wc->findObject(targetName);
	//DEBUG << task->_targetObject->getName() << endl;
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
	
	boost::optional<PTree&> stressNode = tree.get_child_optional("Stress");
	if (stressNode) {
		DEBUG << "\tStress limit: ";
		task->_stressLimit = XMLHelpers::readDouble(stressNode.get());
		DEBUG << task->_stressLimit << endl;
	} else {
		task->_stressLimit = 0.0;
	}
}



void TaskDescriptionLoader::readQualities(PTree& tree, TaskDescription::Qualities& q)
{
	/*DEBUG << "\tShape: ";
	PTree& node1 = tree.get_child("Shape");
	q.shape = XMLHelpers::readDouble(node1);
	DEBUG << q.shape << endl;*/
	
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
	
	// read optional stress & volume weights
	boost::optional<PTree&> stressNode = tree.get_child_optional("Stress");
	if (stressNode) {
		DEBUG << "\tStress: ";
		q.stress = XMLHelpers::readDouble(stressNode.get());
		DEBUG << q.stress << endl;
	} else {
		q.stress = 1.0;
	}
	
	boost::optional<PTree&> volumeNode = tree.get_child_optional("Volume");
	if (volumeNode) {
		DEBUG << "\tVolume: ";
		q.volume = XMLHelpers::readDouble(volumeNode.get());
		DEBUG << q.volume << endl;
	} else {
		q.volume = 0.0;
	}
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



void TaskDescriptionLoader::readHints(rwlibs::xml::PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "- hints" << endl;
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Grasp") readGrasp(p->second, task);
	}
}



void TaskDescriptionLoader::readGrasp(rwlibs::xml::PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "\tGrasp: ";
	Q posq = XMLHelpers::readQ(tree.get_child("Pos"));
	Q rpyq = XMLHelpers::readQ(tree.get_child("RPY"));
	DEBUG << "pos=" << posq << " rot=" << rpyq << endl;
	
	Vector3D<> pos(posq[0], posq[1], posq[2]);
	RPY<> rpy(rpyq[0], rpyq[1], rpyq[2]);
	task->_hints.push_back(Transform3D<>(pos, rpy.toRotation3D()));
}



void TaskDescriptionLoader::readAlignments(rwlibs::xml::PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "- alignments" << endl;
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Alignment") readAlignment(p->second, task);
	}
}



void TaskDescriptionLoader::readAlignment(rwlibs::xml::PTree& tree, TaskDescription::Ptr task)
{
	DEBUG << "\tAlignment: ";
	Q posq = XMLHelpers::readQ(tree.get_child("Pos"));
	Q rpyq = XMLHelpers::readQ(tree.get_child("RPY"));
	Q distq = XMLHelpers::readQ(tree.get_child("Dist"));
	DEBUG << "pos=" << posq << " rot=" << rpyq << " dist=" << distq << endl;
	
	Vector3D<> pos(posq[0], posq[1], posq[2]);
	RPY<> rpy(rpyq[0], rpyq[1], rpyq[2]);
	task->_alignments.push_back(Alignment(Transform3D<>(pos, rpy.toRotation3D()), distq));
}



void TaskDescriptionLoader::save(const TaskDescription::Ptr td, const std::string& filename)
{
	PTree tree;
	
	
	// put target
	tree.put("TaskDescription.Target", td->_targetObject->getName());
	
	// save gripper information
	tree.put("TaskDescription.Gripper.Name", td->_gripperDevice->getName());
	tree.put("TaskDescription.Gripper.TCP", td->_gripperTCP->getName());
	tree.put("TaskDescription.Gripper.Movable", td->_gripperMovable->getName());
	tree.put("TaskDescription.Gripper.Controller", td->_controllerID);
	
	// save interference objects
	BOOST_FOREACH (rw::models::Object::Ptr obj, td->_interferenceObjects) {
		PTree node;
		
		node.put_value(obj->getName());
		
		tree.add_child("TaskDescription.InterferenceObjects.Object", node);
	}
	
	// save limits
	tree.put("TaskDescription.Limits.Interference", td->_interferenceLimit);
	tree.put("TaskDescription.Limits.Wrench", td->_wrenchLimit);
	tree.put("TaskDescription.Limits.Stress", td->_stressLimit);
	
	// save baseline
	//tree.put("TaskDescription.Baseline.Shape", td->_baseLine.shape);
	tree.put("TaskDescription.Baseline.Coverage", td->_baseLine.coverage);
	tree.put("TaskDescription.Baseline.SuccessRatio", td->_baseLine.success);
	tree.put("TaskDescription.Baseline.Wrench", td->_baseLine.wrench);
	
	// save weights
	//tree.put("TaskDescription.Weights.Shape", td->_weights.shape);
	tree.put("TaskDescription.Weights.Coverage", td->_weights.coverage);
	tree.put("TaskDescription.Weights.SuccessRatio", td->_weights.success);
	tree.put("TaskDescription.Weights.Wrench", td->_weights.wrench);
	tree.put("TaskDescription.Weights.Stress", td->_weights.stress);
	tree.put("TaskDescription.Weights.Volume", td->_weights.volume);
	
	// save teach & coverage distance
	Q teachDist = td->_teachDistance;
	teachDist(3) *= Rad2Deg;
	teachDist(4) *= Rad2Deg;
	tree.put("TaskDescription.TeachDistance", XMLHelpers::QToString(teachDist));
	Q preDist = td->_prefilteringDistance;
	preDist(1) *= Rad2Deg;
	preDist(2) *= Rad2Deg;
	tree.put("TaskDescription.PrefilteringDistance", XMLHelpers::QToString(preDist));
	Q covDist = td->_coverageDistance;
	//covDist(6) *= Rad2Deg;
	covDist(1) *= Rad2Deg;
	covDist(2) *= Rad2Deg;
	tree.put("TaskDescription.CoverageDistance", XMLHelpers::QToString(covDist));
	
	// save grasp hints
	BOOST_FOREACH (Transform3D<> hint, td->_hints) {
		PTree node;
		
		node.put("Pos", XMLHelpers::QToString(Q(3, hint.P()[0], hint.P()[1], hint.P()[2])));
		RPY<> rpy(hint.R());
		node.put("RPY", XMLHelpers::QToString(Q(3, rpy[0], rpy[1], rpy[2])));
		
		tree.add_child("TaskDescription.HintGrasps.Grasp", node);
	}
	
	// save alignments
	BOOST_FOREACH (Alignment a, td->_alignments) {
		PTree node;
		
		node.put("Pos", XMLHelpers::QToString(Q(3, a.pose.P()[0], a.pose.P()[1], a.pose.P()[2])));
		RPY<> rpy(a.pose.R());
		node.put("RPY", XMLHelpers::QToString(Q(3, rpy[0], rpy[1], rpy[2])));
		node.put("Dist", XMLHelpers::QToString(a.dist));
		
		tree.add_child("TaskDescription.Alignments.Alignment", node);
	}
	
	// save to XML
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(filename, tree, std::locale(), settings);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
