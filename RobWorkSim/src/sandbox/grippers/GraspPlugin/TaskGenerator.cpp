#include "TaskGenerator.hpp"

#include <vector>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



TaskGenerator::TaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID) :
	_dwc(dwc),
	_gripperID(gripperID),
	_jawdist(0.0),
	_sampledParSurfaces(0),
	_allSamples(NULL)
{
	_wc = _dwc->getWorkcell();
	if (_wc == NULL || _dwc == NULL) {
		RW_ASSERT("Null workcell or dynamic workcell.");
	}
	
	_object = _wc->findObject(objectID);
	if (_object == NULL || _object->getGeometry().size() == 0) {
		RW_ASSERT("Invalid name of the object or no object geometry.");
	}
	
	_gripper = _wc->findDevice(gripperID);
	if (_gripper == NULL) {
		RW_ASSERT("Cannot find the gripper.");
	}
	
	_gripperMovable = dynamic_cast<MovableFrame*>(_gripper->getBase());
	string tcpFrame = _wc->getPropertyMap().get<string>("gripperTCP");
	_gripperTCP = _wc->findFrame(tcpFrame);

	if (_gripperTCP == NULL || _gripperMovable == NULL) {
		RW_ASSERT("Cannot find gripper TCP or movable base.");
	}
	
	_closeQ = Q(1, _gripper->getBounds().first[0]);
	_openQ = Q(1, _gripper->getBounds().second[0]);
	
	//cout << _openQ << _closeQ << endl;
}



void TaskGenerator::moveFrameW(const Transform3D<>& wTtcp, Frame* tcp,
			MovableFrame* base, State& state)
{
    Transform3D<> tcpTbase = Kinematics::frameTframe(tcp, base, state);
    Transform3D<> wTbase_target = wTtcp * tcpTbase;
   
    base->moveTo(wTbase_target, state);
}



rwlibs::task::GraspTask::Ptr TaskGenerator::filterTasks(const rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff)
{
	// create nodes for succesful grasps
	typedef GraspResult::Ptr ValueType;
	typedef KDTreeQ<ValueType> NNSearch;
	vector<NNSearch::KDNode> nodes;
	
	BOOST_FOREACH(GraspTarget& target, tasks->getSubTasks()[0].getTargets()) {
		//if (target.getResult()->testStatus == GraspTask::Success) {
			Q key(7);
            key[0] = target.pose.P()[0];
            key[1] = target.pose.P()[1];
            key[2] = target.pose.P()[2];
            EAA<> eaa(target.pose.R());
            key[3] = eaa.axis()(0);
            key[4] = eaa.axis()(1);
            key[5] = eaa.axis()(2);
            key[6] = eaa.angle();
            //key[7] = 0; // this is to remove neighbouring grasps
            
            //cout << key << endl;
            
			nodes.push_back(NNSearch::KDNode(key, target.getResult()));
		//}
	}
	
	NNSearch *nntree = NNSearch::buildTree(nodes);
	//Q diff(7, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 15*Deg2Rad);
    std::list<const NNSearch::KDNode*> result;
    
    int nFiltered = 0;
    BOOST_FOREACH (NNSearch::KDNode& node, nodes) {
		if (node.value->testStatus != GraspTask::TimeOut) {
			result.clear();
			Q key = node.key;
			nntree->nnSearchRect(key-diff, key+diff, result);
			//cout << result.size() << " of neighbours" << endl;

			int g = 0;
			BOOST_FOREACH (const NNSearch::KDNode* n, result) {
				if (n->value->testStatus != GraspTask::TimeOut) ++g;
				const_cast<NNSearch::KDNode*>(n)->value->testStatus = GraspTask::TimeOut;
				//cout << n->value->testStatus << endl;
				
			}
			nFiltered += g;
			//cout << "Deleted " << g << " neighbours; left: " << nodes.size() - nFiltered << endl;
			
			//++nFiltered;
		}
	}
	
	cout << "Total number of grasps: " << nodes.size() << " Filtered: " << nodes.size() - nFiltered << endl;
	//cout << "COVERAGE: " << 1.0*(nodes.size() - nFiltered)/nodes.size() << endl;
	//cout << diff << endl;
	
	return tasks;
}

