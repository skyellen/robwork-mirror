#include "TaskGenerator.hpp"



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



TaskGenerator::TaskGenerator(rwsim::dynamics::DynamicWorkCell::Ptr dwc, const std::string& objectID, const std::string& gripperID) :
	_dwc(dwc),
	_gripperID(gripperID),
	_jawdist(0.0)
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

