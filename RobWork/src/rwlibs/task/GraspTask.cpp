#include "GraspTask.hpp"

#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace std;
using namespace boost::numeric;
using namespace boost::property_tree;
using namespace rwlibs::task;

namespace {
void writeOutcome(std::ostream& out, GraspTarget& target) {
	GraspResult::Ptr res = target.result;
	if (res == NULL)
		return;

	int status = res->testStatus; // target->getPropertyMap().get<int> ("TestStatus");
	Q quality = res->qualityAfterLifting; //target->getPropertyMap().get<Q>("QualityAfterLifting", Q::zero(1));

	out << "    <outcome>\n";
	// if success we write all informal qualities

	if (status == GraspResult::Success || status == GraspResult::ObjectSlipped) {
		for (size_t i = 0; i < quality.size(); i++) {
			out << "     <informal uri=\"rwgq" << i << "\" quality=\""
					<< quality[i] << "\" />\n";
		}
		RW_ASSERT(quality.size() > 0);
		if (quality[0] > 0.01) { // we use a weak fixed quality rule here
			out << "     <success quality=\"" << quality[0] << "\" />\n";
		} else {
			out << "     <failure cause=\"OBJECTSLIPPED\" />\n";
		}
	} else {
		out << "     <failure cause=\"";
		switch (status) {
		case (GraspResult::ObjectDropped):
			out << "OBJECTDROPPED";
			break;
		case (GraspResult::UnInitialized):
			out << "UNINITIALIZED";
			break;
		case (GraspResult::CollisionInitially):
			out << "COLLISIONINITIALLY";
			break;
		case (GraspResult::ObjectMissed):
			out << "OBJECTMISSED";
			break;
		case (GraspResult::InvKinFailure):
			out << "INVKINFAILURE";
			break;
		case (GraspResult::TimeOut):
			out << "TIMEOUT";
			break;
		case (GraspResult::SimulationFailure):
			out << "SIMULATIONFAILURE";
			break;
		case (GraspResult::CollisionObjectInitially):
			out << "COLLISIONOBJECT";
			break;
		case (GraspResult::CollisionEnvironmentInitially):
			out << "COLLISIONENV";
			break;
		case (GraspResult::CollisionDuringExecution):
			out << "COLLISIONEXE";
			break;
		default:
			RW_THROW("Not supposed to go here!");
			break;
		}
		out << "\" />\n";
	}

	out << "    </outcome>\n";
}

void writePose(std::ostream& out, const Transform3D<>& pose) {
	Quaternion<> quat(pose.R());
	out << "<pose>\n" << " <position domain=\"R3\">\n" << "  <euclidean>"
			<< pose.P()[0] * 1000.0 << " " << pose.P()[1] * 1000.0 << " "
			<< pose.P()[2] * 1000.0 << "</euclidean>\n" << " </position>\n"
			<< " <orientation domain=\"SO3\">\n"
			<< "  <quaternion format=\"wxyz\">" << quat(3) << " " << quat(0)
			<< " " << quat(1) << " " << quat(2) << "</quaternion>\n"
			<< "  <rotmatrix>" << pose.R()(0, 0) << " " << pose.R()(0, 1) << " "
			<< pose.R()(0, 2) << "   " << pose.R()(1, 0) << " "
			<< pose.R()(1, 1) << " " << pose.R()(1, 2) << "   "
			<< pose.R()(2, 0) << " " << pose.R()(2, 1) << " " << pose.R()(2, 2)
			<< "   " << "  </rotmatrix>\n" << " </orientation>\n"
			<< "</pose>\n";
}

void writeUIBK(GraspTask::Ptr gtask, const std::string& outfile) {

	//rwlibs::task::CartesianTask::Ptr grasptask = gtask->getRootTask();

	std::ofstream fstr(outfile.c_str());
	fstr << setprecision(16);
	fstr << "<?xml version=\"1.0\"?> \n"
			<< "<experiments xmlns=\"http://iis.uibk.ac.at/ns/grasping\">\n";
	fstr
			<< "<notes> Generated with RobWork, rwsim::simulator::GraspTask </notes>\n";

	std::string gripperID = gtask->getGripperID();
	std::string tcpID = gtask->getTCPID();
	std::string graspcontrollerID = gtask->getGraspControllerID();

	BOOST_FOREACH(GraspSubTask &task, gtask->getSubTasks() ) {

		Transform3D<> wTe_n = task.offset; //->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
		//Transform3D<> approachDef = task.approach; //task->getPropertyMap().get<Transform3D<> >("Approach", Transform3D<>::identity());
		Q openQ = task.openQ; //task->getPropertyMap().get<Q>("OpenQ");
		Q closeQ = task.closeQ; //task->getPropertyMap().get<Q>("CloseQ");
		std::string objectId = task.objectID; //task->getPropertyMap().get<std::string>("Object",std::string("Undefined"));

		fstr << " <experiment>\n";
		fstr << "  <notes></notes>\n";
		fstr << "  <object type=\"" << objectId << "\">\n";
		fstr << "   <notes> </notes>\n";
		fstr << "  </object>\n";
		fstr << "  <gripper type=\"" << gripperID << "\">\n";
		fstr << "   <notes> GraspController:" << graspcontrollerID << " TCP:"
				<< tcpID << " CloseQ:" << closeQ << "</notes>\n";
		fstr << "   <params>";
		for (size_t i = 0; i < openQ.size(); i++)
			fstr << openQ[i] << " ";
		fstr << "   </params>\n" << "  </gripper>\n";
		fstr << "  <grasps>\n";
		fstr << "   <notes>  </notes>\n"; // don't have any notes yet

		// we don't add predictiondef
		//BOOST_FOREACH( rwlibs::task::CartesianTarget::Ptr target, task->getTargets() ){
		BOOST_FOREACH( GraspTarget &target, task.targets ) {
			Transform3D<> trans = wTe_n * target.pose;

			//bool has = target->getPropertyMap().has("ObjectTtcpApproach");
			//if(has)
			//    trans = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpApproach");

			fstr << "   <grasp>\n";
			writePose(fstr, trans);
			writeOutcome(fstr, target);
			fstr << "   </grasp>\n";
		}
		fstr << "  </grasps>\n";
		fstr << " </experiment>\n";

	}

	// and end with an experiments tag
	fstr << "</experiments>\n";
}

}

void GraspTask::saveUIBK(GraspTask::Ptr task, const std::string& name) {

	writeUIBK(task, name);
}

rwlibs::task::CartesianTask::Ptr GraspTask::toCartesianTask() {
	rwlibs::task::CartesianTask::Ptr root = ownedPtr(
			new rwlibs::task::CartesianTask());

	root->getPropertyMap().set<std::string>("Gripper", _gripperID);
	root->getPropertyMap().set<std::string>("TCP", _tcpID);
	root->getPropertyMap().set<std::string>("GraspController",
			_graspControllerID);
	//std::cout << "SIZE SUBTASKS" << _subtasks.size() << std::endl;
	BOOST_FOREACH(GraspSubTask &stask, _subtasks ) {
		rwlibs::task::CartesianTask::Ptr subtask = ownedPtr(
				new rwlibs::task::CartesianTask());
		root->addTask(subtask);

		subtask->getPropertyMap().set<std::string>("refframe", stask.refframe);
		;
		subtask->getPropertyMap().set<Transform3D<> >("Offset", stask.offset);
		subtask->getPropertyMap().set<Transform3D<> >("Approach",
				stask.approach);
		subtask->getPropertyMap().set<Transform3D<> >("Retract", stask.retract);
		subtask->getPropertyMap().set<Q>("OpenQ", stask.openQ);
		subtask->getPropertyMap().set<Q>("CloseQ", stask.closeQ);
		subtask->getPropertyMap().set<Q>("TauMax", stask.tauMax);
		subtask->setId(stask.getTaskID());

		//std::cout << "Size targets: " << stask.targets.size() << std::endl;
		BOOST_FOREACH(GraspTarget &gtarget, stask.targets) {
			CartesianTarget::Ptr ctarget = ownedPtr(
					new CartesianTarget(gtarget.pose));

			if (gtarget.result == NULL) {
				subtask->addTarget(ctarget);
				continue;
			}

			//gtarget.result = ownedPtr( new GraspResult() );
			GraspResult::Ptr result = gtarget.result;
			// all results saved in the target should be transferred
			ctarget->getPropertyMap().set<int>("TestStatus",
					result->testStatus);

			if (result->gripperConfigurationGrasp.size() > 0)
				ctarget->getPropertyMap().set<Q>("GripperConfiguration",
						result->gripperConfigurationGrasp);
			if (result->gripperConfigurationLift.size() > 0)
				ctarget->getPropertyMap().set<Q>("GripperConfigurationPost",
						result->gripperConfigurationLift);

			// configuration of gripper when task is done
			if (result->qualityBeforeLifting.size() > 0)
				ctarget->getPropertyMap().set<Q>("QualityBeforeLifting",
						result->qualityBeforeLifting);
			if (result->qualityAfterLifting.size() > 0)
				ctarget->getPropertyMap().set<Q>("QualityAfterLifting",
						result->qualityAfterLifting);

			if (!(result->objectTtcpTarget.equal(Transform3D<>::identity())))
				ctarget->getPropertyMap().set<Transform3D<> >(
						"ObjectTtcptTarget", result->objectTtcpTarget);
			if (!(result->objectTtcpApproach.equal(Transform3D<>::identity())))
				ctarget->getPropertyMap().set<Transform3D<> >(
						"ObjectTtcpApproach", result->objectTtcpApproach);
			if (!(result->objectTtcpGrasp.equal(Transform3D<>::identity())))
				ctarget->getPropertyMap().set<Transform3D<> >("ObjectTtcpGrasp",
						result->objectTtcpGrasp);
			if (!(result->objectTtcpLift.equal(Transform3D<>::identity())))
				ctarget->getPropertyMap().set<Transform3D<> >("ObjectTtcpLift",
						result->objectTtcpLift);

			if (result->testStatus == GraspResult::Success
					|| result->testStatus == GraspResult::ObjectSlipped
					|| result->testStatus == GraspResult::ObjectDropped) {
				ctarget->getPropertyMap().set<double>("LiftResult",
						result->liftresult);

				std::vector<double> contactlist(
						result->contactsGrasp.size() * 9, 0.0);
				size_t idxOffset = 0;
				BOOST_FOREACH(rw::sensor::Contact3D& contact, result->contactsGrasp) {
					contactlist[idxOffset + 0] = contact.p(0);
					contactlist[idxOffset + 1] = contact.p(1);
					contactlist[idxOffset + 2] = contact.p(2);
					contactlist[idxOffset + 3] = contact.n(0);
					contactlist[idxOffset + 4] = contact.n(1);
					contactlist[idxOffset + 5] = contact.n(2);
					contactlist[idxOffset + 6] = contact.f(0);
					contactlist[idxOffset + 7] = contact.f(1);
					contactlist[idxOffset + 8] = contact.f(2);
					idxOffset += 9;
				}
				if (contactlist.size() > 0) {
					ctarget->getPropertyMap().set<std::vector<double> >(
							"ContactsGrasp", contactlist);
				}
			}

			if (result->testStatus == GraspResult::Success
					|| result->testStatus == GraspResult::ObjectSlipped) {
				ctarget->getPropertyMap().set<int>("LiftStatus",
						GraspResult::Success);

				std::vector<double> contactlist(result->contactsLift.size() * 9,
						0.0);
				size_t idxOffset = 0;
				BOOST_FOREACH(rw::sensor::Contact3D& contact, result->contactsLift) {
					contactlist[idxOffset + 0] = contact.p(0);
					contactlist[idxOffset + 1] = contact.p(1);
					contactlist[idxOffset + 2] = contact.p(2);
					contactlist[idxOffset + 3] = contact.n(0);
					contactlist[idxOffset + 4] = contact.n(1);
					contactlist[idxOffset + 5] = contact.n(2);
					contactlist[idxOffset + 6] = contact.f(0);
					contactlist[idxOffset + 7] = contact.f(1);
					contactlist[idxOffset + 8] = contact.f(2);
					idxOffset += 9;
				}
				if (contactlist.size() > 0) {
					ctarget->getPropertyMap().set<std::vector<double> >(
							"ContactsLift", contactlist);
				}

			} else {
				ctarget->getPropertyMap().set<int>("LiftStatus",
						GraspResult::ObjectDropped);
			}

			ctarget->getPropertyMap().set<std::vector<rw::math::Transform3D<> > >(
					"InterferenceTs", result->interferenceTs);
			ctarget->getPropertyMap().set<std::vector<double> >(
					"InterferenceDistances", result->interferenceDistances);
			ctarget->getPropertyMap().set<std::vector<double> >(
					"InterferenceAngles", result->interferenceAngles);
			ctarget->getPropertyMap().set<std::vector<double> >("Interferences",
					result->interferences);
			ctarget->getPropertyMap().set<double>("Interference",
					result->interference);

			subtask->addTarget(ctarget);
		}
	}

	return root;
}

void GraspTask::saveRWTask(GraspTask::Ptr task, const std::string& name) {
	std::ofstream outfile(name.c_str());
	rwlibs::task::CartesianTask::Ptr ctask = task->toCartesianTask();
	try {
		XMLTaskSaver saver;
		saver.save(ctask, outfile);
	} catch (const Exception& exp) {
		RW_THROW("Unable to save task: " << exp.what());
	}

	outfile.close();
}

void GraspTask::saveText(GraspTask::Ptr gtask, const std::string& name) {
	std::ofstream outfile(name.c_str());
	if (!outfile.is_open())
		RW_THROW("Could not open file: " << name);
	outfile << setprecision(16);
	//int gripperDim = 0;
	std::string sep(";");
	// outfile << "// Description: {target.pos(3), target.rpy(3), TestStatus(1), GripperConfiguration("<<gripperDim<<"), "
	//         "GripperTObject.pos, GripperTObject.rpy, ObjectTtcpBefore.pos, ObjectTtcpBefore.rpy, ObjectTtcpAfter.pos, ObjectTtcpAfter.rpy}\n";
	outfile
			<< "// One grasp per line, Line Description (quat is xyzw encoded): "
					"target.pos(3), target.quat(4), TestStatus(1), "
					"ObjectTtcpTarget.pos(3); ObjectTtcpTarget.quat(4); "
					"ObjectTtcpBefore.pos(3); ObjectTtcpBefore.quat(4); "
					"ObjectTtcpAfter.pos(3); ObjectTtcpAfter.quat(4); "
					"GripperConfiguration(x)\n";

	outfile
			<< "// TestStatus enum { UnInitialized=0, Success=1, CollisionInitially=2, ObjectMissed=3, ObjectDropped=4, ObjectSlipped=5, TimeOut=6, SimulationFailure=7}\n";

	std::string gripperID = gtask->getGripperID();
	std::string tcpID = gtask->getTCPID();
	std::string graspcontrollerID = gtask->getGraspControllerID();
	//CartesianTask::Ptr root = gtask->getRootTask();

	//BOOST_FOREACH(CartesianTask::Ptr task, root->getTasks()){
	BOOST_FOREACH(GraspSubTask& task, gtask->getSubTasks()) {
		Q openQ = task.openQ; //getPropertyMap().get<Q>("OpenQ");
		Q closeQ = task.closeQ; //->getPropertyMap().get<Q>("CloseQ");

		//std::vector<CartesianTarget::Ptr> targets = task->getTargets();
		//outfile<<"{" << task->getId() << "}\n";
		//BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
		BOOST_FOREACH(GraspTarget& target, task.targets) {
			Transform3D<> ttrans = target.pose;
			//if(!target->getPropertyMap().has("ObjectTtcpTarget")){
			//    ttrans = target->get();
			//} else {
			//    Transform3D<> ttrans = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
			//}

			const Vector3D<>& pos = ttrans.P();
			Quaternion<> quat(ttrans.R());

			if (target.result == NULL) {
				continue;
			}

			int status = target.result->testStatus; //target->getPropertyMap().get<int>("TestStatus", GraspTask::UnInitialized);

			outfile << pos(0) << sep << pos(1) << sep << pos(2) << sep
					<< quat.getQx() << sep << quat.getQy() << sep
					<< quat.getQz() << sep << quat.getQw() << sep << status
					<< sep;

			Transform3D<> t3d;
			//Transform3D<> t3d = target.result->gripperTobjects[0]; //target->getPropertyMap().get<Transform3D<> >("GripperTObject0", Transform3D<>::identity());
			//RPY<> rpyObj(t3d.R());
			//quat = Quaternion<>(t3d.R());
			//outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
			//        << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

			t3d = target.result->objectTtcpTarget; //target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
			quat = Quaternion<>(t3d.R());
			outfile << t3d.P()[0] << sep << t3d.P()[1] << sep << t3d.P()[2]
					<< sep << quat.getQx() << sep << quat.getQy() << sep
					<< quat.getQz() << sep << quat.getQw() << sep;

			t3d = target.result->objectTtcpApproach; //target->getPropertyMap().get<Transform3D<> >("ObjectTtcpApproach", Transform3D<>::identity() );
			quat = Quaternion<>(t3d.R());
			outfile << t3d.P()[0] << sep << t3d.P()[1] << sep << t3d.P()[2]
					<< sep << quat.getQx() << sep << quat.getQy() << sep
					<< quat.getQz() << sep << quat.getQw() << sep;

			t3d = target.result->objectTtcpGrasp; //target->getPropertyMap().get<Transform3D<> >("ObjectTtcpGrasp", Transform3D<>::identity() );
			quat = Quaternion<>(t3d.R());
			outfile << t3d.P()[0] << sep << t3d.P()[1] << sep << t3d.P()[2]
					<< sep << quat.getQx() << sep << quat.getQy() << sep
					<< quat.getQz() << sep << quat.getQw() << sep;

			t3d = target.result->objectTtcpLift; //target->getPropertyMap().get<Transform3D<> >("ObjectTtcpLift", Transform3D<>::identity() );
			quat = Quaternion<>(t3d.R());
			outfile << t3d.P()[0] << sep << t3d.P()[1] << sep << t3d.P()[2]
					<< sep << quat.getQx() << sep << quat.getQy() << sep
					<< quat.getQz() << sep << quat.getQw() << sep;

			Q distance = target.result->gripperConfigurationLift; //->getPropertyMap().get<Q>("GripperConfigurationPost", Q::zero(gripperDim));
			for (size_t i = 0; i < distance.size(); i++)
				outfile << distance[i] << sep;

			outfile << "\n";
		}
	}
	outfile.close();
}

////////////////// GRASP TASK LOADING STUFF

namespace {

struct compareElemStrings: public std::binary_function<std::string, std::string,
		bool> {

	compareElemStrings() {
	}
	;

	bool operator()(const std::string& s1, const std::string& s2) const {
		// first we extract the name without namespaces (xmlns)
		std::string s1_tmp = s1;
		std::string s2_tmp = s2;

		size_t found = s1.find_last_of(':');
		if (found != std::string::npos) {
			s1_tmp = s1.substr(found + 1);
		}

		found = s2.find_last_of(':');
		if (found != std::string::npos) {
			s2_tmp = s2.substr(found + 1);
		}
		//std::cout << s1_tmp << "  " << s2_tmp << std::endl;
		return std::less<std::string>()(s1_tmp, s2_tmp);
	}

};

}

typedef boost::property_tree::basic_ptree<std::string, std::string,
		compareElemStrings> PTree;

namespace {
typedef boost::property_tree::basic_ptree<std::string, std::string,
		compareElemStrings>::iterator CI;
typedef PTree::assoc_iterator OCI;

struct ParserState {
public:
	ParserState(std::string file) :
			dwcfile(file), targetNr(0) {
	}

	const std::string dwcfile, dir;
	int targetNr;
};

bool isName(const std::string& elemName, const std::string& matchName) {
	// first we extract the name without namespaces (xmlns)
	std::string elem = elemName;
	size_t found = elemName.find_last_of(':');
	if (found != std::string::npos) {
		elem = elemName.substr(found + 1);
	}
	return elem == matchName;
}

bool has_child(PTree& tree, const std::string& name) {
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (isName(p->first, name))
			return true;
	}
	return false;
}

std::pair<bool, double> toDouble(const std::string& str) {
	std::pair<bool, double> nothing(false, 0);
	istringstream buf(str);
	double x;
	buf >> x;
	if (!buf)
		return nothing;
	string rest;
	buf >> rest;
	if (buf)
		return nothing;
	else
		return make_pair(true, x);
}

std::vector<double> readArray(PTree& tree) {
	istringstream buf(tree.get_value<string>());
	std::vector<double> values;

	std::string str;
	while (buf >> str) {
		const pair<bool, double> okNum = toDouble(str);
		if (!okNum.first)
			RW_THROW("Number expected. Got \"" << str << "\" ");
		values.push_back(okNum.second);
	}
	return values;
}

Q readQ(PTree& tree) {
	//Log::debugLog()<< "ReadQ" << std::endl;
	std::vector<double> arr = readArray(tree);
	Q q(arr.size());
	for (size_t i = 0; i < q.size(); i++) {
		q[i] = arr[i];
	}
	return q;
}

Vector3D<> readVector3D(PTree& tree) {
	//Log::debugLog()<< "ReadVector3D" << std::endl;
	Q q = readQ(tree);
	if (q.size() != 3)
		RW_THROW("Unexpected sequence of values, must be length 3");
	return Vector3D<>(q[0], q[1], q[2]);
}

rwlibs::task::CartesianTarget::Ptr readGrasp(PTree& tree, ParserState& state) {
	rwlibs::task::CartesianTarget::Ptr target = ownedPtr(
			new rwlibs::task::CartesianTarget(Transform3D<>()));
	std::vector<double> qualities;
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		//std::cout << p->first << std::endl;
		if (isName(p->first, "pose")) {
			// position
			PTree& pos_tree = p->second.get_child("position");
			std::string posdomain = pos_tree.get_child("<xmlattr>").get<
					std::string>("domain", "R3");
			Vector3D<> pos = readVector3D(pos_tree.get_child("euclidean"))
					/ 1000.0;

			// orientation
			//std::cout << "---------------------------" << std::endl;
			PTree& rot_tree = p->second.get_child("orientation");
			std::string rotdomain = pos_tree.get_child("<xmlattr>").get<
					std::string>("domain", "SO3");
			Rotation3D<> rot;
			bool qrotDone = false; // we prefer using quaternion if possible
			for (CI p1 = rot_tree.begin(); p1 != rot_tree.end(); ++p1) {
				//std::cout << p1->first << std::endl;
				if (isName(p1->first, "quaternion")) {
					std::vector<double> vals = readArray(p1->second);
					if (vals.size() != 4)
						RW_THROW("quaternion is wrongly dimensioned!");

					Quaternion<> quat(vals[1], vals[2], vals[3], vals[0]);
					quat.normalize();
					rot = quat.toRotation3D();
					qrotDone = true;
				} else if (isName(p1->first, "rotmatrix")) {
					if (!qrotDone) {
						std::vector<double> vals = readArray(p1->second);
						if (vals.size() != 9)
							RW_THROW("rotmatrix is wrongly dimensioned!");
						rot = Rotation3D<>(vals[0], vals[1], vals[2], vals[3],
								vals[4], vals[5], vals[6], vals[7], vals[8]);
						rot.normalize();
					}
				} else {
					//RW_THROW("Unknown element!" << p1->first);
				}
			}

			//ctask->getPropertyMap().set<std::string>("GripperName", gripperType);
			target->get() = Transform3D<>(pos, rot);
		} else if (isName(p->first, "prediction")) {
			double prediction = toDouble(p->second.get_value<string>()).second;
			//double squal = p->second.get<double>("quality",0.0);
			qualities.push_back(prediction);

			//<prediction def="http://iis.uibk.ac.at/uri/gd-exchange/probability-generative/kde/default">0.65691438799972202</prediction>
		} else if (isName(p->first, "outcome")) {

			int status = GraspResult::UnInitialized;
			if (has_child(p->second, "success")) {
				status = GraspResult::Success;
				if (has_child(p->second.get_child("success"), "<xmlattr>")) {
					double squal = p->second.get_child("success").get_child(
							"<xmlattr>").get<double>("quality", 0.0);
					qualities.push_back(squal);
				} else {
					qualities.push_back(0.0);
				}
			} else if (has_child(p->second, "failure")) {
				status = GraspResult::ObjectDropped;
				std::string cause = p->second.get_child("failure").get_child(
						"<xmlattr>").get<std::string>("cause");
				qualities.push_back(0.0);
				if (cause == "UNINITIALIZED") {
					status = GraspResult::UnInitialized;
				} else if (cause == "COLLISIONINITIALLY") {
					status = GraspResult::CollisionInitially;
				} else if (cause == "TIMEOUT") {
					status = GraspResult::TimeOut;
				} else if (cause == "OBJECTMISSED") {
					status = GraspResult::ObjectMissed;
				} else if (cause == "OBJECTDROPPED") {
					status = GraspResult::ObjectDropped;
				} else if (cause == "OBJECTSLIPPED") {
					status = GraspResult::ObjectSlipped;
				} else if (cause == "SIMULATIONFAILURE") {
					status = GraspResult::SimulationFailure;
				} else if (cause == "POSEESTIMATEFAILURE") {
					status = GraspResult::PoseEstimateFailure;
				} else if (cause == "INVKINFAILURE") {
					status = GraspResult::InvKinFailure;
				} else if (cause == "COLLISIONEXE") {
					status = GraspResult::CollisionDuringExecution;
				}
			} else {
				qualities.push_back(0.0);
			}
			// todo: get all informal quality measures
			for (CI p1 = p->second.begin(); p1 != p->second.end(); ++p1) {
				if (isName(p1->first, "informal")) {
					double qualval = p1->second.get_child("<xmlattr>").get<
							double>("quality", 0.0);
					qualities.push_back(qualval);
				}
			}

			target->getPropertyMap().set<int>("TestStatus", (int) status);
			// TODO: convert from UIBK to RW format
		}
		if (qualities.size() > 0) {
			Q qqual(qualities.size(), &qualities[0]);
			target->getPropertyMap().set<Q>("QualityAfterLifting", qqual);
		} else {
			target->getPropertyMap().set<Q>("QualityAfterLifting", Q(1, -1));
		}

	}
	return target;
}

rwlibs::task::CartesianTask::Ptr readExperiment(PTree& tree,
		ParserState& state) {

	//std::cout << "experiment" << std::endl;
	rwlibs::task::CartesianTask::Ptr ctask = ownedPtr(
			new rwlibs::task::CartesianTask());
	std::vector<double> qualities;

	//GraspTask::Status status = GraspTask::UnInitialized;
	//for (OCI p = tree.ordered_begin(); p != tree.not_found(); ++p) {
	for (CI p = tree.begin(); p != tree.end(); ++p) {

		//std::cout << p->first << "\n";
		if (isName(p->first, "gripper")) {
			string gripperType = p->second.get_child("<xmlattr>").get<
					std::string>("type");
			Q params = readQ(p->second.get_child("params"));
			ctask->getPropertyMap().set<std::string>("Gripper", gripperType);
			// TODO: get notes
		} else if (isName(p->first, "object")) {
			string objectName =
					p->second.get_child("<xmlattr>").get<std::string>("type");
			ctask->getPropertyMap().set<std::string>("Object", objectName);
			// TODO: get notes
		} else if (isName(p->first, "predictiondef")) {

		} else if (isName(p->first, "grasps")) {
			for (CI p1 = p->second.begin(); p1 != p->second.end(); ++p1) {
				if (isName(p1->first, "grasp")) {

					CartesianTarget::Ptr target = readGrasp(p1->second, state);
					ctask->addTarget(target);

				} else if (isName(p1->first, "notes")) {
					// TODO: add notes
				}
			}
		} else if (isName(p->first, "<xmlattr>")) {
			// todo: uri
		} else if (isName(p->first, "notes")) {
		} else {
			RW_THROW("Unknown element!" << p->first);
		}
		//std::cout << "read experiment end" << std::endl;
	}

	return ctask;
}

rwlibs::task::CartesianTask::Ptr readGrasps(PTree& data, ParserState& state) {
	rwlibs::task::CartesianTask::Ptr grasptasks = ownedPtr(
			new rwlibs::task::CartesianTask());

	for (CI p = data.begin(); p != data.end(); ++p) {
		if (isName(p->first, "grasp")) {
			rwlibs::task::CartesianTarget::Ptr target = readGrasp(p->second,
					state);
			grasptasks->addTarget(target);
		} else if (p->first == "<xmlattr>") {
		} else {
			RW_THROW("Unknown element!" << p->first);
		}
	}

	// get tasks from state

	return grasptasks;
}

rwlibs::task::CartesianTask::Ptr readExperiments(PTree& data,
		ParserState& state) {
	// this is a container for experiments
	rwlibs::task::CartesianTask::Ptr grasptasks = ownedPtr(
			new rwlibs::task::CartesianTask());
	//std::cout << "read experiments" << std::endl;
	for (CI p = data.begin(); p != data.end(); ++p) {
		// each experiment is a GraspTask
		if (isName(p->first, "experiment")) {
			grasptasks->addTask(readExperiment(p->second, state));
		} else if (p->first == "notes") {
			// grasptasks->getPropertyMap().set<std::string>( );
		} else if (p->first == "<xmlattr>") {
			// uri
		}
	}
	//std::cout << "read experiments end" << std::endl;
	return grasptasks;

}
}

GraspTask::Ptr GraspTask::load(const std::string& filename) {

	std::string file = IOUtil::getAbsoluteFileName(filename);
	std::string firstelem = IOUtil::getFirstXMLElement(file);
	//std::cout << "FIRST ELEMENT: " << firstelem << std::endl;

	rwlibs::task::CartesianTask::Ptr grasptask;

	if (firstelem == "CartesianTask") {
		XMLTaskLoader loader;
		loader.load(file);
		grasptask = loader.getCartesianTask();
	} else {

		try {
			ParserState state(file);

			//state.dir = StringUtil::getDirectoryName(file);
			PTree tree;
			read_xml(file, tree);

			for (CI p = tree.begin(); p != tree.end(); ++p) {
				//std::cout << p->first << "\n";
				if (isName(p->first, "experiments")) {
					grasptask = readExperiments(p->second, state);
				}
			}
			//rw::loaders::XML::printTree(tree, std::cout);
		} catch (const ptree_error& e) {
			// Convert from parse errors to RobWork errors.
			RW_THROW(e.what());
		}

	}

	GraspTask::Ptr gtask = ownedPtr(new GraspTask(grasptask));
	return gtask;
}

GraspTask::Ptr GraspTask::load(std::istringstream& inputStream) {

	std::istringstream streamCopy;
	streamCopy.str(inputStream.str());

	std::string firstelem = IOUtil::getFirstXMLElement(streamCopy);

	rwlibs::task::CartesianTask::Ptr grasptask;

	if (firstelem == "CartesianTask") {
		XMLTaskLoader loader;
		loader.load(inputStream);
		grasptask = loader.getCartesianTask();
	} else {

		try {
			ParserState state("");

			PTree tree;
			read_xml(inputStream, tree);

			for (CI p = tree.begin(); p != tree.end(); ++p) {
				if (isName(p->first, "experiments")) {
					grasptask = readExperiments(p->second, state);
				}
			}
		} catch (const ptree_error& e) {
			// Convert from parse errors to RobWork errors.
			RW_THROW(e.what());
		}

	}

	GraspTask::Ptr gtask = ownedPtr(new GraspTask(grasptask));
	return gtask;
}

GraspTask::GraspTask(rwlibs::task::CartesianTask::Ptr task) {
	// convert from the Carteasean format
	_gripperID = task->getPropertyMap().get<std::string>("Gripper", "");
	_tcpID = task->getPropertyMap().get<std::string>("TCP", "");
	_graspControllerID = task->getPropertyMap().get<std::string>(
			"GraspController", "");

	_subtasks.resize(task->getTasks().size());
	//std::cout << "NR SUB TASKS: " << task->getTasks().size() << std::endl;
	for (size_t i = 0; i < task->getTasks().size(); i++) {
		rwlibs::task::CartesianTask::Ptr stask = task->getTasks()[i];
		_subtasks[i].refframe = stask->getPropertyMap().get<std::string>(
				"refframe", "WORLD");
		;
		_subtasks[i].offset = stask->getPropertyMap().get<Transform3D<> >(
				"Offset", Transform3D<>::identity());
		_subtasks[i].approach = stask->getPropertyMap().get<Transform3D<> >(
				"Approach", Transform3D<>::identity());
		_subtasks[i].retract = stask->getPropertyMap().get<Transform3D<> >(
				"Retract", Transform3D<>::identity());
		_subtasks[i].openQ = stask->getPropertyMap().get<Q>("OpenQ", Q());
		_subtasks[i].closeQ = stask->getPropertyMap().get<Q>("CloseQ", Q());
		_subtasks[i].tauMax = stask->getPropertyMap().get<Q>("TauMax", Q());
		_subtasks[i].setTaskID(stask->getId());

		_subtasks[i].targets.resize(stask->getTargets().size());
		// std::cout << "Targets size: " <<  stask->getTargets().size() << std::endl;
		for (size_t j = 0; j < stask->getTargets().size(); j++) {
			CartesianTarget::Ptr ctarget = stask->getTargets()[j];
			_subtasks[i].targets[j].pose = ctarget->get();
			_subtasks[i].targets[j].result = ownedPtr(new GraspResult());

			GraspResult::Ptr result = _subtasks[i].targets[j].result;
			// all results saved in the target should be transferred
			result->testStatus = ctarget->getPropertyMap().get<int>(
					"TestStatus", GraspResult::UnInitialized);
			result->liftresult = ctarget->getPropertyMap().get<double>(
					"LiftResult", 0.0);

			result->gripperConfigurationGrasp =
					ctarget->getPropertyMap().get<Q>("GripperConfiguration",
							Q());
			result->gripperConfigurationLift = ctarget->getPropertyMap().get<Q>(
					"GripperConfigurationPost", Q());

			result->qualityBeforeLifting = ctarget->getPropertyMap().get<Q>(
					"QualityBeforeLifting", Q());
			result->qualityAfterLifting = ctarget->getPropertyMap().get<Q>(
					"QualityAfterLifting", Q());

			result->objectTtcpTarget = ctarget->getPropertyMap().get<
					Transform3D<> >("ObjectTtcptTarget",
					Transform3D<>::identity());
			result->objectTtcpApproach = ctarget->getPropertyMap().get<
					Transform3D<> >("ObjectTtcpApproach",
					Transform3D<>::identity());
			result->objectTtcpGrasp = ctarget->getPropertyMap().get<
					Transform3D<> >("ObjectTtcpGrasp",
					Transform3D<>::identity());
			result->objectTtcpLift =
					ctarget->getPropertyMap().get<Transform3D<> >(
							"ObjectTtcpLift", Transform3D<>::identity());

			std::vector<double> contactlist = ctarget->getPropertyMap().get<
					std::vector<double> >("ContactsGrasp",
					std::vector<double>());
			if (contactlist.size() > 0) {
				for (size_t m = 0; m < contactlist.size(); m += 9) {
					rw::sensor::Contact3D contact;
					contact.p(0) = contactlist[m + 0];
					contact.p(1) = contactlist[m + 1];
					contact.p(2) = contactlist[m + 2];
					contact.n(0) = contactlist[m + 3];
					contact.n(1) = contactlist[m + 4];
					contact.n(2) = contactlist[m + 5];
					contact.f(0) = contactlist[m + 6];
					contact.f(1) = contactlist[m + 7];
					contact.f(2) = contactlist[m + 8];
					result->contactsGrasp.push_back(contact);
				}
			}

			contactlist = ctarget->getPropertyMap().get<std::vector<double> >(
					"ContactsLift", std::vector<double>());
			if (contactlist.size() > 0) {
				for (size_t m = 0; m < contactlist.size(); m += 9) {
					rw::sensor::Contact3D contact;
					contact.p(0) = contactlist[m + 0];
					contact.p(1) = contactlist[m + 1];
					contact.p(2) = contactlist[m + 2];
					contact.n(0) = contactlist[m + 3];
					contact.n(1) = contactlist[m + 4];
					contact.n(2) = contactlist[m + 5];
					contact.f(0) = contactlist[m + 6];
					contact.f(1) = contactlist[m + 7];
					contact.f(2) = contactlist[m + 8];
					result->contactsLift.push_back(contact);
				}
			}

		}
	}
}

void GraspTask::setGripperID(const std::string& id) {
	_gripperID = id;
}
void GraspTask::setTCPID(const std::string& id) {
	_tcpID = id;
}
void GraspTask::setGraspControllerID(const std::string& id) {
	_graspControllerID = id;
}

std::string GraspTask::getGripperID() {
	return _gripperID;
}
std::string GraspTask::getTCPID() {
	return _tcpID;
}
std::string GraspTask::getGraspControllerID() {
	return _graspControllerID;
}

void GraspTask::filterTasks(std::vector<GraspResult::TestStatus> &includeMask) {
	// generate map out of the includeMask vector
	std::map<int, bool> includeMap;

	for (int i = 0; i < GraspResult::SizeOfStatusArray; i++) {
		includeMap[i] = false;
	}

	BOOST_FOREACH(GraspResult::TestStatus includeRule, includeMask) {
		includeMap[(int) includeRule] = true;
	}

	BOOST_FOREACH(GraspSubTask &stask, getSubTasks()) {
		std::vector<GraspTarget> stargets;

		BOOST_FOREACH(GraspTarget &target, stask.targets) {
			if (target.result == NULL) {
				continue;
			}
			GraspResult::TestStatus status =
					(GraspResult::TestStatus) target.result->testStatus;

			if (includeMap[status]) {
				stargets.push_back(target);
			}
		}
		stask.targets = stargets;
	}
}

std::vector<std::pair<GraspSubTask*, GraspTarget*> > GraspTask::getAllTargets() {
	std::vector<std::pair<GraspSubTask*, GraspTarget*> > result;

	BOOST_FOREACH(GraspSubTask &stask, getSubTasks()) {
		BOOST_FOREACH(GraspTarget &target, stask.getTargets() ) {
			result.push_back(std::make_pair(&stask, &target));
		}
	}

	return result;
}
