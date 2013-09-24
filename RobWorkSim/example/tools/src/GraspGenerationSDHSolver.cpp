/*
 * GraspGenerationSDHSolver.cpp
 *
 *  Created on: 02/07/2012
 *      Author: Thomas Thulesen (tnt@mmmi.sdu.dk)
 */

#include <iostream>
#include <sstream>
#include <vector>

#include "SDHInvKinSolver.hpp"

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwsim/util/SurfacePoseSampler.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rwlibs::task;
using namespace rwlibs::proximitystrategies;

Vector3D<> projPlane(Vector3D<> vec, Vector3D<> normal) {
	return vec-dot(vec,normal)*vec;
}

double angleProj(Vector3D<> v1, Vector3D<> v2, Vector3D<> normal) {
	return angle(projPlane(v1,normal),projPlane(v2,normal),normal);
}

int main(int argc, char** argv) {
	unsigned seed;
	read(open("/dev/urandom", O_RDONLY), &seed, sizeof seed);
	Math::seed(time(NULL)+seed);
	srand ( time(NULL)+seed );
	//Math::seed(2);
	//srand(2);

	if( argc < 6 ){
		std::cout << "------ Usage: " << std::endl;
		std::cout << "- Arg 1 name of workcell (.wc.xml) file" << std::endl;
		std::cout << "- Arg 2 name of object\n" << std::endl;
		std::cout << "- Arg 3 PG70, SDH\n" << std::endl;
		std::cout << "- Arg 4 name of gripper device\n" << std::endl;
		std::cout << "- Arg 5 number of samples\n" << std::endl;
		std::cout << "- Arg 6 name of output xml file\n" << std::endl;
		return 0;
	}
	std::string filename(argv[1]);
	std::string objectname(argv[2]);
	std::string grippertype(argv[3]);
	std::string grippername(argv[4]);
	std::string samples(argv[5]);
	std::string outfile(argv[6]);

	//DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(filename);
	//WorkCell::Ptr workcell = dwc->getWorkcell();

	WorkCell::Ptr workcell = WorkCellLoader::Factory::load(filename);
	RW_ASSERT(workcell);
	Object::Ptr object = workcell->findObject(objectname);
	Device::Ptr gripper = workcell->findDevice(grippername);
	//Frame* handoffset = workcell->findFrame("handoffset");
	MovableFrame* base = workcell->findFrame<MovableFrame>("SchunkHand.Base");

	Frame* tcpframe = workcell->findFrame("SchunkHand.SDHTCP");

	//Transform3D<> tcpTbase = Kinematics::frameTframe(tcpframe, gripper->getBase(), workcell->getDefaultState());
	Transform3D<> tcpTbase = Kinematics::frameTframe(tcpframe, base, workcell->getDefaultState());
	Transform3D<> objectTworld = Kinematics::worldTframe(object->getBase(), workcell->getDefaultState());

	SurfacePoseSampler::Ptr sampler = ownedPtr( new SurfacePoseSampler(object->getGeometry()) );
	sampler->setRandomPositionEnabled(false);
	sampler->setRandomRotationEnabled(false);
/*
	Q openQ(1,0.0);
	Q closeQ(1,1.0);
	if( grippertype=="PG70" ){
		openQ  = Q(1, 0.034);
		closeQ = Q(1, 0.0);
		std::cout << "PG70 Not yet implemented!" << std::endl;
		return 0;
	} else if( grippertype== "SDH"){ //SDH_BALL
		openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
	} else {
		std::cout << "Valid gripper types: PG70, SDH" << std::endl;
		return 0;
	}
*/
	CartesianTask::Ptr task = ownedPtr( new CartesianTask());
	//Outer: GraspController, Gripper, TCP, refframe
	task->getPropertyMap().set<std::string >("GraspController", "GraspController");
	task->getPropertyMap().set<std::string >("Gripper", grippername);
	task->getPropertyMap().set<std::string >("TCP", "SchunkHand.SDHTCP");

	GraspTask::Ptr gtasks = ownedPtr( new GraspTask(task) );

	SDHInvKinSolver* invKin = new SDHInvKinSolver();
	CollisionDetector::Ptr cd = ownedPtr(new CollisionDetector(workcell,ProximityStrategyFactory::makeDefaultCollisionStrategy()));

	State state = workcell->getDefaultState();

	Q aopen(7, -5*Deg2Rad, -5*Deg2Rad ,  0, -5*Deg2Rad, -5*Deg2Rad, -5*Deg2Rad, -5*Deg2Rad);
	Q aclose(7, 20*Deg2Rad,  35*Deg2Rad ,  0,  20*Deg2Rad,  35*Deg2Rad,  20*Deg2Rad,  35*Deg2Rad); // Decreased by 10 degrees everywhere

	int samplesInt;
	std::stringstream(samples) >> samplesInt;
	const int NR_OF_SAMPLES = samplesInt;
	bool collOpen;
	Transform3D<> pose, target;
	Vector3D<> approach;
	Q minQ(7, -Pi/2.0, -Pi/2.0,      0, -Pi/2.0, -Pi/2.0, -Pi/2.0, -Pi/2.0);
	Q maxQ(7,  Pi/2.0,  Pi/2.0, Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0,  Pi/2.0);
	Q tau, q;
	std::vector<boost::tuple<Transform3D<>, Q, bool> > res;
	std::vector<Transform3D<> > targets;
	for(int i=0; i<NR_OF_SAMPLES; i++){
		if (i%(NR_OF_SAMPLES/100)==0) std::cout << i << std::endl;
		if (grippertype=="PG70") {
			// Do nothing yet!
		} else if( grippertype== "SDH"){
			targets.clear();
			for(int j=0; j<3; j++) {
				target = sampler->sample();
				target.P() -= (target.R()*Vector3D<>::z())*Math::ran(0.005,0.03);
				targets.push_back(target);
			}
			approach = normalize(cross(targets[1].P()-targets[0].P(), targets[2].P()-targets[0].P()));
			Vector3D<> targetAvg = (targets[0].P()+targets[1].P()+targets[2].P())/3.0;
			//if(dot(Vector3D<>::z(),approach)<0) approach = -approach;

			//Filter + Inverse + Filter?

			bool filtered = false;
			for(size_t k=0;k<targets.size() && !filtered;k++){
				Vector3D<> pos = targets[k].P();
				Vector3D<> dir = targets[k].R()*Vector3D<>::z();// z points into the surface normal direction
				//Vector3D<> dir = normalize(pos-targetAvg);
				Vector3D<> yaxis = targets[k].R()*Vector3D<>::y();

				//std::cout<<  pos<<  " "<<  dir<<  std::endl;
				// make transformation with y pointing in approach direction

				//Vector3D<> xaxis = cross(approach,dir);
				//targets[k] = Transform3D<>(pos, Rotation3D<>(xaxis,approach,dir));

				double a = angleProj(yaxis,approach,dir);
				targets[k] = Transform3D<>(pos,targets[k].R()*(EAA<>(0.,0.,a)).toRotation3D());

				double ang = std::abs(angleProj(normalize(pos-targetAvg),dir,approach));
				if(ang > 45.*Deg2Rad && ang < 135.*Deg2Rad) filtered = true;
				ang = std::abs(angleProj(normalize(pos-targetAvg),dir,normalize(cross(approach,pos-targetAvg))));
				if(ang < 135.*Deg2Rad) filtered = true;
			}

			//Transform3D<> wTo = Kinematics::worldTframe(object->getBase(),workcell->getDefaultState());
			if (/*dot(wTo*approach,Vector3D<>::z())<=0.0 && */!filtered) {
			//if (true) {
				res = invKin->solve(targets, approach);
			} else {
				res.clear();
			}

			if(res.size()>0 && res[0].get<2>())
			{
				//std::cout << "Task Found at " << i;
				pose = res[0].get<0>();
				q = res[0].get<1>();
				//Adjust Forces (tau) & Clamp Q
				// TODO: again very specific to device
				Q openQ  = Math::clampQ(q + aopen, minQ, maxQ);
				Q closeQ = Math::clampQ(q + aclose, minQ, maxQ);

				//Q openQ2  = Math::clampQ(openQ, minQ, maxQ);
				//Q closeQ2 = Math::clampQ(q, minQ, maxQ);
				// TODO: this is SDH specific
				tau = Q(7, 2.0, 2.0, 10.0, 2.0, 2.0, 2.0, 2.0);
				// depending on the value of joint 2 adjust the forces
				double alpha = openQ(2);
				if(alpha<45*Deg2Rad){
					tau(3) = tau(0)/(2*cos(alpha));
					tau(5) = tau(0)/(2*cos(alpha));
				} else {
					tau(0) = std::max( 2*cos(alpha)*tau(3), 0.2);
				}
				state = workcell->getDefaultState();
				base->moveTo(objectTworld*pose*tcpTbase,state);
				gripper->setQ(openQ,state);
				//Filter + add Collision Check!
				state = workcell->getDefaultState();
				base->moveTo(objectTworld*pose*tcpTbase,state);
				gripper->setQ(openQ,state);
				collOpen = cd->inCollision(state);
				//gripper->setQ(closeQ2,state);
				//bool collClose = cd->inCollision(state);
				//collOpen = false;

				if(!collOpen) {
					GraspSubTask stask;
					stask.setRetract( Transform3D<>( Vector3D<>(0,0,0.1)) );
					stask.setOpenQ( openQ );
					stask.setCloseQ( closeQ );
					stask.setTauMax( tau );
					stask.setRefFrame(objectname);
					//std::ostringstream taskID;
					//taskID << i;
					//stask.setTaskID( taskID.str() );
					stask.addTarget( pose );
					gtasks->addSubTask( stask );
					//Uncertainty Measure + Priorities
					std::cout << "Task Added at " << i << std::endl;
					//std::cout << " - Added Task" << std::endl;
				//} else {
					//std::cout << " - In Collision" << std::endl;
				}
			}
		}
	}

	GraspTask::saveRWTask(gtasks,outfile);
	std::cout << "Finished" << std::endl;

	return 0;
}
