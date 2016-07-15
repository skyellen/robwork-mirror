#include <fstream>
#include <vector>
#include <string>
#include <stdio.h>

#include <rw/math/Random.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/task/GraspTask.hpp>

using namespace std;
using rw::common::ownedPtr;
using namespace rw::math;
using namespace rwlibs::task;

const double SOFT_LAYER_SIZE = 0.0005;

void loadPoses(std::string filename, std::vector<Transform3D<> >& initialPoses, std::vector<Transform3D<> >& sampledPoses){
    std::ifstream in;
    in.open(filename.c_str());
    char line[200];

    in.getline(line,200);
    int initialPoseCnt, samplesPerPose;
    sscanf(line, "%i", &initialPoseCnt);
    in.getline(line,200);
    sscanf(line, "%i", &samplesPerPose);

    std::cout << "initialPoseCnt: " << initialPoseCnt << std::endl;
    std::cout << "samplesPerPose: " << samplesPerPose << std::endl;

    float x,y,z,r[9];
    // first we read initial psoes
    for(int i=0;i<initialPoseCnt;i++) {
        in.getline(line,200);
        sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f", &x, &y, &z, &r[0], &r[1], &r[2], &r[3], &r[4], &r[5], &r[6], &r[7], &r[8] );
        Rotation3D<> rot(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
        rot.normalize();
        initialPoses.push_back( inverse( Transform3D<>(Vector3D<>(x,y,z), rot)));
    }

    // next we read the poses to simulate
    for(int i=0;i<initialPoseCnt*samplesPerPose;i++) {
        in.getline(line,200);
        sscanf(line, "%f %f %f %f %f %f %f %f %f %f %f %f", &x, &y, &z, &r[0], &r[1], &r[2], &r[3], &r[4], &r[5], &r[6], &r[7], &r[8] );
        Rotation3D<> rot(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
        rot.normalize();
        sampledPoses.push_back( inverse( Transform3D<>(Vector3D<>(x,y,z), rot)));
    }

}
#include <boost/lexical_cast.hpp>
int main(int argc, char** argv)
{
    Random::seed(time(NULL));
    srand ( time(NULL) );

    if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of HGP txt file" << std::endl;
	    std::cout << "- Arg 2 name of output grasp task file" << std::endl;
	    return 0;
	}

	std::string grasptask_file(argv[1]);
	std::string grasptask_file_out( argv[2] );

	std::vector<Transform3D<> > initPoses, samplePoses;
	std::cout << "Loading poses from txt file" << std::endl;
	loadPoses(grasptask_file, initPoses, samplePoses);

	std::cout << "Creating tasks" << std::endl;
	GraspTask::Ptr gtask = ownedPtr( new GraspTask() );
	// add one task with all the targets

	gtask->setGripperID("GS20");
	gtask->setTCPID("TCPGS20");
	gtask->setGraspControllerID("GraspController");

	gtask->getSubTasks().resize(1);
    gtask->getSubTasks()[0].closeQ = Q(1, 0.0005);
    gtask->getSubTasks()[0].openQ = Q(1, 0.005);
    gtask->getSubTasks()[0].retract.P() = Vector3D<>(0,0,-0.04);
	std::vector<GraspTarget>& targets = gtask->getSubTasks()[0].getTargets();
	BOOST_FOREACH(Transform3D<> &t3d, initPoses){
	    targets.push_back( GraspTarget( t3d ) );
	}

	GraspTask::saveRWTask( gtask, grasptask_file_out+"_init.uibk.xml" );
    //gtask->getSubTasks().resize(1);
    //std::vector<GraspTarget>& targets = gtask->getSubTasks()[0].getTargets();
	int nrPosesWritten=0, nrSaves=0;
    BOOST_FOREACH(Transform3D<> &t3d, samplePoses){
        targets.push_back( GraspTarget( t3d ) );
        nrPosesWritten++;
        if(nrPosesWritten>30000){
            GraspTask::saveRWTask( gtask, grasptask_file_out+"_"+boost::lexical_cast<std::string>(nrSaves)+"_init_samples.uibk.xml" );
            nrSaves++;
            targets.clear();
            nrPosesWritten = 0;
        }
    }
    GraspTask::saveRWTask( gtask, grasptask_file_out+"_"+boost::lexical_cast<std::string>(nrSaves)+"_init_samples.uibk.xml" );

    std::vector<GraspTarget>& targets2 = gtask->getSubTasks()[0].getTargets();
    gtask->getSubTasks()[0] = GraspSubTask();
    gtask->getSubTasks()[0].closeQ = Q(1, 0.0005);
    gtask->getSubTasks()[0].openQ = Q(1, 0.005);
    gtask->getSubTasks()[0].retract.P() = Vector3D<>(0,0,-0.04);

    nrPosesWritten=0; nrSaves=0;
    BOOST_FOREACH(Transform3D<> &t3d, samplePoses){
        targets2.push_back( GraspTarget( t3d ) );
        nrPosesWritten++;
        if(nrPosesWritten>30000){
            GraspTask::saveRWTask( gtask, grasptask_file_out+"_"+boost::lexical_cast<std::string>(nrSaves)+"_samples.uibk.xml" );
            nrSaves++;
            targets2.clear();
            nrPosesWritten = 0;
        }
    }
    GraspTask::saveRWTask( gtask, grasptask_file_out+"_"+boost::lexical_cast<std::string>(nrSaves)+"_samples.uibk.xml" );



    return 0;
}
