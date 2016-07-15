/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <vector>

#include <rw/common/Log.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/trajectory/Path.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

#include <fstream>

using namespace rw::common;
using namespace rw::kinematics;
using rw::loaders::PathLoader;
using namespace rw::math;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::dynamics::DynamicWorkCell;

void saveDist(std::string filename, std::vector<Transform3D<> >& poses, Rotation3D<> rot){
    std::cout << "Openning file: " << filename << std::endl;
    std::ofstream file( filename.c_str() );
    TimerUtil::sleepMs(100);

    file << "axis[0] \t axis[1] \t axis[2] \n";
    file << "0" << "\t" << "0" << "\t" << "1" << "\n";

    file << "p[0] \t p[1] \t p[2] \t rpy[0] \t rpy[1] \t rpy[2] \t eaa[0] \t eaa[1] \t eaa[2] \t";
    file << "rot(0,0) \t rot(1,0) \t rot(2,0) \t";
    file << "rot(0,1) \t rot(1,1) \t rot(2,1) \t";
    file << "rot(0,2) \t rot(1,2) \t rot(2,2) \n";
    for(size_t j=0;j<poses.size();j++){
        Transform3D<> t = poses[j];
        EAA<> eaa(rot*t.R());
        RPY<> rpy(rot*t.R());
        Rotation3D<> rot = rot*t.R();
        file << t.P()[0] << "\t" << t.P()[1] << "\t" << t.P()[2] << "\t";
        file << rpy(0) << "\t" << rpy(1) << "\t" << rpy(2) << "\t";
        file << eaa(0) << "\t" << eaa(1) << "\t" << eaa(2) << "\t";
        file << rot(0,0) << "\t" << rot(1,0) << "\t" << rot(2,0) << "\t";
        file << rot(0,1) << "\t" << rot(1,1) << "\t" << rot(2,1) << "\t";
        file << rot(0,2) << "\t" << rot(1,2) << "\t" << rot(2,2) << "\n";

    }
    file.close();

}

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of dynamic workcell input" << std::endl;
	    std::cout << "- Arg 1 name of path to filter" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
    std::string startpath(argv[2]);
    std::string endpath(argv[3]);
    std::string endid(argv[4]);

	Log::infoLog() << "Loading workcell" << std::endl;
	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(filename);
	Log::infoLog() << "workcell loadet" << std::endl;

	MovableFrame *plate = dwc->getWorkcell()->findFrame<MovableFrame>("plate");
	RW_ASSERT(plate);

	Log::infoLog() << "saving workcell" << std::endl;

	const rw::trajectory::TimedStatePath path = PathLoader::loadTimedStatePath(*dwc->getWorkcell(),endpath);
	const rw::trajectory::TimedStatePath startPath = PathLoader::loadTimedStatePath(*dwc->getWorkcell(),startpath);

	rw::trajectory::TimedStatePath outpath, startoutpath, outpathmisses, startoutpathmisses;
	std::vector<Transform3D<> > endTrans, startTrans;
	std::vector<Transform3D<> > endTransMisses, startTransMisses;
	//Unused: bool rotSet=false;
	Vector3D<> rotAxis;
	for(std::size_t i=0;i<path.size();i++){
	    const State &state = path[i].getValue();

	    Transform3D<> t = plate->getTransform(state);
	    Vector3D<> p(1.95147, 1.54077, 0.654097);

	    Transform3D<> tStart = plate->getTransform( startPath[i].getValue() );

	    if( MetricUtil::dist2(t.P(),p)<0.10 ){

	        outpath.push_back(rw::trajectory::TimedState(outpath.size(), state) );

	        const State &sstate =  startPath[i].getValue();
	        startoutpath.push_back(rw::trajectory::TimedState(startoutpath.size()-1, sstate) );

	        endTrans.push_back(t);
	        startTrans.push_back(tStart);
	    } else {
            outpathmisses.push_back(rw::trajectory::TimedState(outpath.size(), state) );

            const State &sstate =  startPath[i].getValue();
            startoutpathmisses.push_back(rw::trajectory::TimedState(startoutpath.size()-1, sstate) );


	        endTransMisses.push_back(t);
            startTransMisses.push_back(tStart);

	    }
	}


	PathLoader::storeTimedStatePath(*dwc->getWorkcell(), outpath, "endconfig.rwplay");
	PathLoader::storeTimedStatePath(*dwc->getWorkcell(), startoutpath, "startconfig.rwplay");

    PathLoader::storeTimedStatePath(*dwc->getWorkcell(), outpathmisses, "endconfigmisses.rwplay");
    PathLoader::storeTimedStatePath(*dwc->getWorkcell(), startoutpathmisses, "startconfigmisses.rwplay");


    rotAxis = plate->getTransform(path[2].getValue()).R()*Vector3D<>(0,0,1);
    Rotation3D<> rot = EAA<>(rotAxis, Vector3D<>(0,0,1)).toRotation3D() ;
    Vector3D<> iz = EAA<>(0,90*Deg2Rad,0).toRotation3D()*Vector3D<>(0,0,1);
    std::cout << Rad2Deg*angle(rotAxis, iz) << std::endl;


    //Rotation3D<> rot = EAA<>(0, 90*Deg2Rad, 0).toRotation3D() ;


	saveDist("end_distribution.txt", endTrans, rot);
	saveDist("end_distribution_misses.txt", endTransMisses, rot);
    saveDist("start_distribution.txt", startTrans, rot);
    saveDist("start_distribution_misses.txt", startTransMisses, rot);

	Log::infoLog() << "outpath size: " << outpath.size() << std::endl;
	return 0;
}
