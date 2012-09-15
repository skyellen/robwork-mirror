#include <iostream>

#include <rw/math/Math.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <sandbox/loaders/XercesXML/CollisionSetupLoader.hpp>
#include <rw/loaders/xml/XMLPathFormat.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;


	struct BinContentData {
		int m_PickZoneNumber;
		const std::string m_BinName;
		int m_N;
		BinContentData(const std::string& BinName, int PickZoneNumber, int n):
			m_PickZoneNumber(PickZoneNumber),
			m_BinName(BinName),
			m_N(n)
		{
		
		}

		//BinContentData();
		bool operator< (const BinContentData& rhs) const
		{	
			if (m_BinName == rhs.m_BinName)
				return m_PickZoneNumber > rhs.m_PickZoneNumber;
			return m_BinName>rhs.m_BinName;// && m_PickZoneNumber <= rhs.m_PickZoneNumber;
		}
	};



int main(int argc, char** argv) { 
	{

		std::set<BinContentData> m_BinContentQueue;
		m_BinContentQueue.insert(BinContentData("Bin1", 0, 0));
		m_BinContentQueue.insert(BinContentData("Bin1", 1,0));
		m_BinContentQueue.insert(BinContentData("Bin1", 3,0));
		m_BinContentQueue.insert(BinContentData("Bin1", 4,0));
		m_BinContentQueue.insert(BinContentData("Bin1", 0,1));
		m_BinContentQueue.insert(BinContentData("Bin1", 1,1));
		m_BinContentQueue.insert(BinContentData("Bin1", 3,1));
		m_BinContentQueue.insert(BinContentData("Bin1", 5,0));
		m_BinContentQueue.insert(BinContentData("Bin1", 7,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 0,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 1,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 3,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 4,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 0,1));
		m_BinContentQueue.insert(BinContentData("Bin2", 1,2));
		m_BinContentQueue.insert(BinContentData("Bin2", 3,1));
		m_BinContentQueue.insert(BinContentData("Bin2", 5,0));
		m_BinContentQueue.insert(BinContentData("Bin2", 7,0));

		BOOST_FOREACH(const BinContentData& data, m_BinContentQueue) {
			std::cout<<"Content = "<<data.m_BinName<<" "<<data.m_PickZoneNumber<<" "<<data.m_N<<std::endl;
		}


		return 0;















	Rotation3D<> r3d(0.80596, 0, -0.59197, 0, -1, 0, -0.59197, 0, -0.80596);
	std::cout<<"r3d start = "<<r3d<<std::endl;
	std::cout<<"Det = "<<LinearAlgebra::det(r3d.m())<<std::endl;
	Quaternion<> q(r3d);
	std::cout<<"Q = "<<q<<std::endl;
	std::cout<<"r3d new = "<<q.toRotation3D()<<std::endl;
	Quaternion<> q2(q.toRotation3D());
	std::cout<<"Q 2 = "<<q2<<std::endl;
	
	}

    Transform3D<> Told(rw::math::Vector3D<>(-0.675637, -0.0022987, 0.44), rw::math::Rotation3D<>(1, 0, 0, 0, -1, 0, 0, 0, -1));
    Transform3D<> Tnew(Vector3D<>(-0.594613, -0.140229, 0.44), Rotation3D<>(0.80596, 0, -0.59197, 0, -1 , 0, -0.59197, 0, -0.80596));
    std::cout <<"start " << Told << std::endl;
    std::cout <<"end   " << Tnew << std::endl;


    rw::trajectory::LinearInterpolator<rw::math::Transform3D<> > interpolator(Told, Tnew, 1.0);

    for (double i = 0; i < 1.0; i += 0.1) {
                    Transform3D<> T = interpolator.x(i * 1.0); //v�rdi fra interpolator
                    std::cout <<"inter " << T.P() << std::endl;
                    std::cout <<T.R() << std::endl;
    }

    Transform3D<> Told1(Vector3D<>(-0.597573, -0.13795, 0.415675), Rotation3D<>(0.80596, 0, -0.59197, 0, -1, 0, -0.59197, 0, -0.80596));
    Transform3D<> Tnew1(Vector3D<>(-0.597573, -0.23567, 0.415675), Rotation3D<>(0.80596, 0, -0.59197, 0, -1, 0, -0.59197, 0, -0.80596));

    std::cout <<"\nstart1 " << Told1 << std::endl;
    std::cout <<"end1   " << Tnew1 << std::endl;

    rw::trajectory::LinearInterpolator<rw::math::Transform3D<> > interpolator1(Told1, Tnew1, 1.0);

    for (double i = 0; i < 1.0; i += 0.1) {
                    Transform3D<> T = interpolator1.x(i * 1.0); //v�rdi fra interpolator
                    std::cout <<"inter " << T.P() << std::endl;
                    std::cout <<T.R() << std::endl;
    }












    Rotation3D<> r3d(-0.99999669678603531, -0.0020716226013274717, -0.001521445633434071,
    				 -0.001521761408544605, 0.00015086113440790838, 0.99999883072019702,
    				 0.0018535970374102858, 0.99999827110886175, -0.00014804031424759767);
    Vector3D<> v3d(-0.00046301776312795973,-0.63014974943015156,0.09990255421621469);
    Transform3D<> t3d(v3d,r3d);
	
    Quaternion<> q(r3d);
	std::cout<<"Rot Before = "<<r3d<<std::endl;
	
	std::cout<<"R Det = "<<LinearAlgebra::det(r3d.m())<<std::endl;
	r3d.normalize();
	Quaternion<> qn(r3d);
	std::cout<<"Rot Normalized= "<<r3d<<std::endl;
	std::cout<<"Rot After  = "<<q.toRotation3D()<<std::endl;
	std::cout<<"Rot Normalized After  = "<<qn.toRotation3D()<<std::endl;
	std::cout<<"R Det = "<<LinearAlgebra::det(r3d.m())<<std::endl;
	Quaternion<> q2(q.toRotation3D());
	std::cout<<"q: "<<q<<std::endl;
	std::cout<<"q2:"<<q2<<std::endl;
	std::cout<<"qn: "<<qn<<std::endl;
	return 0;

	std::cout<<"Clearance Performance Test"<<std::endl;
	
	WorkCell::Ptr workcell = WorkCellLoader::load("D:/workspace/PickPlacePlanner/data/WorkCell/WorkCellRoboLab/PilotCellSimple.xml");
	if (workcell == NULL) {
		std::cout<<"Unable to load workcell"<<std::endl;
		return 0;
	}
	Device::Ptr device = workcell->findDevice("KukaKr30ha");
	if (device == NULL) {
		std::cout<<"Unable to find device"<<std::endl;
		return 0;
	}

	std::cout<<XMLPathFormat::TimedQId<<std::endl;


 /*   ProximityPairList strpairs = sandbox::CollisionSetupLoader::load("d:/workspace/PickPlacePlanner/data/WorkCell/ClearanceSetupPilot.xml");
	rw::kinematics::FramePairList clearanceTestFrames;
    for (ProximityPairList::iterator it = strpairs.begin(); it != strpairs.end(); ++it) {
        Frame* frame1 = workcell->findFrame((*it).first);
        Frame* frame2 = workcell->findFrame((*it).second);
		if (frame1 == NULL) 
			RW_THROW2(10001, "No frame named "<<(*it).first<<" found!");
		
		if (CollisionModelInfo::get(frame1).size() == 0)
			std::cout<<"No collision model to frame "<<(*it).first<<std::endl;

        if (frame2 == NULL) 
			RW_THROW2(10003, "No frame named "<<(*it).second<<" found!");
		
		if (CollisionModelInfo::get(frame1).size() == 0)
			std::cout<<"No collision model to frame "<<(*it).second<<std::endl;
		clearanceTestFrames.push_back(std::make_pair(frame1, frame2));
    }
	std::cout<<"Clearance Test Frames = "<<clearanceTestFrames.size()<<std::endl;
 
	rw::common::Ptr<ProximityStrategyPQP> proximityStrategy = ownedPtr(new ProximityStrategyPQP());
	DistanceCalculator::Ptr distanceCalculator = ownedPtr(new DistanceCalculator(clearanceTestFrames, proximityStrategy));

	Math::seed(0);
	QSampler::Ptr sampler = QSampler::makeUniform(device);
	State state = workcell->getDefaultState();
	//For loading the geometries
	distanceCalculator->distance(state);
	distanceCalculator->resetComputationTimeAndCount();
	int collisions = 0;
	double sum = 0;
	for (size_t i = 0; i<1000; i++) {
		Q q = sampler->sample();
		device->setQ(q, state);
		double d = distanceCalculator->distance(state).distance;
		if (d == 0)
			collisions++;
		sum += d;
	}
	std::cout<<"Time (s) = "<<distanceCalculator->getComputationTime()<<std::endl;
	std::cout<<"Avg Time (ms) = "<<1000.*distanceCalculator->getComputationTime() / distanceCalculator->getCount()<<std::endl;
	std::cout<<"Count = "<<distanceCalculator->getCount()<<std::endl;
	std::cout<<"Number of collisions = "<<collisions<<std::endl;
	std::cout<<"Total Clearance Dist = "<<sum<<std::endl;
*/
    


	return 0;
}