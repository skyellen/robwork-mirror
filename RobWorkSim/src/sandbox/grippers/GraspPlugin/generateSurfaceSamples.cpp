#include <iostream>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include "TaskDescription.hpp"
#include "TaskGenerator.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rw::models;
using namespace rw::loaders;
using namespace rwlibs::task;
using namespace rwsim;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::simulator;
using namespace boost::program_options;
namespace po = boost::program_options;



int main(int argc, char* argv[])
{
	/* initialize stuff */
	Math::seed();
	
	/* parse comand line */
	int number;
	string dwcFilename;
	string tdFilename;
	string outFilename;

	string usage = "This is a script used to generate a database of surface samples which can be used to seed grasp target generation.\n\n"
		"Usage:\n"
		"generate-surface-samples --dwc DWC_FILE --td TD_FILE [-n N_OF_SAMPLES] [-o OUTPUT_FILE]\n";
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		(",n", value<int>(&number)->default_value(1000), "number of samples to generate")
		("dwc", value<string>(&dwcFilename)->required(), "dynamic workcell file")
		("td", value<string>(&tdFilename)->required(), "task description file")
		(",o", value<string>(&outFilename)->default_value("out.samples.xml"), "output file")
	;
	
	variables_map vm;
	try {
		store(command_line_parser(argc, argv).options(desc).run(), vm);
		notify(vm);
		
		if (vm.count("help")) {
			cout << usage << endl;
			cout << desc << endl;
			return 0;
		}
	} catch (...) {
		cout << usage << endl;
		cout << desc << endl;
		return -1;
	}
	
	/* load stuff */
	cout << "* Loading dwc... ";
	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(dwcFilename);
	cout << "Loaded." << endl;
	cout << "* Loading task description... ";
	TaskDescription::Ptr td = TaskDescriptionLoader::load(tdFilename, dwc);
	cout << "Loaded." << endl;
	
	/* prepare sampling */
	TaskGenerator::Ptr generator = new TaskGenerator(td);

	// make sampler
	TriMeshSurfaceSampler sampler(td->getTargetObject()->getGeometry()[0]);
	sampler.setRandomPositionEnabled(false);
	sampler.setRandomRotationEnabled(false);
	
	// make sampling geometry
	CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
	PlainTriMeshF *rayMesh = new PlainTriMeshF(1);
    (*rayMesh)[0] = Triangle<float>(Vector3D<float>(0, (float)-0.001, 0), Vector3D<float>(0, (float)0.001, 0), Vector3D<float>(0, 0, (float)10));
    ProximityModel::Ptr ray = cstrategy->createModel();
    Geometry geom(rayMesh); // we have to wrap the trimesh in an geom object
    geom.setId("Ray");
    ray->addGeometry(geom);
    
    ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), td->getTargetObject()->getGeometry()[0]);
    
    /* generate samples */
    cout << "Generating samples..." << endl;
    vector<SurfaceSample> samples;
    for (int i = 0; i < number; ++i) {
		SurfaceSample ssample = generator->sample(sampler, object, ray, cstrategy);
		samples.push_back(ssample);
	}
	cout << "Done." << endl;
	
	/* save results */
	SurfaceSample::saveToXML(outFilename, samples);
	
	return 0;
}
