#include <iostream>
#include <rw/rw.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#include "Gripper.hpp"
#include "GripperXMLLoader.hpp"
#include "JawPrimitive.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace boost::program_options;
namespace po = boost::program_options;



int main(int argc, char* argv[])
{
	// parameters
	int number;
	Q low(10, 	0,	0.01, 	0.001, 	0.001, 	0, 	0, 	0, 		0, 		60, 	0);
	Q high(10, 	1,	0.2, 	0.05, 	0.05, 	1, 	90, 1, 		0.05, 	120, 	0.25);
	
	// program options
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		("number,n", value<int>(&number)->default_value(1), "number of grippers to generate [1]")
		("base,b", value<vector<double> >()->multitoken(), "bases description; 3 parameters should follow")
		("length,l", value<vector<double> >()->multitoken(), "min & max length [0.01; 0.2]")
		("width,w", value<vector<double> >()->multitoken(), "min & max width [0.001; 0.05]")
		("depth,d", value<vector<double> >()->multitoken(), "min & max depth [0.001; 0.05]")
		("chfdepth", value<vector<double> >()->multitoken(), "min & max chamfer depth <0; 1>")
		("chfangle", value<vector<double> >()->multitoken(), "min & max chamfer angle <0; 90>")
		("cutdepth", value<vector<double> >()->multitoken(), "min & max cut depth")
		("cutangle", value<vector<double> >()->multitoken(), "min & max cut angle <0; 180>")
		("cutradius", value<vector<double> >()->multitoken(), "min & max cut radius [0; 0.25]")
		("tcp,t", value<vector<double> >()->multitoken(), "min & max tcp position")
		("opening,o", value<vector<double> >()->multitoken(), "min & max opening")
		("force,o", value<vector<double> >()->multitoken(), "min & max opening")
		("stl", "save base & jaw meshes to STL")
	;
	positional_options_description posDesc;
	posDesc.add("out", -1);
	
	try {
		variables_map vm;
		store(command_line_parser(argc, argv).options(desc).positional(posDesc).run(), vm);
		notify(vm);
		
		/* PROCESS */
		if (vm.count("help")) {
			cout  	<< "This is a script creating gripper XML files database.\n"
					<< "A number of grippers is randomly generated within given constraints.\n" << endl;

			cout << desc << endl;
			return 1;
		}
		
		// setup sampler
		Device::QBox box(low, high);
		QSampler::Ptr jawSampler = QSampler::makeUniform(box);
		
		for (int i = 0; i < number; ++i) {
			Gripper::Ptr gripper = ownedPtr(new Gripper);
			
			stringstream sname;
			sname << "gripper" << i;
			gripper->setName(sname.str());
			
			Q jawParams = jawSampler->sample();
			jawParams(0) = int(jawParams(0));
			jawParams(6) = jawParams(6) * jawParams(1);
			cout << jawParams << endl;
			gripper->setJawGeometry(jawParams);
			
			GripperXMLLoader::save(gripper, ".", gripper->getName()+".grp.xml");
			if (vm.count("stl")) {
				STLFile::save(*gripper->getJawGeometry()->getGeometryData()->getTriMesh(), gripper->getName()+"_jaw.stl");
				STLFile::save(*gripper->getBaseGeometry()->getGeometryData()->getTriMesh(), gripper->getName()+"_base.stl");
			}
		}
		
	} catch (...) {
		cout << "ERROR! Usage:\n" << endl;
		cout << desc << endl;
		return -1;
	}
	
	return 0;
}
