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
using namespace boost::program_options;
namespace po = boost::program_options;



int main(int argc, char* argv[])
{
	// parameters
	double params[] = { 0, 0.1, 0.025, 0.02, 0, 0*Deg2Rad, 0, 0, 90*Deg2Rad, 0, 0 };
	Q jawParams(11, params);
	Q baseParams(3, 0.15, 0.1, 0.05);
	Q qualities(5, 0.0, 0.0, 0.0, 0.0, 0.0);
	
	// program options
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		("name", value<string>(), "name of the gripper")
		("jaw,j", value<vector<double> >()->multitoken(), "jaw description; 10 parameters should follow")
		("base,b", value<vector<double> >()->multitoken(), "base description; 3 parameters should follow")
		("tcp,t", value<double>(), "TCP offset")
		("jawdist,m", value<double>(), "min. jaws opening")
		("opening,M", value<double>(), "max. jaws opening")
		("stroke,s", value<double>(), "gripper stroke")
		("force,f", value<double>(), "max. gripping force")
		("quality,q", value<vector<double> >()->multitoken(), "quality")
		("out,o", value<string>(), "output file")
		("stl", "save base & jaw meshes to STL")
	;
	positional_options_description posDesc;
	posDesc.add("out", -1);
	
	try {
		cout << "Processing cmdline" << endl;
		
		variables_map vm;
		store(command_line_parser(argc, argv).options(desc).positional(posDesc).style(command_line_style::unix_style ^ command_line_style::allow_short).run(), vm);
		notify(vm);
		
		cout << "Processed" << endl;
		
		/* PROCESS */
		if (vm.count("help")) {
			cout  	<< "This is a script creating gripper XML description file for given parametrization.\n"
					<< "New gripper is saved to [out] if specified or [gripper_name].grp.xml if not.\n"
					<< "If --stl option is specified, base and jaw meshes are saved to [gripper_name]_base.stl "
					<< "and [gripper_name]_jaw.stl, respectively.\n" << endl;

			cout << desc << endl;
			return 1;
		}
		
		cout << "Making gripper" << endl;
		Gripper::Ptr gripper = ownedPtr(new Gripper);
		
		if (vm.count("name")) {
			gripper->setName(vm["name"].as<string>());
		}
		
		if (vm.count("jaw")) {
			cout << "Found jaw" << endl;
			jawParams = Q(vm["jaw"].as<vector<double> >());
			jawParams(5) *= Deg2Rad;
			jawParams(8) *= Deg2Rad;
			jawParams(10) *= Deg2Rad;
			gripper->setJawGeometry(jawParams);
		}
		
		if (vm.count("base")) {
			baseParams = Q(vm["base"].as<vector<double> >());
			gripper->setBaseGeometry(baseParams);
		}
		
		if (vm.count("tcp")) {
			cout << "Found tcp" << endl;
			//gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, gripper->getJawParameters()[1]-vm["tcp"].as<double>())));
			gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, vm["tcp"].as<double>())));
		}
		
		if (vm.count("jawdist")) {
			gripper->setJawdist(vm["jawdist"].as<double>());
		}
		
		if (vm.count("opening")) {
			gripper->setOpening(vm["opening"].as<double>());
		}
		
		if (vm.count("stroke")) {
			double opening = gripper->getOpening();
			gripper->setJawdist(opening - vm["stroke"].as<double>());
		}
		
		if (vm.count("force")) {
			gripper->setForce(vm["force"].as<double>());
		}
		
		if (vm.count("quality")) {
			qualities = Q(vm["quality"].as<vector<double> >());
			gripper->getQuality().success = qualities(0);
			gripper->getQuality().coverage = qualities(1);
			gripper->getQuality().wrench = qualities(2);
			gripper->getQuality().topwrench = qualities(3);
			gripper->getQuality().quality = qualities(4);
		}
		
		if (vm.count("stl")) {
			STLFile::save(*gripper->getJawGeometry()->getGeometryData()->getTriMesh(), gripper->getName()+"_jaw.stl");
			STLFile::save(*gripper->getBaseGeometry()->getGeometryData()->getTriMesh(), gripper->getName()+"_base.stl");
		}
		
		string filename;
		if (vm.count("out")) {
			filename = vm["out"].as<string>();
		} else {
			filename = gripper->getName() + ".grp.xml";
		}
		
		// save gripper
		GripperXMLLoader::save(gripper, filename);
		
	} catch (...) {
		cout << "ERROR" << endl;
		return -1;
	}
	
	return 0;
}
