#include <iostream>
#include <rw/rw.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#include "JawPrimitive.hpp"



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace boost::program_options;
namespace po = boost::program_options;



int main(int argc, char* argv[])
{
	Q params(8, 0.1, 0.025, 0.02, 0, 0*Deg2Rad, 0, 0, 90*Deg2Rad);
	
	options_description desc("Allowed options");
	desc.add_options()
		("help,h", "help message")
		("length,l", value<double>(), "length of gripper")
		("width,w", value<double>(), "width of gripper")
		("depth,d", value<double>(), "depth of gripper")
		("adepth,c", value<double>(), "depth of chamfer")
		("aangle,x", value<double>(), "angle of chamfer")
		("cpos,p", value<double>(), "cut position")
		("cdepth,v", value<double>(), "cut depth")
		("cangle,a", value<double>(), "cut angle")
	;
	
	variables_map vm;
	store(po::parse_command_line(argc, argv, desc), vm);
	notify(vm);
	
	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}
	
	if (vm.count("length")) params[0] = vm["length"].as<double>();
	if (vm.count("width")) params[1] = vm["width"].as<double>();
	if (vm.count("depth")) params[2] = vm["depth"].as<double>();
	if (vm.count("adepth")) params[3] = vm["adepth"].as<double>();
	if (vm.count("aangle")) params[4] = Deg2Rad*vm["aangle"].as<double>();
	if (vm.count("cpos")) params[5] = vm["cpos"].as<double>();
	if (vm.count("cdepth")) params[6] = vm["cdepth"].as<double>();
	if (vm.count("cangle")) params[7] = Deg2Rad*vm["cangle"].as<double>();
	
	//cout << params[1] << endl;
	
	//JawPrimitive::Ptr jaw = ownedPtr(new JawPrimitive(params));
	JawPrimitive jaw(params);
	
	TriMesh::Ptr mesh = jaw.createMesh(0);
	STLFile::save(*mesh, "left.stl");
	STLFile::save(*mesh, "right.stl");
	
	return 0;
}
