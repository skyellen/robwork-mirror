/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <vector>

#include <rw/common/Log.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

using namespace rw::common;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of dynamic workcell input" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);

	Log::infoLog() << "Loading workcell" << std::endl;
	DynamicWorkCell::Ptr dwc = DynamicWorkCellLoader::load(filename);
	Log::infoLog() << "workcell loadet" << std::endl;

	Log::infoLog() << "saving workcell" << std::endl;




	Log::infoLog() << "workcell saved.." << std::endl;
	return 0;
}
