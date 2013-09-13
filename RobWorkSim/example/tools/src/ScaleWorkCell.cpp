/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <rw/common/Log.hpp>
#include <rw/loaders.hpp>
#include <rw/models.hpp>
#include <sandbox/loaders/RWXMLFile.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;
using namespace boost::numeric;

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of workcell input" << std::endl;
	    std::cout << "- Arg 2 name of workcell ouput" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	std::string outname(argv[2]);

	Log::infoLog() << "Loading workcell" << std::endl;
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(filename);
	Log::infoLog() << "workcell loadet" << std::endl;

	Log::infoLog() << "saving workcell" << std::endl;


	RWXMLFile::saveWorkCell(*wc, wc->getDefaultState(), outname);
	Log::infoLog() << "workcell saved.." << std::endl;
	return 0;
}
