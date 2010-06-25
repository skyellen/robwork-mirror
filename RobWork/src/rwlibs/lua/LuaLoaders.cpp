#include "LuaLoaders.hpp"

#include "LuaModels.hpp"
#include "LuaSensor.hpp"
#include "LuaTrajectory.hpp"
#include "LuaProximity.hpp"

using namespace rwlibs::lua;
using namespace rwlibs::lua::loaders;

#include <iostream>
using namespace std;
#include <sstream>

#define LuaNS rwlibs::lua::loaders

#define NS rw::loaders

namespace
{
    string eToString(const rw::common::Exception& e)
    {
        ostringstream buf;
        buf << e.getMessage();
        return buf.str();
    }

    template <typename T>
    string toString(const T& x)
    {
        ostringstream buf;
        buf << x;
        return buf.str();
    }
}

models::WorkCell LuaNS::loadWorkCell(const std::string& filename){
	rw::models::WorkCellPtr wc = NS::WorkCellLoader::load(filename);
	if(wc==NULL)
		std::cout << "SOMETHING IS WRONG!" << std::endl;
	std::cout << "workcell loadet! " << wc->getName() << std::endl;
	return models::WorkCell( wc );
}

proximity::CollisionSetup LuaNS::loadCollisionSetup(const std::string& filename){
	return  NS::CollisionSetupLoader::load("",filename) ;
}

// Image
sensor::Image* LuaNS::loadImage(const std::string& filename){
	rw::sensor::ImagePtr img = NS::ImageFactory::load(filename);
	return new sensor::Image(img);
}

void LuaNS::saveAsPGM(sensor::Image* img, const std::string& filename){
	// TODO: convert if necesary
	img->get()->saveAsPGM(filename);
}

// paths
void LuaNS::storePath(const trajectory::QPath& path, const std::string& file){
	NS::PathLoader::storePath(path, file);
}

void LuaNS::storePath(
    const models::WorkCell& workcell,
    const rw::trajectory::StatePath& path,
    const std::string& file)
{
	NS::PathLoader::storeStatePath(*workcell.get(), path, file);
}

void LuaNS::storePath(
    const models::WorkCell& workcell,
    const trajectory::TimedStatePath& path,
    const std::string& file)
{
	NS::PathLoader::storeTimedStatePath(*workcell.get(), path, file);
}

trajectory::QPath* LuaNS::loadQPath(const std::string& file)
{
	return new trajectory::QPath( NS::PathLoader::loadPath(file) );
}

trajectory::StatePath* LuaNS::loadStatePath(
    const models::WorkCell& workcell,
    const std::string& file)
{
	return new trajectory::StatePath( *NS::PathLoader::loadStatePath( *workcell.get(), file).release() );
}

trajectory::TimedStatePath* LuaNS::loadTimedStatePath(
    const models::WorkCell& workcell,
    const std::string& file)
{
	return new trajectory::TimedStatePath( *NS::PathLoader::loadTimedStatePath( *workcell.get(), file ).release() );
}





