#include "LuaLoaders.hpp"

#include "LuaModels.hpp"
#include "LuaSensor.hpp"
#include "LuaTrajectory.hpp"
#include "LuaProximity.hpp"

using namespace rwlibs::lua;

#include <iostream>
using namespace std;
#include <sstream>

#define LuaNS rwlibs::lua

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

WorkCell LuaNS::loadWorkCell(const std::string& filename){
	rw::models::WorkCellPtr wc = NS::WorkCellLoader::load(filename);
	if(wc==NULL)
		std::cout << "SOMETHING IS WRONG!" << std::endl;
	std::cout << "workcell loadet! " << wc->getName() << std::endl;
	return WorkCell( wc );
}

CollisionSetup LuaNS::loadCollisionSetup(const std::string& filename){
	return  NS::CollisionSetupLoader::load("",filename) ;
}

// Image
Image* LuaNS::loadImage(const std::string& filename){
	rw::sensor::ImagePtr img = NS::ImageFactory::load(filename);
	return new Image(img);
}

void LuaNS::saveAsPGM(Image* img, const std::string& filename){
	// TODO: convert if necesary
	img->get()->saveAsPGM(filename);
}

// paths
void LuaNS::storePath(const QPath& path, const std::string& file){
	NS::PathLoader::storePath(path, file);
}

void LuaNS::storePath(
    const WorkCell& workcell,
    const rw::trajectory::StatePath& path,
    const std::string& file)
{
	NS::PathLoader::storeStatePath(*workcell.get(), path, file);
}

void LuaNS::storePath(
    const WorkCell& workcell,
    const TimedStatePath& path,
    const std::string& file)
{
	NS::PathLoader::storeTimedStatePath(*workcell.get(), path, file);
}

QPath* LuaNS::loadQPath(const std::string& file)
{
	return new QPath( NS::PathLoader::loadPath(file) );
}

StatePath* LuaNS::loadStatePath(
    const WorkCell& workcell,
    const std::string& file)
{
	return new StatePath( *NS::PathLoader::loadStatePath( *workcell.get(), file).release() );
}

TimedStatePath* LuaNS::loadTimedStatePath(
    const WorkCell& workcell,
    const std::string& file)
{
	return new TimedStatePath( *NS::PathLoader::loadTimedStatePath( *workcell.get(), file ).release() );
}





