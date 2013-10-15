#ifndef CUTACTIONPARAM_HPP
#define CUTACTIONPARAM_HPP

#include <rw/rw.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/lexical_cast.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/geometry/IntersectUtil.hpp>
#include <rw/proximity/ProximityData.hpp>

#include <iostream>
#include <fstream>
#include <string>



struct CutActionParam {
    // position and orientation of object in world coordinates
	rw::math::Vector3D<> posObj;
	rw::math::RPY<> rpyObj;

	// position and orientation of knife in world coordinates
	rw::math::Vector3D<> pos;
	rw::math::RPY<> rpy;

	// cutting direction in world coordinates
	rw::math::Vector3D<> dir;

    // position and orientation of gripper tcp relative to knife
    rw::math::Vector3D<> posGripper;
    rw::math::RPY<> rpyGripper;
    // configuration of gripper
    rw::math::Q gripperQ;

	// length to cut in the given direction before stopping
	double len;

    static void save(const std::vector<CutActionParam >& params, const std::string& file);

	static std::vector<CutActionParam > load(const std::string& file);
};

#endif
