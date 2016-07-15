#ifndef CUTACTIONPARAM_HPP
#define CUTACTIONPARAM_HPP

#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Q.hpp>

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
