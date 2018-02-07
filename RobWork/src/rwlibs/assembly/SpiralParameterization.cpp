/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "SpiralParameterization.hpp"

using namespace rw::common;
using namespace rwlibs::assembly;

SpiralParameterization::SpiralParameterization(PropertyMap::Ptr pmap) {
	reset(pmap);
}

SpiralParameterization::SpiralParameterization(double r, double n, double length_peg, double length_push, double length_start_traj, double depth_in_hole_for_success, double speed, double d_path, double maxAllowedForce):
	r(r), n(n), length_peg(length_peg), length_push(length_push), length_start_traj(length_start_traj), depth_in_hole_for_success(depth_in_hole_for_success), speed(speed), d_path(d_path), maxAllowedForce(maxAllowedForce)
{
}

SpiralParameterization::~SpiralParameterization() {
}

PropertyMap::Ptr SpiralParameterization::toPropertyMap() const {
	PropertyMap::Ptr map = ownedPtr(new PropertyMap());
	if (!map->has("r"))
		map->add("r","Radius (in millimeters).",0.);
	if (!map->has("n"))
		map->add("n","Number of turns. Must be a positive value.",0.);
	if (!map->has("length_peg"))
		map->add("length_peg","The length from peg end to the rotation point (in meters).",0.);
	if (!map->has("length_push"))
		map->add("length_push","The distance the objects should be pushed together (in meters). This requires compliance.",0.);
	if (!map->has("length_start_traj"))
		map->add("length_start_traj","The approach distance between objects (in meters).",0.);
	if (!map->has("depth_in_hole_for_success"))
		map->add("depth_in_hole_for_success","The insertion depth where strategy is considered as succeeded (in meters).",0.);
	if (!map->has("speed"))
		map->add("speed","Speed (in millimeters per second). Must be a positive value.",0.);
	if (!map->has("d_path"))
		map->add("d_path","Discretization of spiral (in radians). Must be a positive value.",0.);
	if (!map->has("maxAllowedForce"))
		map->add("maxAllowedForce","Stop with error if this force is exceeded (in Newton).",0.);
	map->set("r",r*1000.);
	map->set("n",n);
	map->set("length_peg",length_peg*1000.);
	map->set("length_push",length_push*1000.);
	map->set("length_start_traj",length_start_traj*1000.);
	map->set("depth_in_hole_for_success",depth_in_hole_for_success*1000.);
	map->set("speed",speed*1000.);
	map->set("d_path",d_path);
	map->set("maxAllowedForce",maxAllowedForce);
	return map;
}

AssemblyParameterization::Ptr SpiralParameterization::clone() const {
	return ownedPtr(new SpiralParameterization(r,n,length_peg,length_push,length_start_traj,depth_in_hole_for_success,speed,d_path,maxAllowedForce));
}

void SpiralParameterization::reset(PropertyMap::Ptr pmap) {
	AssemblyParameterization::reset(pmap);
	r = pmap->get<double>("r",1.)/1000.;
	n = pmap->get<double>("n",2.);
	length_peg = pmap->get<double>("length_peg",0)/1000.;
	length_push = pmap->get<double>("length_push",0)/1000.;
	length_start_traj = pmap->get<double>("length_start_traj",0)/1000.;
	depth_in_hole_for_success = pmap->get<double>("depth_in_hole_for_success",0)/1000.;
	speed = pmap->get<double>("speed",1.0)/1000.;
	d_path = pmap->get<double>("d_path",0.1);
	maxAllowedForce = pmap->get<double>("maxAllowedForce",10000000);
}
