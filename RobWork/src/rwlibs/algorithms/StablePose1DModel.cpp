/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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

#include "StablePose1DModel.hpp"

#include <rw/geometry/Plane.hpp>
#include <rw/math/Math.hpp>

#include <boost/foreach.hpp>

using namespace std;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::algorithms;
 
 

double StablePose1DModel::fitError(const rw::math::Rotation3D<>& sample) const
{
	// calculate a norm of plane distances
	double dx = 0.0;
	if (!Math::isNaN(_dx)) {
		dx = dot(sample.getRow(0), _normal) - _dx;
	}
	
	double dy = 0.0;
	if (!Math::isNaN(_dy)) {
		dy = dot(sample.getRow(1), _normal) - _dy;
	}
	
	double dz = 0.0;
	if (!Math::isNaN(_dz)) {
		dz = dot(sample.getRow(2), _normal) - _dz;
	}
	
	double error = Vector3D<>(dx, dy, dz).norm2();
	
	return error;
}



bool StablePose1DModel::invalid() const
{
	return _invalid;
}



double StablePose1DModel::refit(const std::vector<rw::math::Rotation3D<> >& samples)
{
	_data = samples;
	int n = _data.size();

	// find x, y, z versor positions on the sphere
	vector<Vector3D<> > x_points, y_points, z_points;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		// taking a row of the rotation matrix is the quickest way of finding a versor position on the unit sphere
		// allegedly, this should be columns, but that doesn't work
		x_points.push_back(sample.getRow(0));
		y_points.push_back(sample.getRow(1));
		z_points.push_back(sample.getRow(2));
	}
	
	// fit planes
	Plane x_plane, y_plane, z_plane;
	double x_error = x_plane.refit(x_points);
	double y_error = y_plane.refit(y_points);
	double z_error = z_plane.refit(z_points);
	
	// find the best fitting plane
	Plane* best_plane = NULL;
	double best_error = 1000; //numeric_limits<double>::max();
	
	if (x_error < best_error) {
		best_plane = &x_plane;
		best_error = x_error;
	}
	
	if (y_error < best_error) {
		best_plane = &y_plane;
		best_error = y_error;
	}
	
	if (z_error < best_error) {
		best_plane = &z_plane;
		best_error = z_error;
	}
	
	if (!best_plane) {
		_invalid = true;
		return 0.0;
	} else {
		_invalid = false;
	}
	
	_normal = best_plane->normal();
	
	// calculate distances
	if (best_plane != &x_plane) {
		Vector3D<> c;
		BOOST_FOREACH (const Vector3D<>& p, x_points) {
			c += p;
		}
		c /= x_points.size();
		
		_dx = dot(c, _normal);
	} else {
		_dx = x_plane.d();
	}
	
	if (best_plane != &y_plane) {
		Vector3D<> c;
		BOOST_FOREACH (const Vector3D<>& p, y_points) {
			c += p;
		}
		c /= y_points.size();
		
		_dy = dot(c, _normal);
	} else {
		_dy = y_plane.d();
	}
	
	if (best_plane != &z_plane) {
		Vector3D<> c;
		BOOST_FOREACH (const Vector3D<>& p, z_points) {
			c += p;
		}
		c /= z_points.size();
		
		_dz = dot(c, _normal);
	} else {
		_dz = z_plane.d();
	}
	
	// calculate fitting error
	double error = 0.0;
	for (std::vector<rw::math::Rotation3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		double sample_error = fitError(*i);
		error += sample_error * sample_error;
	}

	error /= (n > 0 ? n : 1);
	setQuality(error);
	
	return error;

	
}



bool StablePose1DModel::same(const StablePose1DModel& model, double threshold) const
{
	double dx = _dx - model._dx;
	double dy = _dy - model._dy;
	double dz = _dz - model._dz;
	
	if (Math::isNaN(dx)) dx = 0.0;
	if (Math::isNaN(dy)) dy = 0.0;
	if (Math::isNaN(dz)) dz = 0.0;
	
	double d = 0.5 * angle(_normal, model._normal) + 0.5 * Vector3D<>(dx, dy, dz).norm2();
	
	return (d <= threshold);
}
