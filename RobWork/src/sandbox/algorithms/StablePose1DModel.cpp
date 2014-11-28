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

#include <cmath>
#include <boost/foreach.hpp>

#include "PlaneModel.hpp"



using namespace std;
using namespace boost;
using namespace rw::math;
using namespace rwlibs::algorithms;
 
 

double StablePose1DModel::fitError(const rw::math::Rotation3D<>& sample) const
{
	// calculate a norm of plane distances
	double dx = 0.0;
	if (!isnan(_dx)) {
		dx = dot(sample.getRow(0), _normal) - _dx;
	}
	
	double dy = 0.0;
	if (!isnan(_dy)) {
		dy = dot(sample.getRow(1), _normal) - _dy;
	}
	
	double dz = 0.0;
	if (!isnan(_dz)) {
		dz = dot(sample.getRow(2), _normal) - _dz;
	}
	
	double error = Vector3D<>(dx, dy, dz).norm2();
	
	//cout << "Fit error for sample " << sample << "= " << error << " " << dx << ", " << dy << ", " << dz << endl;
	
	return error;
}



bool StablePose1DModel::invalid() const
{
	return false;
}



double StablePose1DModel::refit(const std::vector<rw::math::Rotation3D<> >& samples)
{
	const double NormalAlignmentThreshold = 5.0 * Deg2Rad;
	
	_data = samples;
	int n = _data.size();

	/* re-fit point model */
	/* 1. fit x, y, z planes and pick the best one */
	//int idx = 0;
	Vector3D<> normal;
	size_t maxInliers = 0;
	
	cout << _data[0].getRow(0)[2] << " " << _data[0].getRow(1)[2] << " " << _data[0].getRow(2)[2] << endl;
	
	// x plane
	vector<Vector3D<> > xPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		xPoints.push_back(sample.getRow(0));
	}
	PlaneModel xPlane = PlaneModel::bestModel(PlaneModel::findModels(xPoints, 100, 4, 0.25, 1.0));
	cout << "xplane " << xPlane << endl;
	normal = xPlane.normal();
	maxInliers = xPlane.getNumberOfInliers();
	
	// y plane
	vector<Vector3D<> > yPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		yPoints.push_back(sample.getRow(1));
	}
	PlaneModel yPlane = PlaneModel::bestModel(PlaneModel::findModels(yPoints, 100, 4, 0.25, 1.0));
	cout << "yplane " << yPlane << endl;
	if (yPlane.getNumberOfInliers() > maxInliers) {
		normal = yPlane.normal();
		maxInliers = yPlane.getNumberOfInliers();
	}
	
	// z plane
	vector<Vector3D<> > zPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		//cout << sample.getRow(2) << endl;
		zPoints.push_back(sample.getRow(2));
	}
	PlaneModel zPlane = PlaneModel::bestModel(PlaneModel::findModels(zPoints, 100, 4, 0.25, 1.0));
	cout << "zplane " << zPlane << endl;
	if (zPlane.getNumberOfInliers() > maxInliers) {
		normal = zPlane.normal();
		maxInliers = zPlane.getNumberOfInliers();
	}
	
	// TODO: do re-fitting and segmentation of a sphere
	
	/* check if normals are aligned */
	if (angle(xPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dx = dot(xPlane.d() * xPlane.normal(), normal);
	} else {
		_dx = NAN;
	}
	
	if (angle(yPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dy = dot(yPlane.d() * yPlane.normal(), normal);
	} else {
		_dy = NAN;
	}
	
	if (angle(zPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dz = dot(zPlane.d() * zPlane.normal(), normal);
	} else {
		_dz = NAN;
	}
	
	_normal = normal;
	
	// calculate total fit error
	double error = 0.0;
	for (std::vector<rw::math::Rotation3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		double sample_error = fitError(*i);
		error += sample_error * sample_error;
	}

	error /= (n > 0 ? n : 1);
	setQuality(error);
	
	cout << *this << "  | error= " << error << endl;
	
	return error;
}



bool StablePose1DModel::same(const StablePose1DModel& model, double threshold) const
{
	double dx = _dx - model._dx;
	double dy = _dy - model._dy;
	double dz = _dz - model._dz;
	
	if (isnan(dx)) dx = 0.0;
	if (isnan(dy)) dy = 0.0;
	if (isnan(dz)) dz = 0.0;
	
	double d = 0.5 * angle(_normal, model._normal) + 0.5 * Vector3D<>(dx, dy, dz).norm2();
	
	//cout << "Model diff " << *this << " and " << model << " is " << d << " so it's " << (d <= threshold) << endl;
	
	return (d <= threshold);
}
