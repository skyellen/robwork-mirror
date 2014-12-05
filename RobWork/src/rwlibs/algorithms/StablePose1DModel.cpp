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
#include <algorithm>
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
	return _invalid;
}



double StablePose1DModel::refit(const std::vector<rw::math::Rotation3D<> >& samples)
{
	const double NormalAlignmentThreshold = 10.0 * Deg2Rad;
	const int PlaneFitIterations = 100;
	const double PlaneFitThreshold = 0.1;
	const double PlaneModelThreshold = 0.1;
	
	_data = samples;
	int n = _data.size();

	/* re-fit point model */
	/* 1. fit x, y, z planes and pick the best one */
	//int idx = 0; // which plane fit was the best: x, y, or z?
	PlaneModel *bestPlane = NULL; // which plane was the best?
	Vector3D<> normal;
	size_t maxInliers = 0;
	
	//cout << _data[0].getRow(0)[2] << " " << _data[0].getRow(1)[2] << " " << _data[0].getRow(2)[2] << endl;
	
	bool invalidFlag = true;
	
	// x plane
	vector<Vector3D<> > xPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		xPoints.push_back(sample.getRow(0));
	}
	vector<PlaneModel> xModels = PlaneModel::findModels(xPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);	
	PlaneModel xPlane = PlaneModel::likelyModel(xModels);
	//cout << "xplane " << xPlane << endl;
	
	if (xModels.size() > 0) {
		invalidFlag = false;
		normal = xPlane.normal();
		maxInliers = xPlane.getNumberOfInliers();
		bestPlane = &xPlane;
		
		//cout << "x plane is the best " << maxInliers << endl;
	}
	
	// y plane
	vector<Vector3D<> > yPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		yPoints.push_back(sample.getRow(1));
	}
	vector<PlaneModel> yModels = PlaneModel::findModels(yPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);
	PlaneModel yPlane = PlaneModel::likelyModel(yModels);
	//cout << "yplane " << yPlane << endl;
	if (yModels.size() > 0 && yPlane.getNumberOfInliers() > maxInliers) {
		invalidFlag = false;
		normal = yPlane.normal();
		maxInliers = yPlane.getNumberOfInliers();
		bestPlane = &yPlane;
		
		//cout << "y plane is the best " << maxInliers << endl;
	}
	
	// z plane
	vector<Vector3D<> > zPoints;
	BOOST_FOREACH (rw::math::Rotation3D<>& sample, _data) {
		//cout << sample.getRow(2) << endl;
		zPoints.push_back(sample.getRow(2));
	}
	vector<PlaneModel> zModels = PlaneModel::findModels(zPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);
	PlaneModel zPlane = PlaneModel::likelyModel(zModels);
	//cout << "zplane " << zPlane << endl;
	if (zModels.size() > 0 && zPlane.getNumberOfInliers() > maxInliers) {
		invalidFlag = false;
		normal = zPlane.normal();
		maxInliers = zPlane.getNumberOfInliers();
		bestPlane = &zPlane;
		
		//cout << "z plane is the best " << maxInliers << endl;
	}
	
	//cout << "Max inliers= " << maxInliers << endl;
	
	// check if at least one plane found
	if (invalidFlag || !bestPlane) {
		//cout << "invalid because no best plane found" << endl;
		_invalid = true;
		return 0.0;
	}
	
	// TODO: do re-fitting and segmentation of a sphere
	/* re-fit two planes with the least number of inliers, using inliers of the best plane */
	
	invalidFlag = true;
	// x plane
	if (bestPlane != &xPlane) {
		//cout << "refitting x plane" << endl;
		// make new inliers
		xPoints.clear();

		for (size_t i = 0; i < bestPlane->getInlierIndices().size(); ++i) {
			int idx = bestPlane->getInlierIndices()[i];
			xPoints.push_back(_data[idx].getRow(0));
		}
		
		//cout << "XPoints size=" << xPoints.size() << endl;
		xModels = PlaneModel::findModels(xPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);
		xPlane = PlaneModel::likelyModel(xModels);
		
		if (xModels.size() > 0) {
			invalidFlag = false;
		}
	}
	
	// y plane
	if (bestPlane != &yPlane) {
		//cout << "refitting y plane" << endl;
		// make new inliers
		yPoints.clear();

		for (size_t i = 0; i < bestPlane->getInlierIndices().size(); ++i) {
			int idx = bestPlane->getInlierIndices()[i];
			yPoints.push_back(_data[idx].getRow(1));
		}
		
		//cout << "YPoints size=" << yPoints.size() << endl;
		yModels = PlaneModel::findModels(yPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);
		yPlane = PlaneModel::likelyModel(yModels);
		
		if (yModels.size() > 0) {
			invalidFlag = false;
		}
	}
	
	// z plane
	if (bestPlane != &zPlane) {
		//cout << "refitting z plane" << endl;
		// make new inliers
		zPoints.clear();

		for (size_t i = 0; i < bestPlane->getInlierIndices().size(); ++i) {
			int idx = bestPlane->getInlierIndices()[i];
			zPoints.push_back(_data[idx].getRow(2));
		}
		
		//cout << "ZPoints size=" << zPoints.size() << endl;
		zModels = PlaneModel::findModels(zPoints, PlaneFitIterations, 4, PlaneFitThreshold, PlaneModelThreshold);
		zPlane = PlaneModel::likelyModel(zModels);
	}
	
	// check if at least one plane found
	if (invalidFlag) {
		//cout << "invalid because no other plane found" << endl;
		_invalid = true;
		return 0.0;
	}
	
	/* check if normals are aligned */
	size_t nan_counter = 0;
	if (angle(xPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dx = dot(xPlane.d() * xPlane.normal(), normal);
	} else {
		_dx = NAN;
		++nan_counter;
	}
	
	if (angle(yPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dy = dot(yPlane.d() * yPlane.normal(), normal);
	} else {
		_dy = NAN;
		++nan_counter;
	}
	
	if (angle(zPlane.normal(), normal) <= NormalAlignmentThreshold) {
		_dz = dot(zPlane.d() * zPlane.normal(), normal);
	} else {
		_dz = NAN;
		++nan_counter;
	}
	
	// if two of the normals are not aligned, it is not a good model
	if (nan_counter >= 2) {
		//cout << "invalid because normals were unaligned" << endl;
		_invalid = true;
		
		return 0.0;
	} else {
		_invalid = false;
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
	
	//cout << *this << "  | error= " << error << endl;
	
	return error;
}



bool StablePose1DModel::same(const StablePose1DModel& model, double threshold) const
{
	//*
	// naive approach
	double dx = _dx - model._dx;
	double dy = _dy - model._dy;
	double dz = _dz - model._dz;
	
	if (isnan(dx)) dx = 0.0;
	if (isnan(dy)) dy = 0.0;
	if (isnan(dz)) dz = 0.0;
	
	double d = 0.5 * angle(_normal, model._normal) + 0.5 * Vector3D<>(dx, dy, dz).norm2();
	
	//cout << "model dist= " << d << endl;
	
	/*/
	// how many inliers of one model belong to another
	const StablePose1DModel* better;
	const StablePose1DModel* worse;
	
	if (getNumberOfInliers() > model.getNumberOfInliers()) {
		better = this;
		worse = &model;
	} else {
		better = &model;
		worse = this;
	}
	
	int n = 0; // counts how many inliers of worse model belong to better model as well
	BOOST_FOREACH (const Rotation3D<>& r, worse->getData()) {
		const vector<Rotation3D<> >& data = better->getData();
		if (find(data.begin(), data.end(), r) != data.end()) {
			++n;
		}
	}
	
	double d = 1.0 - 1.0 * n / worse->getNumberOfInliers();
	//*/
	
	//cout << "Model distance is " << d  << " = " << n << " / " << worse->getNumberOfInliers() << endl;
	
	return (d <= threshold);
}
