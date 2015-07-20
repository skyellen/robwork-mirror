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
 
 
 
#include "PointModel.hpp"

#include <rw/math/Metric.hpp>



using namespace rwlibs::algorithms;
 
 

double PointModel::fitError(const rw::math::Vector3D<>& sample) const
{
	// return a distance of the sample to the model
	return fabs((sample - _model).norm2());
}



bool PointModel::invalid() const
{
	return false;
}



double PointModel::refit(const std::vector<rw::math::Vector3D<> >& samples)
{
	_data = samples;

	// re-fit point model
	int n = _data.size();
	_model = rw::math::Vector3D<>();
	for (int i = 0; i < n; ++i) {
		_model += _data[i];
	}
	_model *= 1.0 / (n > 0 ? n : 1);
	
	// calculate total fit error
	double error = 0.0;
	for (std::vector<rw::math::Vector3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		double sample_error = fitError(*i);
		error += sample_error * sample_error;
	}
	
	error = sqrt(error) / (n > 0 ? n : 1);
	setQuality(error);
	
	return error;
}



bool PointModel::same(const PointModel& model, double threshold) const
{
	double d = (model._model - _model).norm2();
	
	return d <= threshold;
}
