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
 
 
 
#include "LineModel.hpp"

#include <rw/math/Metric.hpp>



using namespace rwlibs::algorithms;
 
 

double LineModel::fitError(const rw::math::Vector3D<>& sample) const
{
	// return a distance of the sample to the model
	return _model.distance(sample);
}



bool LineModel::invalid() const
{
	// test whether all the data points are co-linear
	return false;
}



double LineModel::refit(const std::vector<rw::math::Vector3D<> >& samples)
{
	_data = samples;

	// re-fit plane
	_model.refit(_data);
	
	// calculate total fit error
	double error = 0.0;
	for (std::vector<rw::math::Vector3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		double sample_error = fitError(*i);
		error += sample_error * sample_error;
	}
	
	int n = _data.size();
	error /= (n > 0 ? n : 1);
	setQuality(error);
	
	return error;
}



bool LineModel::same(const LineModel& model, double threshold) const
{
	// make a metric to compute distance between planes
	rw::math::Metric<rw::geometry::Line>::Ptr metric = rw::geometry::Line::makeMetric();
	
	double d = metric->distance(_model, model._model);
	//std::cout << "distance= " << d << std::endl;
	
	return d <= threshold;
}
