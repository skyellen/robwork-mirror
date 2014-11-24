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
 
 
 
#include "PlaneConstraint.hpp"

#include <rw/math/Metric.hpp>



using namespace rwlibs::algorithms;
 
 

double PlaneConstraint::fitError(ConstraintSample sample) const
{
	// return a distance of the sample to the model
	return _model.distance(sample.P());
}



bool PlaneConstraint::invalid() const
{
	return false;
}



void PlaneConstraint::refit()
{
	// extract positions from the samples
	std::vector<rw::math::Vector3D<> > points;
	
	for (std::vector<ConstraintSample>::iterator i = _data.begin(); i != _data.end(); ++i) {
		points.push_back(i->P());
	}
	
	// re-fit plane
	_model.refit(points);
}



bool PlaneConstraint::same(const PlaneConstraint& model, double threshold) const
{
	// make a metric to compute distance between planes
	rw::math::Metric<rw::geometry::Plane>::Ptr metric = rw::geometry::Plane::makeMetric();
	
	double d = metric->distance(_model, model._model);
	
	return d <= threshold;
}



void PlaneConstraint::update(ConstraintSample sample)
{
	_data.push_back(sample);
	
	refit();
}
