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
 
 
 
#include "StablePose0DModel.hpp"

#include <cmath>
#include <algorithm>
#include <boost/foreach.hpp>



using namespace std;
using namespace boost;
using namespace rw::math;
using namespace rwlibs::algorithms;
 
 

double StablePose0DModel::fitError(const rw::math::Rotation3D<>& sample) const
{
	// error is calculated using a standard euclidean metric for x, y & z axes difference	
	Rotation3D<> rot = _rot;
	
	double ax = angle(rot * Vector3D<>::x(), sample * Vector3D<>::x());
	double ay = angle(rot * Vector3D<>::y(), sample * Vector3D<>::y());
	double az = angle(rot * Vector3D<>::z(), sample * Vector3D<>::z());
	
	double error = sqrt(ax*ax + ay*ay + az*az);

	return error;
}



bool StablePose0DModel::invalid() const
{
	return false;
}



double StablePose0DModel::refit(const std::vector<rw::math::Rotation3D<> >& samples)
{
	_data = samples;
	
	size_t n = samples.size();
	
	// refitting is done simply by taking an average of orientations represented as quaternions
	Quaternion<> model(0.0, 0.0, 0.0, 0.0);
	
	// average is calculated using slerp
	double weight = 1.0;
	int i = 0;
	BOOST_FOREACH (const Rotation3D<>& s, samples) {
		++i;
		model = model.slerp(Quaternion<>(s), weight / i);
	}

	_rot = model.toRotation3D();

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



bool StablePose0DModel::same(const StablePose0DModel& model, double threshold) const
{
	double ax = angle(_rot * Vector3D<>::x(), model.orientation() * Vector3D<>::x());
	double ay = angle(_rot * Vector3D<>::y(), model.orientation() * Vector3D<>::y());
	double az = angle(_rot * Vector3D<>::z(), model.orientation() * Vector3D<>::z());
	
	double d = sqrt(ax*ax + ay*ay + az*az);
	
	return (d <= threshold);
}
