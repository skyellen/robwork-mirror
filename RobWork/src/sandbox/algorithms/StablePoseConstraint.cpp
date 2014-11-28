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
 
 
 
#include "StablePoseConstraint.hpp"



using namespace rwlibs::algorithms;
 
 
 
double StablePoseConstraint::fitError(rw::math::Transform3D<> sample) const
{
}



bool StablePoseConstraint::invalid() const
{
}



double StablePoseConstraint::refit(const std::vector<rw::math::Transform3D<> >& samples)
{
	// fit a plane to x-values
	_data = samples;
	
	// positions of X, Y, Z points on a unit sphere
	std::vector<rw::math::Vector3D<> > xpoints, ypoints, zpoints;
	for (std::vector<rw::math::Transform3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		xpoints.push_back(i->R().getCol(0));
		ypoints.push_back(i->R().getCol(1));
		zpoints.push_back(i->R().getCol(2));
	}
	
	// fit planes
	_xplane.refit(xpoints);
	_yplane.refit(ypoints);
	_zplane.refit(zpoints);
	
	
}



bool StablePoseConstraint::same(const StablePoseConstraint& model, double threshold) const
{
	return true;
}

