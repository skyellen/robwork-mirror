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

#ifndef RWSIM_UTIL_SUPPORTPOSE_HPP_
#define RWSIM_UTIL_SUPPORTPOSE_HPP_

#include <rw/math/Vector3D.hpp>
#include <vector>

namespace rwsim {
namespace util {


	class SupportPose {
	public:

		SupportPose(rw::math::Vector3D<> rotAxis):
			_degree(1),
			_rotAxes(1,rotAxis),
			_posAxes(1),
			_rotAxesTable(1),
			_probability(-1)
		{};

		SupportPose(int degree, double prob):
			_degree(degree),
			_rotAxes(1),
			_posAxes(1),
			_rotAxesTable(1),
			_probability(prob)
		{};

		virtual ~SupportPose(){};

		// redundant, since length of _rotAxes is also the degree.
		// though its nice to have
		int _degree;
		// invariant rotation axes
		std::vector< rw::math::Vector3D<> > _rotAxes; // relative to own coordinate frame
        // invariant position
		std::vector< rw::math::Vector3D<> > _posAxes; // position of contact relative to own coordinate frame

		std::vector< rw::math::Vector3D<> > _rotAxesTable; // relative to supporting structures frame

		// each rotation axis can be valid in a number of angle intervals
		std::vector< std::vector<std::pair<double,double> > > _segments;

		// the height from

		//rw::math::Transform3D<> _trans;

		// the statistics
		double _probability;
		double _quality;
	};
}
}
#endif
