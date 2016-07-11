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

#ifndef RWSIM_UTIL_HUGHLINEEXTRACTOR_HPP_
#define RWSIM_UTIL_HUGHLINEEXTRACTOR_HPP_

#include <rw/math/Line2D.hpp>
#include <vector>

namespace rw { namespace sensor { class Image; } }

namespace rwsim {
namespace util {

	class HughLineExtractor {
	public:
		static std::vector<rw::math::Line2D > extractLines(rw::sensor::Image& img);

	};
}
}
#endif /* HUGHLINEEXTRACTOR_HPP_ */
