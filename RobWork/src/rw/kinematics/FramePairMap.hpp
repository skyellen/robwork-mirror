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


#ifndef FRAMEPAIRMAP_HPP_
#define FRAMEPAIRMAP_HPP_

/**
 * @file FramePairMap.hpp
 */

#include <rw/common/PairMap.hpp>

namespace rw { namespace kinematics {
	class Frame;
/** @addtogroup kinematics */
    /*@{*/

	//! @brief A map from an unordered pair of frames to some value.
    template <class T>
    class FramePairMap: public rw::common::PairMap<const Frame*, T>
    {
    public:
    	//! @copydoc rw::common::PairMap::PairMap
		FramePairMap(int s = 20) :
			 rw::common::PairMap<const Frame*, T>(s)
		{}

		//! @copydoc rw::common::PairMap::PairMap
    	FramePairMap(const T& defaultVal, int s = 20) :
    		 rw::common::PairMap<const Frame*, T>(defaultVal,s)
		{}
    };
    /**@}*/

}}

#endif /* FRAMEPAIRMAP_HPP_ */
