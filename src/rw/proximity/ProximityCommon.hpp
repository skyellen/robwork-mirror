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


#ifndef RW_COLLISION_PROXIMITYCOMMON_HPP
#define RW_COLLISION_PROXIMITYCOMMON_HPP

#include <rw/kinematics/Frame.hpp>

#include <list>
#include <vector>
#include <string>
#include <set>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

    /**
     * @brief A pair of frames
     */
    typedef std::pair<kinematics::Frame*, kinematics::Frame*> FramePair;

	/**
       @brief A set of frames.
    */
	typedef std::set<kinematics::Frame*> FrameSet;

    /**
       @brief A set of frame pairs.
    */
    typedef std::set<FramePair> FramePairSet;

    /**
     * @brief A list of frame pairs
     */
    typedef std::vector<FramePair> FramePairList;

    /**
     * @brief A pair of frame names
     */
    typedef std::pair<std::string, std::string> ProximityPair;

    /**
     * @brief A list of pairs for with ProximityPairs
     */
    typedef std::vector<ProximityPair> ProximityPairList;

	/*@}*/
}} // end namespaces

#endif // end include guard
