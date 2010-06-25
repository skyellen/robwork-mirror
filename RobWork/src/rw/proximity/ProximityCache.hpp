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


#ifndef RW_PROXIMITY_PROXIMITYCACHE_HPP
#define RW_PROXIMITY_PROXIMITYCACHE_HPP

#include "ProximityStrategy.hpp"

namespace rw {
namespace proximity {


	class ProximityCache {
	public:
		ProximityCache(ProximityStrategy *owner):
			_owner(owner)
		{
		}
		virtual ~ProximityCache(){};
		virtual size_t size() const = 0;
		virtual void clear() = 0;

		ProximityStrategy *_owner;
	};

	typedef rw::common::Ptr<ProximityCache> ProximityCachePtr;

}
}

#endif /* PROXIMITYCACHE_HPP_ */
