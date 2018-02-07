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

#include <rw/common/Ptr.hpp>

#include <cstddef>

namespace rw {
namespace proximity {


	/**
	 * @brief Interface for cache used by ProximityStrategy
	 */
	class ProximityCache {
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ProximityCache> Ptr;
		/**
		 * @brief Constructor
		 */
		ProximityCache(void *owner):
			_owner(owner)
		{
		}
		/**
		 * @brief Destructor
		 */
		virtual ~ProximityCache(){};

		/**
		 * @brief Returns size of cache
		 * @return size
		 */
		virtual size_t size() const = 0;

		/** 
		 * @brief Clears cache
		 */ 
		virtual void clear() = 0;

		/**
		 * @brief Owner of the cache
		 */
		void *_owner;
	};

#ifdef RW_USE_DEPRECATED
	typedef rw::common::Ptr<ProximityCache> ProximityCachePtr;
#endif
}
}

#endif /* PROXIMITYCACHE_HPP_ */
