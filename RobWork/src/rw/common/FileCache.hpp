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
#ifndef RW_COMMON_FILECACHE_HPP_
#define RW_COMMON_FILECACHE_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>
#include <map>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief a cache that use a timestamp in combination with a key to determine the uniqueness
     * of an item in the cache.
     */
	template<class KEY, class VAL, class STAMP_T>
	class FileCache
	{
	public:
		/**
		 * @brief default constructor
		 */
		FileCache()
		{
		};

		/**
		 * @brief default destructor
		 */
		virtual ~FileCache()
		{
		};

		/**
		 * @brief Tests whether a key is present in the cache
		 */
		bool isInCache(const KEY& id, const STAMP_T& stamp)
		{
			if (_map.find(id) == _map.end() || _keyToStamp.find(id) == _keyToStamp.end())
				return false;
			if( _keyToStamp[id] != stamp )
				return false;
			return true;
		}

		/**
		 * @brief tests if the key id is in the cache
		 */
		bool has(const KEY& id, const STAMP_T& stamp)
		{
			return isInCache(id,stamp);
		}

		/**
		 * @brief gets the value that is associated with the key
		 */
		rw::common::Ptr<VAL> get(const KEY& key)
		{
			if (_map.find(key) == _map.end())
				RW_THROW("Key does not exist!");
			return _map[key];
		}

		/**
		 * @brief Ads a value to a key that was aquired at some specific
		 * time.
		 */
		void add(const KEY& key, VAL *val, const STAMP_T& stamp)
		{
			_keyToStamp[key] = stamp;
			_map[key] = ownedPtr(val);
		}

        /**
         * @brief Ads a value to a key that was aquired at some specific
         * time.
         */
        void add(const KEY& key, rw::common::Ptr<VAL> val, const STAMP_T& stamp)
        {
            _keyToStamp[key] = stamp;
            _map[key] = val;
        }

		/**
		 * @brief remove all values-key pairs that match key
		 */
		void remove(const KEY& key)
		{
			_map.erase(key);
			_keyToStamp.erase(key);
		}

		/**
		 * @brief clear all value-key pairs from this Cache
		 */
		void clear(){
			_map.clear();
			_keyToStamp.clear();
		}


	private:
		typedef std::map<KEY, rw::common::Ptr<VAL> > KeyToValMap;
		KeyToValMap _map;
		std::map<KEY, STAMP_T> _keyToStamp;
	};
	// @}
}
}
#endif /* FILECACHE_HPP_ */
