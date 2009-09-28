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

#ifndef RW_COMMON_CACHE_HPP
#define RW_COMMON_CACHE_HPP

#include <boost/shared_ptr.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>
#include <map>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief This class is a template for caching
     */
	template <class KEY, class VAL>
	class Cache
	{
	public:
		/**
		 * @brief default constructor
		 */
		Cache(){};

		/**
		 * @brief default destructor
		 */
		virtual ~Cache(){};

		/**
		 * @brief Tests whether a key is present in the cache
		 */
		bool isInCache(const KEY& id){
			if( _map.find(id) == _map.end() )
				return false;
			return true;
		}

		/**
		 * @brief tests if the key id is in the cache
		 */
		bool has(const KEY& id){
			if( _map.find(id) == _map.end() )
				return false;
			return true;
		}

		/**
		 * @brief gets the value that is associated with the key
		 */
		rw::common::Ptr<VAL> get(const KEY& key){
			if( _map.find(key) == _map.end() )
				RW_THROW("Key does not exist!");
			return _map[key];
		}

		/**
		 * @brief Ads a value to a key that was aquired at some specific
		 * time.
		 */
		void add(const KEY& key, VAL *val){
			_map[key] = Ptr<VAL>(boost::shared_ptr<VAL>( val ));
		}

		/**
		 * @brief remove all values-key pairs that match key
		 */
		void remove(const KEY& key){
			_map.erase(key);
		}

		/**
		 * @brief clear all value-key pairs from this Cache
		 */
		void clear(){
		    _map.clear();
		}

	private:
		typedef std::map<KEY, rw::common::Ptr<VAL> > KeyToValMap;
		KeyToValMap _map;
	};

}
}
#endif /*RW_COMMON_CACHE_HPP*/
