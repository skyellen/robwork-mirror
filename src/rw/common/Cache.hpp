/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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

	private:
		typedef std::map<KEY, rw::common::Ptr<VAL> > KeyToValMap;
		KeyToValMap _map;
	};

}
}
#endif /*RW_COMMON_CACHE_HPP*/
