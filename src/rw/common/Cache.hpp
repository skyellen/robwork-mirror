#ifndef rw_common_Cache_HPP_
#define rw_common_Cache_HPP_

#include <boost/shared_ptr.hpp>

#include <rw/common/macros.hpp>
#include <map>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief 
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
		 * @brief 
		 */
		bool isInCache(const KEY& id){
			if( _map.find(id) == _map.end() )
				return false;
			return true;
		}
		
		/**
		 * @brief 
		 */
		boost::shared_ptr<VAL> get(const KEY& key){
			if( _map.find(key) == _map.end() )
				RW_THROW("Key does not exist!");
			return _map[key];
		}
		
		/**
		 * @brief Ads a value to a key that was aquired at some specific
		 * time. 
		 */
		void add(const KEY& key, VAL *val){
			_map[key] = boost::shared_ptr<VAL>( val );
		}
		
		/**
		 * @brief remove all values-key pairs that match key
		 */
		void remove(const KEY& key){
			_map.erase(key);
		}
		
	private:
		typedef std::map<KEY, boost::shared_ptr<VAL> > KeyToValMap;
		KeyToValMap _map;
	};

}
}
#endif /*Cache_HPP_*/
