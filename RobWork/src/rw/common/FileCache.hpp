/*
 * FileCache.hpp
 *
 *  Created on: 24-09-2009
 *      Author: jimali
 */

#ifndef FILECACHE_HPP_
#define FILECACHE_HPP_


#include <boost/shared_ptr.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>
#include <map>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

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
        _map[key] = Ptr<VAL> (boost::shared_ptr<VAL>(val));
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

}
}
#endif /* FILECACHE_HPP_ */
