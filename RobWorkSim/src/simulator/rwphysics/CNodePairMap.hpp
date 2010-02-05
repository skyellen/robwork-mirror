#ifndef CNODEPAIRMAP_HPP_
#define CNODEPAIRMAP_HPP_

#include "ConstraintNode.hpp"

#include <vector>

namespace dynamics {

/**
 * This implementation is creates a mapping between CNodePair and
 * some user defined type. Lookup and insertion is O(1).   
 */
template <class T>
class CNodePairMap {
public:
    /**
     * @brief creates a framemap
     */
    CNodePairMap(T defaultVal):
        _map(0),
        _n(0),
        _defaultVal(defaultVal)
    {
        
    }
    
    /**
     * @brief creates a framemap with an initial size of s
     */
    CNodePairMap(int s, T defaultVal):
        _map(s*s+s, defaultVal),
        _n(s),
        _defaultVal(defaultVal)
    {
    }
    
    /**
     * @brief inserts a value
     */
    void insert(const CNodePair& pair, T& value){
        const int idx = pair.second->getID();
        if(idx>=_n)
            resize(idx);
        getValue(pair) = value;
    }
    
    /**
     * @brief returns a reference to the object that pair maps into
     */
    T& get(const CNodePair& pair){
        const int idx = pair.second->getID();
        if(idx>=_n)
            resize(idx);
        return getValue(pair);  
    }    
    
    /**
     * @brief 
     */
    const T& operator[](const CNodePair& pair) const { 
        const int idx = pair.second->getID();
        if(idx>=_n)
            resize(idx);
        return getValue(pair);
    }
    
    /**
     * @brief get the value associated with frame 
     */
    T& operator[](const CNodePair& pair) {
        const int idx = pair.second->getID();
        if(idx>=_n)
            resize(idx);
        return getValue(pair);  
    }
    
private:
    
    void resize(int n) const {
        if(n<_n)
            return;
        _map.resize(n*n + n, _defaultVal);
        _n = n;
    }
    
    T& getValue( const CNodePair& pair ) {
        return _map[ pair.first->getID()*_n+pair.second->getID() ];
    }

    const T& getValue( const CNodePair& pair ) const {
        return _map[ pair.first->getID()*_n+pair.second->getID() ];
    }

    
    // the max id of any node in map
    mutable int _n;
    mutable std::vector<T> _map;
    const T _defaultVal;
};

}

#endif /*FRAMEMAP_HPP_*/
