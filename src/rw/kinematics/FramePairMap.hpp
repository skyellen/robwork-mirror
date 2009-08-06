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

#ifndef FRAMEPAIRMAP_HPP_
#define FRAMEPAIRMAP_HPP_

#include <rw/kinematics/Frame.hpp>
#include <vector>

#include <map>

namespace rw { namespace kinematics {
    typedef std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*> FramePair;
    /**
     * @brief a specialized mapping implementation for framepairs. It uses the internal
     * structure of Frames to provide fast O(1) lookup for mappings from a FramePair
     * to anything.
     *
     * @note A requirement is that all frames must be registered in the same StateStructure.
     */
    template <class T>
    class FramePairMap {
    public:
        /**
         * @brief creates a framemap
         * @param s [in] the initial size. Default value is 20
         */
        FramePairMap(int s = 20) :
            _initialSize(s),
            _defaultVal(T())
            //_map(_initialSize, _defaultVal)
        {}

        /**
         * @brief creates a framemap with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        FramePairMap(const T& defaultVal, int s = 20) :
            _initialSize(s),
            _defaultVal(defaultVal)
            //_map(_initialSize, _defaultVal)
        {}

        /**
         * @brief inserts a value into the frame map
         * @param pair [in] the frame pair for which the value is to be associated
         * @param value [in] the value that is to be associated to the frame
         */
        void insert(const FramePair& pair, const T& value)
        {
            operator[](pair) = value;
        }

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using non-const operator[]).
        */
        bool has(const FramePair& pair)
        {
            FramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            if( _map.find(p)!=_map.end() )
                return true;
            else
                return false;
        }

        /**
           @brief return a reference to the value that is associated with the
           frame \b frame.

           If no value has been inserted for \b frame, then the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param pair [in] the frame pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        const T& operator[](const FramePair& pair) const
        {
            FramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

        /**
           @brief return a reference to the value that is associated with the
           frame \b frame

           If no value has been inserted for \b frame, then the default value of
           \b T is inserted in the map and returned.

           @param pair [in] the frame pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        T& operator[](const FramePair& pair)
        {
            FramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

        /**
           @brief Clear the frame map.
        */
        void clear()
        {
            _map.clear();
            //_map.resize(_initialSize, _defaultVal);
        }

    private:
        int _initialSize;
        T _defaultVal;
        mutable std::map<rw::kinematics::FramePair, T> _map;
    };
}}

#endif /* FRAMEPAIRMAP_HPP_ */
