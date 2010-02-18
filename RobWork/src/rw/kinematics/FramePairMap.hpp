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


#ifndef FRAMEPAIRMAP_HPP_
#define FRAMEPAIRMAP_HPP_

#include <rw/kinematics/Frame.hpp>
#include <vector>

#include <map>

namespace rw { namespace kinematics {

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
        void insert(const ConstFramePair& pair, const T& value)
        {
            operator[](pair) = value;
        }

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using non-const operator[]).
        */
        bool has(const ConstFramePair& pair)
        {
            ConstFramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            if( _map.find(p)!=_map.end() )
                return true;
            else
                return false;
        }

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using operator[]).
        */
        bool has(const Frame* f1, const Frame* f2)
        {
            return has(ConstFramePair(f1, f2));
        }

        /**
           @brief return a reference to the value that is associated with the
           frame pair \b pair.

           If no value has been inserted for \b pair, then the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param pair [in] the frame pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        const T& operator[](const ConstFramePair& pair) const
        {
            ConstFramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

        /**
           @brief return a reference to the value that is associated with the
           frame pair consisting of \b f1 and f2.

           If no value has been inserted the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param f1 [in] the first frame in the pair for which to find its associated values.
           @param f2 [in] the first frame in the pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        const T& operator()(const Frame* f1, const Frame* f2) const
        {
            return operator[](ConstFramePair(f1, f2));
        }


        /**
           @brief return a reference to the value that is associated with the
           frame \b frame

           If no value has been inserted for \b frame, then the default value of
           \b T is inserted in the map and returned.

           @param pair [in] the frame pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        T& operator[](const ConstFramePair& pair)
        {
            ConstFramePair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

         /**
           @brief return a reference to the value that is associated with the
           frame pair consisting of \b f1 and f2.

           If no value has been inserted the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param f1 [in] the first frame in the pair for which to find its associated values.
           @param f2 [in] the first frame in the pair for which to find its associated values.
           @return reference to the value associated to frame.
        */
        T& operator()(const Frame* f1, const Frame* f2)
        {
            return operator[](ConstFramePair(f1, f2));
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
        mutable std::map<rw::kinematics::ConstFramePair, T> _map;
    };
}}

#endif /* FRAMEPAIRMAP_HPP_ */
