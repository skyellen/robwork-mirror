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


#ifndef PAIRMAP_HPP_
#define PAIRMAP_HPP_

/**
 * @file PairMap.hpp
 */

#include <map>

namespace rw { namespace common {

	/** @addtogroup common */
    /*@{*/

    /**
     * @brief a specialized mapping implementation for pairs. It uses the internal
     * structure of template T1 to provide fast O(1) lookup for mappings from a Pair
     * to anything. The order of the Pairs does not matter.
     *
     * @note A requirement is that all pairs must be registered in the same StateStructure.
     */
    template <class T1, class T2>
    class PairMap {
    	typedef std::pair<T1, T1> Pair;

    public:
        /**
         * @brief creates a map
         * @param s [in] the initial size. Default value is 20
         */
        PairMap(int s = 20) :
            _initialSize(s),
            _defaultVal(T2())
        {}

        /**
         * @brief creates a map with an initial size of s
         * @param s [in] nr of elements of the types T with default value "defaultVal"
         * @param defaultVal [in] the default value of new instances of T
         */
        PairMap(const T2& defaultVal, int s = 20) :
            _initialSize(s),
            _defaultVal(defaultVal)
        {}

        /**
         * @brief inserts a value into the map
         * @param pair [in] the pair for which the value is to be associated
         * @param value [in] the value that is to be associated to the pair
         */
        void insert(const Pair& pair, const T2& value)
        {
            operator[](pair) = value;
        }

        /**
           @brief True iff a value for \b frame has been inserted in the map (or
           accessed using non-const operator[]).
        */
        bool has(const Pair& pair)
        {
            Pair p = pair;
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
           @param f1 [in] the first in the pair for which to find its associated values.
           @param f2 [in] the second in the pair for which to find its associated values.
        */
        bool has(const T1 f1, const T1 f2)
        {
            return has(Pair(f1, f2));
        }

        /**
           @brief return a reference to the value that is associated with the
           pair \b pair.

           If no value has been inserted for \b pair, then the default value of
           \b T2 is returned. Use has() to see if a value has been stored for \b
           pair.

           @param pair [in] the pair for which to find its associated values.
           @return reference to the value associated to the pair.
        */
        const T2& operator[](const Pair& pair) const
        {
        	Pair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

        /**
           @brief return a reference to the value that is associated with the
           pair consisting of \b f1 and f2.

           If no value has been inserted the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param f1 [in] the first in the pair for which to find its associated values.
           @param f2 [in] the second in the pair for which to find its associated values.
           @return reference to the value associated to the pair.
        */
        const T2& operator()(T1 f1, T1 f2) const
        {
            return operator[](Pair(f1, f2));
        }

        /**
           @brief return a reference to the value that is associated with the
           \b pair

           If no value has been inserted for \b pair, then the default value of
           \b T2 is inserted in the map and returned.

           @param pair [in] the pair for which to find its associated values.
           @return reference to the value associated to pair.
        */
        T2& operator[](const Pair& pair)
        {
        	Pair p = pair;
            if(p.first>p.second)
                std::swap(p.first,p.second);
            return _map[p];
        }

         /**
           @brief return a reference to the value that is associated with the
           pair consisting of \b f1 and f2.

           If no value has been inserted the default value of
           \b T is returned. Use has() to see if a value has been stored for \b
           frame.

           @param f1 [in] the first frame in the pair for which to find its associated values.
           @param f2 [in] the second frame in the pair for which to find its associated values.
           @return reference to the value associated to pair.
        */
        T2& operator()(T1 f1, T1 f2)
        {
            return operator[](Pair(f1, f2));
        }

        /**
          @brief Erase a pair from the map
          @param pair [in] the pair for which to erase from the map.
         */
        void erase(const Pair& pair)
        {
        	Pair p = pair;
			if(p.first>p.second)
				std::swap(p.first,p.second);
        	_map.erase(p);
        }

        /**
		  @brief Erase a pair from the map
		  @param f1 [in] the first frame in the pair for which to erase from the map.
          @param f2 [in] the second frame in the pair for which to erase from the map.
		 */
		void erase(T1 f1, T1 f2)
		{
			erase(Pair(f1,f2));
		}

        /**
           @brief Clear the map.
        */
        void clear()
        {
            _map.clear();
        }

        /**
		   @brief Return the map size.
		   @return the number of elements in the map.
		*/
		unsigned int size() const
		{
			return _map.size();
		}

        /**
		   @brief Return maximum size.
		   @return the maximum number of elements that the map object can hold.
		*/
		unsigned int max_size() const
		{
			return _map.max_size();
		}

        /**
		   @brief Test whether map is empty.
		   @return whether the map container is empty, i.e. whether its size is 0.
		*/
		bool empty() const
		{
			return _map.empty();
		}

    private:
        int _initialSize;
        T2 _defaultVal;
        mutable std::map<Pair, T2> _map;
    };
    /**@}*/
}}

#endif /* PAIRMAP_HPP_ */
