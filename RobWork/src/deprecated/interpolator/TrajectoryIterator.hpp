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


#ifndef rw_interpolator_PathIterator_HPP
#define rw_interpolator_PathIterator_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include "InterpolatorIterator.hpp"
#include "Interpolator.hpp"

#include <vector>

namespace rw { namespace interpolator {
    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This is a bidirectional iterator class for a Path.
     */
    class TrajectoryIterator
    {
    private:
        const std::vector<interpolator::Interpolator*> *_interpolators;
        unsigned int _currIndex;
        double _currOffset;
        bool _endReached,_beginReached;
        double _length;
        interpolator::InterpolatorIterator _currIIterator;

    public:
        /**
         * @brief Constructor
         *
         * @param interpolators [in] a pointer to a vector of FunctionSegments
         */
        TrajectoryIterator(const std::vector<interpolator::Interpolator*>* interpolators = NULL);

        /**
         * @brief This function can be used to decrease the iterator position.
         * The position can be decreased no longer than to the length 0.
         *
         * @param val [in] a double that describes how much to decrease the
         * iterator position
         */
        void operator -=(double val);

        /**
         * @brief This function can be used to increase the iterator position.
         * The position can be increased no longer than the length og the
         * complete iterated path.
         *
         * @param val [in] a double that describes how much to increase the
         * iterator position
         */
        void operator +=(double val);

        /**
         * @brief used to test if the end of the path is reached.
         *
         * @return true if the iterator has reached the end of the path false
         * otherwise.
         */
        bool isEnd(){ return _endReached;};

        /**
         * @brief used to test if the beginning of the path is reached.
         *
         * @return true if the iterator has reached the beginning of the path
         * false otherwise.
         */
        bool isBegin(){ return _beginReached;};

        /**
         * @brief Extracts a point at the current position in the path.
         *
         * @param iter [in] the path iterator
         *
         * @return the point at the current position in the path.
         */
        friend math::Q operator*(TrajectoryIterator& iter){
            return iter.getX();
        }

        /**
         * @brief Extracts a point at the current position in the path.
         *
         * @return the point at the current position in the path.
         */
        math::Q getX() const {
            return _currIIterator.getX();
        }

        /**
         * @brief Extracts a point of the derivative of the path
         * at the current position in the path
         *
         * @return the derived point at the current position in the path.
         */
        math::Q getXd() const {
            return _currIIterator.getXd();
        }

        /**
         * @brief Extracts a point of the double derivative of the path at the
         * current position in the path
         *
         * @return the double derived point at the current position in the path.
         */
        math::Q getXdd() const {
            return _currIIterator.getXdd();
        }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
