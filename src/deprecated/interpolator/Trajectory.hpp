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


#ifndef rw_interpolator_InterpolationPath_HPP
#define rw_interpolator_InterpolationPath_HPP

/**
 * @file Trajectory.hpp
 */

#include <vector>

#include "TrajectoryIterator.hpp"
#include "Interpolator.hpp"

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This is a container class for storing a complete interpolated path.
     *
     * The Path consists of zero to multiple optionally different interpolators.
     * The Path object enables the user to use random or sequential access.
     * Though it should be mentioned that Random Access on the path object can
     * be inefficient. For efficient Random Access the segments of each
     * interpolator should be used.
     *
     * @warning The path class does not guarentee that an interpolator A is
     * "connected" or continues in relation to the interpolator B that exist
     * right before or after A.
     */
    class Trajectory 
    {
    private:
        /**
         * @brief A vector of interpolators representing this Path.
         */
        std::vector<interpolator::Interpolator*> _interpolators;

    public:
        /**
         * @brief Constructs an empty path.
         */
        Trajectory(){};

        /**
         * @brief used to check if this path is empty or not
         * @return true if path is empty false if it is not empty
         */
        bool isEmpty(){return _interpolators.empty();};

        /**
         * @brief returns a configuration X at some specific discreticed place in the
         * interpolated path.
         * @param d [in] a double that specifies at what place in the path the
         * configuration X is to be returned. @f$ d\in [0;length] @f$ where length is
         * the complete length of the interpolated path.
         * @return An array value.
         */
        math::Q getX(double d);

        /**
         * @brief Used to get a TrajectoryIterator of this path
         * @return TrajectoryIterator of this path.
         */
        TrajectoryIterator getIterator(){
            return TrajectoryIterator(&_interpolators);
        };

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length all the pathsegments.
         * @return the length of the interpolated path
         */
        double getLength() const {
            double length = 0;
            for(unsigned int i=0; i<_interpolators.size(); i++){
                length += _interpolators[i]->getLength();
            }
            return length;
        };

        /**
         * @brief returns the first configuration in path
         * @return first configuration in path
         */
        math::Q getFirstX(){
            return _interpolators[0]->getFirstX();
        }

        /**
         * @brief returns the last configuration in the path
         * @return last configuration in the path
         */
        math::Q getLastX(){
            return _interpolators[_interpolators.size()-1]->getLastX();

        }

        /**
         * @brief This adds an interpolator to the end of this Path.
         * @param ipath [in] - A pointer to an interpolator.
         */
        void add(interpolator::Interpolator *ipath){
            _interpolators.push_back(ipath);
        };

        /**
         * @brief This inserts an interpolator to the i'th position of the
         * interpolator vector. The current i'th interpolator is pushed forward.
         * @param ipath [in] - A pointer to an interpolator.
         * @param i [in] - The index of where the interpolator is to be inserted.
         */
        void insert(interpolator::Interpolator *ipath, unsigned int i){
            if(i<0 || i>=_interpolators.size())
                return;
            _interpolators.insert(_interpolators.begin()+i,ipath);
        };

        /**
         * @brief This removes an interpolator from the i'th position of the
         * interpolator vector.
         * @param i [in] - The index of where the interpolator is to be removed.
         */
        void remove(unsigned int i){
            if(i<0 || i>=_interpolators.size())
                return;
            _interpolators.erase(_interpolators.begin()+i);
        }

        /**
         * @brief The vector of interpolators is the basis of the Path.
         * @return The vector of pointers to Interpolator objects.
         */
        const std::vector<interpolator::Interpolator*>& getInterpolators(){
            return _interpolators;
        };

    };
    /*@}*/

}} // end namespaces

#endif // end include guard
