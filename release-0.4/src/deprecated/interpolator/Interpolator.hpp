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


#ifndef rw_interpolator_Interpolator_HPP
#define rw_interpolator_Interpolator_HPP

/**
 * @file Interpolator.hpp
 */

#include <rw/math/Q.hpp>
#include "InterpolatorIterator.hpp"

#include <boost/numeric/ublas/vector.hpp>
#include <vector>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This abstract interpolator class defines a general interpolator interface.
     */
    class Interpolator
    {
    public:

    protected:
        /**
         * @brief Creates empty object.
         */
        Interpolator();

    public:
        /**
         * @brief Destroys object
         */
        virtual ~Interpolator();

        /**
         * @brief returns a configuration Q at some specific discreticed place in the
         * interpolated path.
         * @param d [in] a double that specifies at what place in the path the
         * configuration Q is to be returned. @f$ d\in [0;length] @f$ where length is
         * the complete length of the interpolated path.
         * @return a configuration of a device
         */
        virtual rw::math::Q getX(double d) const = 0;

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length of the interpolated path. The length can be parameterized with an
         * arbitrary size difined by the specialized class.
         * @return the length of the interpolated path
         */
        virtual double getLength() const = 0;

        /**
         * @brief returns the set of viaPoints that this interpolator
         * interpolates over, including start and end point.
         * @return a vector of via points
         */
        virtual std::vector<rw::math::Q> getViaPoints() const = 0;

        /**
         * @brief The array of segments can be used if efficient random access is nessesary.
         * @return vector of pointers to FunctionSegments.
         */
        virtual const std::vector<FunctionSegment*>& getSegments() const = 0;

        /**
         * @brief This function constructs a iterator for a this Interpolator. This Iterator
         * is bi-directional and enables efficient sequential access.
         * @return InterpolatorIterator that can be used to iterate through the
         * interpolated path.
         */        
        virtual InterpolatorIterator getIterator() const {
            return InterpolatorIterator(&getSegments());
        }

        /**
         * @brief returns the first configuration in path
         * @return first configuration in path
         */
        rw::math::Q getFirstX() 
        {
            return getSegments().front()->getX(0);
        }

        /**
         * @brief returns the last configuration in the path
         * @return last configuration in the path
         */
        rw::math::Q getLastX()
        {
            FunctionSegment *segment = getSegments().back();
            return segment->getX(segment->getLength());
        }

    private:
        Interpolator(const Interpolator&);
        Interpolator& operator=(const Interpolator&);
    };
    /*@}*/
}} // end namespaces

#endif // end include guard
