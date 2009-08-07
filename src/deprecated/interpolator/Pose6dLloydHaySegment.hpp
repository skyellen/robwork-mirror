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


#ifndef rw_interpolator_Pose6dLloydHaySegment_HPP
#define rw_interpolator_Pose6dLloydHaySegment_HPP

/**
 * @file Pose6dLloydHaySegment.hpp
 */

#include "Pose6dStraightSegment.hpp"

namespace rw { namespace interpolator {

    /**
     * @brief Lloyd Hayward interpolator for 6D Cartesian poses
     */
    class Pose6dLloydHaySegment
    {
    private:
        Pose6dStraightSegment _x1,_x2;
        std::pair<double,double> _interval;
        double _k;
        double _tau;

    public:
        /**
         *
         * @brief Constructs a Lloyd Hayward blend between two FunctionSegments.
         * The end of x1 and start of x2 is required to be the same. Also the
         * dimension of x1 should equal the dimension of x2.
         *
         * @param x1 [in] The first of the two segments.
         *
         * @param x2 [in] The second of the two segments.
         *
         * @param k [in] A parameter that controls the amount of acceleration
         * compensation to apply.
         *
         * @param tau [in] The half of the blend time.
         */
        Pose6dLloydHaySegment(
            const Pose6dStraightSegment& x1,
            const Pose6dStraightSegment& x2,
            double k,
            double tau);

        /**
         * @brief copy constructor
         * \param segment [in] segment to copy
         */
        Pose6dLloydHaySegment(const Pose6dLloydHaySegment& segment);

        /**
         * @brief assigment operator
         * @param segment [in] segment to assign
         * @return segment assigned to
         */
        Pose6dLloydHaySegment& operator=(const Pose6dLloydHaySegment& segment);

        /**
         * @brief descructor
         */
        virtual ~Pose6dLloydHaySegment(){};

        /**
         * @copydoc FunctionSegment::getX
         *
         */
        math::Transform3D<> getX(double t) const;

        /**
         * @copydoc FunctionSegment::getXd
         *
         */
        math::VelocityScrew6D<> getXd(double t) const;

        /**
         * @copydoc FunctionSegment::getXdd
         *
         */
        math::Transform3D<> getXdd(double t) const;

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length all the pathsegments.
         * @return the interval in which this function segment is valid.
         */
        virtual std::pair<double,double> getInterval() const {return _interval;};

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length all the pathsegments.
         * @return the length of the interval of this Pose6dFunctionSegment
         */
        virtual double getIntervalLength() const {return _interval.second - _interval.first;};
    };

}} // end namespaces

#endif // end include guard
