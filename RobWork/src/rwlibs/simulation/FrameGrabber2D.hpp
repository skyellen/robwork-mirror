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


#ifndef RWLIBS_SIMULATION_FRAMEGRAPPER2D_HPP
#define RWLIBS_SIMULATION_FRAMEGRAPPER2D_HPP

/**
 * @file FrameGrabber.hpp
 */

#include <rw/sensor/Scan2D.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief The FrameGrabber2D abstract interface, can be used to grab images from a
     * specialized source.
     */
    class FrameGrabber2D
    {
    public:
        /**
         * @brief returns the width of the image
         * @return the height of the image
         */
        virtual int getWidth() const = 0;

        /**
         * @brief returns the image
         * @return the image
         */
        virtual rw::sensor::Scan2D& getScan() = 0;

        /**
         * @brief this function grabs a image from the specialized source and
         * copies it to the FrameGrabber25D image.
         */
        virtual void grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state) = 0;

        /**
         * @brief maximum depth that this framegrabber can handle
         * @return maximum depth in meter
         */
        virtual double getMaxDepth() = 0;

        /**
         * @brief minimum depth that this framegrabber can handle
         * @return minimum depth in meter
         */
        virtual double getMinDepth() = 0;


    };

    typedef rw::common::Ptr<FrameGrabber2D> FrameGrabber2DPtr;

    /* @} */
}} // end namespaces

#endif // end include guard
