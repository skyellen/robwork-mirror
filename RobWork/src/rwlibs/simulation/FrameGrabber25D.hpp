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


#ifndef RWLIBS_SIMULATION_FRAMEGRAPPER25D_HPP
#define RWLIBS_SIMULATION_FRAMEGRAPPER25D_HPP

/**
 * @file FrameGrabber.hpp
 */

#include <rw/geometry/PointCloud.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief The FrameGrabber25D abstract interface, can be used to grab images from a
     * specialized source.
     */
    class FrameGrabber25D
    {
    public:
        typedef rw::common::Ptr<FrameGrabber25D> Ptr;

        /**
         * @brief constructor
         * @param width [in] width of the image that this FrameGrabber25D uses.
         * @param height [in] height of the image that this FrameGrabber25D uses.
         */
        FrameGrabber25D(size_t width, size_t height):
            _width(width),
            _height(height)
        {
            _img = new rw::geometry::PointCloud( (int)width, (int)height );
        }

        /**
         * @brief destructor
         */
        virtual ~FrameGrabber25D()
        {
            delete _img;
        }

        /**
         * @brief returns the width of the image
         * @return the height of the image
         */
        size_t getWidth() const { return _img->getWidth(); }

        /**
         * @brief returns the height of the image
         * @return the height of the image
         */
        size_t getHeight() const { return _img->getHeight(); }


        /**
         * @brief Returns the field of view measured around the y-axis.
         * @return Field of view measured around y-axis in radians
         */
        virtual double getFieldOfViewY() = 0;

        /**
         * @brief resizes the image that this frameGrabber use. The colorcode will
         * default to the one that FrameGrabber25D was initialized with.
         * @param width [in] width of image
         * @param height [in] height of image
         */
        void resize(size_t width, size_t height) {
            delete _img;
            _width = width;
            _height = height;
            _img = new rw::geometry::PointCloud((int)width, (int)height);
        };

        /**
         * @brief returns the image
         * @return the image
         */
        virtual rw::geometry::PointCloud& getImage() { return *_img; }


        /**
         * @brief this function grabs a image from the specialized source and
         * copies it to the FrameGrabber25D image.
         */
        //virtual void grab(rw::kinematics::Frame *frame,
        //                  const rw::kinematics::State& state,
        //                  std::vector<rw::math::Vector3D<float> >* result) = 0;


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

    protected:
        //! @brief The image
        rw::geometry::PointCloud *_img;
        size_t _width;
        size_t _height;

    };

    typedef rw::common::Ptr<FrameGrabber25D> FrameGrabber25DPtr;

    /* @} */
}} // end namespaces

#endif // end include guard
