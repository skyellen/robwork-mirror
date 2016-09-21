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


#ifndef RWLIBS_SIMULATION_CAMERA_FRAMEGRAPPER_HPP
#define RWLIBS_SIMULATION_CAMERA_FRAMEGRAPPER_HPP

/**
 * @file FrameGrabber.hpp
 */

#include <rw/sensor/Image.hpp>

namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs { namespace simulation {
    /** @addtogroup simulation */
    /* @{ */

    /**
     * @brief The FrameGrabber abstract interface, can be used to grab images from a
     * specialized source.
     */
    class FrameGrabber
    {
    public:
    	//! @brief Smart pointer type for FrameGrabber.
        typedef rw::common::Ptr<FrameGrabber> Ptr;

        /**
         * @brief constructor
         * @param width [in] width of the image that this FrameGrabber uses.
         * @param height [in] height of the image that this FrameGrabber uses.
         * @param encoding [in] color encoding of the image that this FrameGrabber uses.
         */
        FrameGrabber(int width, int height, rw::sensor::Image::ColorCode encoding) :
            _colorCode(encoding)
        {
            _img = new rw::sensor::Image( width, height, encoding, rw::sensor::Image::Depth8U );
        }

        /**
         * @brief destructor
         */
        virtual ~FrameGrabber()
        {
            delete _img;
        }

        /**
         * @brief returns the width of the image
         * @return the width of the image
         */
        int getWidth() { return _img->getWidth(); }

        /**
         * @brief returns the height of the image
         * @return the height of the image
         */
        int getHeight() { return _img->getHeight(); }

        /**
         * @brief resizes the image that this frameGrabber use. The colorcode will
         * default to the one that FrameGrabber was initialized with.
         * @param width [in] width of image
         * @param height [in] height of image
         */
        virtual void resize(int width, int height) {
            _img->resize(width,height);
        };

        /**
         * @brief resizes the image that this frameGrabber use.
         * @param width [in] width of image.
         * @param height [in] height of image.
         * @param colorCode [in] Color encoding of the image.
         */
        virtual void resize(int width, int height, rw::sensor::Image::ColorCode colorCode)
        {
            _colorCode = colorCode;
            delete _img;
            _img = new rw::sensor::Image(width, height, colorCode, rw::sensor::Image::Depth8U);
        };

        /**
         * @brief returns the image
         * @return the image
         */
        virtual rw::sensor::Image& getImage()
        {
            return *_img;
        }

        /**
         * @brief this function grabs a image from the specialized source and
         * copies it to the FrameGrabber image.
         */
        virtual void grab(rw::kinematics::Frame *frame,
                          const rw::kinematics::State& state) = 0;
    private:
        FrameGrabber(){}
    protected:
        //! @brief The image
        rw::sensor::Image *_img;

        //! @brief Colorcode of the image
        rw::sensor::Image::ColorCode _colorCode;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
