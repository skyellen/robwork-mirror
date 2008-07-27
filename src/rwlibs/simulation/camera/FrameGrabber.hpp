/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_sensors_camera_FrameGrabber_HPP
#define rwlibs_sensors_camera_FrameGrabber_HPP

/**
 * @file FrameGrabber.hpp
 */

#include <rw/sensor/Image.hpp>
#include <rw/kinematics/Frame.hpp>

namespace rwlibs { namespace simulation {
    /** @addtogroup sensors */
    /* @{ */

    /**
     * @brief The FrameGrabber abstract interface, can be used to grab images from a
     * specialized source.
     */
    class FrameGrabber
    {
    public:
        /**
         * @brief constructor
         * @param width [in] width of the image that this FrameGrabber uses.
         * @param height [in] height of the image that this FrameGrabber uses.
         * @param encoding [in] color encoding of the image that this FrameGrabber uses.
         */
        FrameGrabber(int width, int height, rw::sensor::Image::ColorCode encoding) :
            _colorCode(encoding)
        {
            _img = new rw::sensor::Image( width, height, encoding);
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
         * @return the height of the image
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
        void resize(int width, int height) {
            //delete _img;
            _img = new rw::sensor::Image(width, height, _colorCode);
        };

        /**
         * @brief resizes the image that this frameGrabber use.
         * @param width [in] width of image.
         * @param height [in] height of image.
         * @param colorCode [in] Color encoding of the image.
         */
        void resize(int width, int height, rw::sensor::Image::ColorCode colorCode)
        {
            _colorCode = colorCode;
            //delete _img; // TODO: shared_ptr
            _img = new rw::sensor::Image(width, height, colorCode);
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

    protected:
        //! @brief The image
        rw::sensor::Image *_img;

        //! @brief Colorcode of the image
        rw::sensor::Image::ColorCode _colorCode;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
