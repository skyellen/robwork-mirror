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


#ifndef RW_SENSOR_IMAGE25D_HPP
#define RW_SENSOR_IMAGE25D_HPP

#include <vector>

#include <rw/math/Vector3D.hpp>
#include <rw/common/Ptr.hpp>
namespace rw {
namespace sensor {

/** @addtogroup sensor */
/* @{ */

/**
 * @brief Data structure for 2.5D range data. The data is saved in
 * structured into an image
 */
class Image25D {

public:
    typedef rw::common::Ptr<Image25D> Ptr;

    /**
     * @brief default constructor
     */
    Image25D():
        _width(0),
        _height(0)
    {};

    /**
     * @brief constructor
     * @param width [in] width of the image
     * @param height [in] height of the image
     */
    Image25D(int width, int height):
        _width(width),
        _height(height),
        _data(width*height)
    {}

    /**
     * @brief destructor
     *
     */
    virtual ~Image25D();

    /**
     * @brief resizes the current image.
     * @param width
     * @param height
     */
    void resize(int width, int height){
        _width = width;
        _height = height;
        _data.resize(_width*_height);
    }

    /**
     * @brief returns a char pointer to the image data
     * @return char pointer to the image data
     */
    std::vector<rw::math::Vector3D<float> >& getData() { return _data; };

    /**
     * @brief returns a char pointer to the image data
     * @return const char pointer to the image data
     */
    const std::vector<rw::math::Vector3D<float> >& getData() const{ return _data; };

    /**
     * @brief returns the width of this image
     * @return image width
     */
    unsigned int getWidth() const { return _width;};

    /**
     * @brief returns the height of this image
     * @return image height
     */
    unsigned int getHeight() const { return _height;};

private:
    unsigned int _width, _height;

protected:
    /**
     * @brief Float array of image data
     */
    std::vector<rw::math::Vector3D<float> > _data;
};

/* @} */

}
}

#endif /*RW_SENSOR_IMAGE3D_HPP_*/
