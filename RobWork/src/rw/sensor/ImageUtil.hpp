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


#ifndef RW_SENSOR_IMAGEUTIL_HPP
#define RW_SENSOR_IMAGEUTIL_HPP

/**
 * @file ImageUtil.hpp
 */

#include <rw/common/Ptr.hpp>

namespace rw { namespace geometry { class PointCloud; } }

namespace rw { namespace sensor {

class Image;

/** @addtogroup sensor */
/*@{*/

/**
 * @brief a collection of simple image utility functions
 */
class ImageUtil {
public:

    /**
     * @brief converts an image of RGB type into an image of
     * GRAY type.
     */
    static void RGB2GRAY(const Image& src, Image& dst);

    /*
     * @brief converts an image of type GRAY into an image of type RGB
     * @param src
     * @param dst
     */
    //static void GRAY2RGB(const Image& src, Image& dst);

    /**
     * @brief sets the value of all channels of an image to
     * \b color.
     */
    static void reset(Image& img, int color=0);

    /**
     * @brief flips the image around the x-axis (horizontal)
     * @param img
     */
    static void flipX(Image& img);

    /**
     * @brief flips the image around the y-axis (vertical)
     * @param img
     */
    static void flipY(Image& img);

    /**
     * @param img
     */
    //static void flipY(const Image& srcimg, Image& dstimg);

    /**
     * convert pointcloud to a depth image. Colors are scaled to min and ax distance of
     * points.
     * @param cloud [in] cloud to convert to image
     * @return image showing the pointcloud as a depth image
     */
    static rw::common::Ptr<rw::sensor::Image> makeDepthImage(const rw::geometry::PointCloud& cloud) ;

    /**
     * convert pointcloud to a depth image. Colors are scaled to min and max distance
     * specified by user
     * @param cloud [in] cloud to convert to image
     * @param min [in] the minimum distance corresponding to black
     * @param max [in] the maximum distance corresponding to white
     * @return image showing the pointcloud as a depth image
     */
    static rw::common::Ptr<rw::sensor::Image> makeDepthImage(const rw::geometry::PointCloud& cloud, float min, float max);
private:
    ImageUtil();
};

/*@}*/

}
}

/// for SDTV not HDTV
/*

friend void RGB24ToYUV422(unsigned char rgb1[3],
unsigned char rgb2[3],
unsigned char yuv[4]){

}

friend void YUV422ToRGB24(unsigned char yuv[4],unsigned char rgb1[3], unsigned char rgb2[3]){
int u,y1,v,y2;
u  = yuv[0];
y1 = yuv[1];
v  = yuv[2];
y2 = yuv[3];

// Conversion
int r = y1 + 1.370705 * v;
int g = y1 - 0.698001 * v - 0.337633 * u;
int b = y1 + 1.732446 * u;

// Clamp to 0..1
if (r < 0) r = 0;
if (g < 0) g = 0;
if (b < 0) b = 0;
if (r > 255) r = 255;
if (g > 255) g = 255;
if (b > 255) b = 255;

rgb1[0] = r&0xFF;
rgb1[1] = g&0xFF;
rgb1[2] = b&0xFF;

// Conversion
r = y2 + 1.370705 * v;
g = y2 - 0.698001 * v - 0.337633 * u;
b = y2 + 1.732446 * u;

// Clamp to 0..1
if (r < 0) r = 0;
if (g < 0) g = 0;
if (b < 0) b = 0;
if (r > 255) r = 255;
if (g > 255) g = 255;
if (b > 255) b = 255;

rgb2[0] = r&0xFF;
rgb2[1] = g&0xFF;
rgb2[2] = b&0xFF;
}

*/

#endif /*RW_SENSOR_IMAGEUTIL_HPP*/
