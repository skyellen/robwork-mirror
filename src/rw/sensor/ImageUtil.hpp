/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_SENSOR_IMAGEUTIL_HPP
#define RW_SENSOR_IMAGEUTIL_HPP

#include <rw/common/macros.hpp>
#include "Image.hpp"

namespace rw { namespace sensor {

/**
 * @brief a collection of simple image utility functions
 */
class ImageUtil {
public:

    /**
     * @brief converts an image of RGB24 type into an image of
     * MONO8 type.
     */
    static void convertToGrayscale(const Image& src, Image& dst);

    /**
     *
     */
    static void resetImage(Image& img, int color=0);
private:
    ImageUtil();
};

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
