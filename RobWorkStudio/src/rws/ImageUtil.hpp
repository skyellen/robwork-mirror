/*
 * ImageUtil.hpp
 *
 *  Created on: Jul 17, 2011
 *      Author: jimali
 */

#ifndef RWS_IMAGEUTIL_HPP_
#define RWS_IMAGEUTIL_HPP_

#include <rw/sensor/Image.hpp>
#include <QImage>

namespace rws {

/**
 * utility functions (mostly for conversion) for using thw RobWork Image class in Qt contexts
 */
class ImageUtil {
public:

    static rw::sensor::Image::Ptr toRwImage( const QImage& srcimg );

    static void toRwImage( const QImage& srcimg, rw::sensor::Image& dstimg);

};

}
#endif /* IMAGEUTIL_HPP_ */
