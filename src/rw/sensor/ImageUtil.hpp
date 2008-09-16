#ifndef RW_SENSOR_IMAGEUTIL_HPP_
#define RW_SENSOR_IMAGEUTIL_HPP_

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
    
private:
    ImageUtil();
};

}
}

#endif /*IMAGEUTIL_HPP_*/
