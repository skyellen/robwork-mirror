
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/sensor.hpp>

#ifndef RWLIBS_LUA_SENSOR_HPP
#define RWLIBS_LUA_SENSOR_HPP


namespace rwlibs {
namespace lua {
namespace sensor {

    class Image
    {
    public:
    	Image(rw::sensor::ImagePtr wc);

    	rw::sensor::ImagePtr get();

    	rw::sensor::ImagePtr _image;
    };

}}}


#endif
