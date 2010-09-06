
/**
 * We need to specify the wrapper classes,
 */
#include "LuaMath.hpp"
#include <rw/sensor.hpp>

#ifndef RWLIBS_LUA_SENSOR_HPP
#define RWLIBS_LUA_SENSOR_HPP


namespace rwlibs {
namespace lua {
    //! @addtogroup lua
    // @{

    /**
     * @brief lua wrapper class for rw::sensor::Image
     */
    class Image
    {
    public:
        //! @brief constructor
    	Image(rw::sensor::ImagePtr wc);

    	//! @brief get the rw::sensor::Image
    	rw::sensor::ImagePtr get();

    	//! @brief the rw::sensor::Image
    	rw::sensor::ImagePtr _image;
    };

    // @}
}}


#endif
