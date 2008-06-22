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

#ifndef rwlibs_sensors_swissranger_SRConstants_HPP
#define rwlibs_sensors_swissranger_SRConstants_HPP

/**
 * @file SRConstants.hpp
 */

// The doxygen comment has to be put here to take effect.
namespace rwlibs { namespace sensors {
    /**
       Namespace providing the common constants for the SwissRanger
    */
    namespace swissranger {}}
}

namespace rwlibs { namespace sensors { namespace swissranger {
    /**
     * @brief Image width in pixels
     */
    const unsigned int IMG_WIDTH = 160;

    /**
     * @brief Image height in pixels
     */
    const unsigned int IMG_HEIGHT = 124;

    /**
     * @brief Image size in pixels
     */
    const unsigned int IMG_SIZE = IMG_WIDTH*IMG_HEIGHT;

    /**
     * @brief Camera focal length in meter
     */ 
    const double FOCAL_LENGTH = 8e-3;

    /**
     * @brief Camera horizontal field of view in degrees
     */
    const double HORZ_FOV_DEG = 44.7592;

    /**
     * @brief Camera horizontal field of view in radians
     */
    const double HORZ_FOV_RAD = HORZ_FOV_DEG*3.141592653589793116/180;

    /**
     * @brief Width of a single pixel on the chip (in meter)
     */
    const double PIXEL_WIDTH = 3.9e-5;

    /**
     * @brief Height of a single pixel on the chip (in meter)
     */
    const double PIXEL_HEIGHT = 5.4e-5;

}}} // end namespaces

#endif // end include guard
