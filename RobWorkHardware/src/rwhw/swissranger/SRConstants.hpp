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

#ifndef RWHW_SWISSRANGER_SRCONSTANTS_HPP
#define RWHW_SWISSRANGER_SRCONSTANTS_HPP

/**
 * @file SRConstants.hpp
 */

/**
 * Namespace providing the common constants for the SwissRanger
 */
namespace rwhw { namespace swissranger {
    /** @addtogroup swissranger */
    /* @{ */

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
    /* @} */

}} // end namespaces

#endif // end include guard
