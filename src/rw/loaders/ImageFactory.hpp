/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef RW_LOADERS_IMAGEFACTORY_HPP
#define RW_LOADERS_IMAGEFACTORY_HPP

/**
 * @file ImageFactory.hpp
 */

#include <string>
#include <memory>

#include <rw/sensor/Image.hpp>

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Loader factory for image files.
     */
    class ImageFactory
    {
    public:
        /**
         * @brief Loads/imports an image from a file.
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] name of image file.
         */
        static rw::sensor::ImagePtr load(const std::string& filename);

    private:
        ImageFactory() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
