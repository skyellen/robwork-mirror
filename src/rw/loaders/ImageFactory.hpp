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
