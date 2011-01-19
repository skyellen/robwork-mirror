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


#ifndef RWLIBS_DRAWABLE_DRAWABLEFACTORY_HPP
#define RWLIBS_DRAWABLE_DRAWABLEFACTORY_HPP

/**
 * @file DrawableFactory.hpp
 */

#include "Drawable.hpp"
#include <rw/common/FileCache.hpp>

namespace rwlibs { namespace opengl {

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Factory for construction of drawables based on their type
     */
    class DrawableFactory {
    public:

        /**
         * @brief Factory method for constructing a Drawable based on
         * a string.
         *
         * The method probes the string to see it describes a geometric
         * primitive or a file name. In case of a geometric primitive it
         * forwards to call to DrawableFactory::ConstructDrawableFromGeometry.
         * Otherwise it calls DrawableFactory::LoadDrawableFile
         * otherwise
         */
        static Drawable::Ptr getDrawable(const std::string& str, const std::string& name);

        /**
         * @brief Factory method constructing a Drawable from a file.
         * @param filename [in] path and name of file to load
         * @return drawable
         *
         * The factory determines which type of Drawable to used
         * based on the filename extension. In case no extension
         * exists if test whether a file with the same name or a .stl, .stla, .stlb,
         * .3ds, .ac or .ac3d exists.
         *
         * An exception is thrown if the file can't be loaded.
         */
        static Drawable::Ptr loadDrawableFile(const std::string &filename, const std::string& name);

        /**
         * @brief Factory method constructing a Drawable based on
         * a Geometry ID string.
         *
         * The method constructs a Drawable representing the geometry
         * described in the string
         *
         * An exception is thrown if the string cannot be parsed correctly.
         *
         * @param str [in] Geometry ID string
         * @param useCache [in] True to use caching. Default false
         * @return Point to drawable object
         */
        static Drawable::Ptr constructFromGeometry(
            const std::string& str,
            const std::string& name,
            bool useCache=false);

    private:
        typedef rw::common::FileCache<std::string, Render, std::string> FactoryCache;
    	static FactoryCache& getCache();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
