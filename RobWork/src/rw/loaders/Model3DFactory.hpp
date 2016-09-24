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


#ifndef RWLIBS_OPENGL_MODEL3DFACTORY_HPP
#define RWLIBS_OPENGL_MODEL3DFACTORY_HPP

/**
 * @file loaders/Model3DFactory.hpp
 */

#include <rw/graphics/Model3D.hpp>
#include <rw/common/FileCache.hpp>

namespace rw { namespace loaders {

    /** @addtogroup graphics */
    /*@{*/

    /**
     * @brief Factory for construction of drawables based on their type
     */
    class Model3DFactory {
    public:

        /**
         * @brief Factory method for constructing a Drawable based on
         * a string.
         *
         * The method probes the string to see if it describes a geometric
         * primitive or a file name. In case of a geometric primitive it
         * forwards to call to constructFromGeometry.
         * Otherwise it calls loadModel
         * otherwise
         */
        static rw::graphics::Model3D::Ptr getModel(const std::string& str, const std::string& name);

        /**
         * @brief Factory method constructing a Drawable from a file.
         * @param filename [in] path and name of file to load
         * @param name [in] the id/name of the drawable
         * @return drawable
         *
         * The factory determines which type of Drawable to used
         * based on the filename extension. In case no extension
         * exists if test whether a file with the same name or a .stl, .stla, .stlb,
         * .3ds, .ac or .ac3d exists.
         *
         * An exception is thrown if the file can't be loaded.
         */
        static rw::graphics::Model3D::Ptr loadModel(const std::string &filename, const std::string& name);

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
         * @param name [in] the id/name of the drawable
         * @param useCache [in] True to use caching. Default false
         * @return Point to drawable object
         */
        static rw::graphics::Model3D::Ptr constructFromGeometry(const std::string& str, const std::string& name, bool useCache=false);

    private:
        typedef rw::common::FileCache<std::string, rw::graphics::Model3D, std::string> FactoryCache;
    	static FactoryCache& getCache();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
