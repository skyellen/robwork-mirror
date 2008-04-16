/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rwlibs_drawable_DrawableFactory_HPP
#define rwlibs_drawable_DrawableFactory_HPP

/**
 * @file DrawableFactory.hpp
 */

#include "Drawable.hpp"
//#include "RenderCache.hpp"
#include <rw/common/Cache.hpp>

namespace rwlibs { namespace drawable {

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
        static Drawable* GetDrawable(const std::string& str);

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
        static Drawable* LoadDrawableFile(const std::string &filename);


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
         * @return Point to drawable object
         */
        static Drawable* ConstructFromGeometry(const std::string& str, bool useCache=false);
        
        /**
         * @brief Resets the cached geometries.
         */
        //static void EmptyCache();
        
    private:
    	
    	static rw::common::Cache<std::string, Render>& getCache();
        //static std::map<DrawableData*> _drawableCache;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
