/*
 * GeometryFactory.hpp
 *
 *  Created on: 10-10-2008
 *      Author: jimali
 */

#ifndef GEOMETRYFACTORY_HPP_
#define GEOMETRYFACTORY_HPP_

#include <rw/common/Cache.hpp>
#include "Geometry.hpp"
#include "GeometryData.hpp"

namespace rw { namespace geometry {

    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Factory for geometric primitive
     *
     * The following primitives are supported
     *
     * GeometryBox:
     * Syntax: "#Box dx dy dz"
     * where "dx, dy, dz" are floats specifying the dimensions
     *
     * GeometryCylinder:
     * Syntax: "#Cylinder radius height level"
     * where "radius" and "height" are float specifying radius and
     * height and "level" is a non-negative integer specifying the
     * discretization level.
     */
    class GeometryFactory
    {
    public:
        /**
         * @brief Factory method for geometric primitive
         *
         * The Factory takes no ownership of the Geometry objects it constructs
         * and thus returns the geometry by auto_ptr.
         *
         * An exception is thrown if the string cannot be parsed correctly.
         *
         * @param str [in] string to parse
         * @return Pointer to a new geometry object
         */
        static Geometry* getGeometry(const std::string& str, bool useCache=true);

    private:
        typedef rw::common::Cache<std::string, GeometryData> Cache;
        static Cache& getCache();

    };

    /* @} */

}} // end namespaces


#endif /* GEOMETRYFACTORY_HPP_ */
