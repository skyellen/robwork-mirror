#ifndef RW_GEOMETRY_GEOMETRYFACTORY_HPP
#define RW_GEOMETRY_GEOMETRYFACTORY_HPP

#include <string>

#include "Geometry.hpp"

#include <memory>

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
        static std::auto_ptr<Geometry> getGeometry(const std::string& str);
    };

    /* @} */

}} // end namespaces

#endif //#ifndef RW_GEOMETRY_GEOMETRYFACTORY_HPP
