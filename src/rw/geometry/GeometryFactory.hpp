#ifndef RW_GEOMETRY_GEOMETRYFACTORY_HPP
#define RW_GEOMETRY_GEOMETRYFACTORY_HPP

#include <string>

#include "Geometry.hpp"

namespace rw {
namespace geometry {

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
     * height and "level" is an unsigned integer specifying the 
     * discretization level.
     */ 
    class GeometryFactory {
    public:
        /**
         * @brief Factory method for geometric primitive
         *
         * The Factory takes no ownership of the Geometry objects
         * it constructs.
         *
         * Throws an exception is the string cannot be parsed
         * correctly.
         *
         * @param str [in] string to parse
         * @return Pointer to a new geometry object
         */
        static Geometry* GetGeometry(const std::string& str);
        
        
    };

    /* @} */

} //end namespace geometry
} //end namespace rw

#endif //#ifndef RW_GEOMETRY_GEOMETRYFACTORY_HPP
