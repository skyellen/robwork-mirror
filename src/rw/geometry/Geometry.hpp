#ifndef RW_GEOMETRY_GEOMETRY_HPP
#define RW_GEOMETRY_GEOMETRY_HPP


#include "Face.hpp"
#include <list>

namespace rw {
namespace geometry {


    /**
     * @brief Geometry provides an interface for geometries which
     * can be used for visualization and collision detection
     */
    class Geometry {
    public:
        virtual ~Geometry() {}

        /**
         * @brief Returns reference to list of faces
         * 
         * When the Geometry object is delete the reference becomes invalid.
         * If the faces are needed afterwards, then copy the list.
         * 
         * @return Reference to list of faces
         */
        virtual const std::list<Face<float> >& getFaces() const = 0;
        
    };


} //end namespace rw
} //end namespace geometry

#endif //#ifndef RW_GEOMETRY_GEOMETRY_HPP
