#ifndef RW_GEOMETRY_GEOMETRYFACE_HPP
#define RW_GEOMETRY_GEOMETRYFACE_HPP

#include "Geometry.hpp"



namespace rw {
namespace geometry {

    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Provides a geometry of a list of faces
     */
    class GeometryFace: public Geometry {
    public:
        /**
         * @brief Constructs box with the specified dimensions
         */
        GeometryFace(const std::string& id, std::vector<Face<float> > *faces):
            Geometry(id),_faces(faces){}

        /**
         * @brief Destructor
         */
        virtual ~GeometryFace(){ delete _faces; }

        /**
         * @copydoc Geometry::getFaces
         */
        virtual const std::vector<Face<float> >& getFaces() const;
    private:
        std::vector<Face<float> > *_faces;


    };

    /*@}*/

} //end namespace geometry
} //end namespace rw

#endif //#ifndef RW_GEOMETRY_GeometryFace_HPP
