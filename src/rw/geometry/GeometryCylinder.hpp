#ifndef RW_GEOMETRY_GEOMETRYCYLINDER_HPP
#define RW_GEOMETRY_GEOMETRYCYLINDER_HPP

#include "Geometry.hpp"



namespace rw {
namespace geometry {
    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Provides a geometric cylinder primitive
     *
     * GeometryCylinder provides a triangulated cylinder primitive. The centre
     * of any cylinder will be \f$(0,0,0\f$. Radius, height and the triangle
     * resolution is defined in the constructor. The height corresponds to 
     * the z-direction.
     */
    class GeometryCylinder: public Geometry {
    public:
        /**
         * @brief Constructs cylinder primitive with the specified setup
         *
         * The cylinder is aligned with the height in the z-direction.
         *
         * @param radius [in] radius of the cylinder.
         * @param height [in] height of the cylinder.
         * @param level [in] the discretization level
         */
        GeometryCylinder(float radius, float height, unsigned int level = 16);

        /**
         * @brief Destructor
         */
        virtual ~GeometryCylinder();
        
        /**
         * @copydoc Geometry::getFaces
         */
        virtual const std::list<Face<float> >& getFaces() const;
    private:
        std::list<Face<float> > _faces;
        

    };
    
    /* @} */

} //end namespace geometry
} //end namespace rw

#endif //#ifndef RW_GEOMETRY_GEOMETRYCUBE_HPP
