#ifndef RW_GEOMETRY_GEOMETRYCYLINDER_HPP
#define RW_GEOMETRY_GEOMETRYCYLINDER_HPP

#include "Geometry.hpp"

namespace rw { namespace geometry {
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
        GeometryCylinder(float radius, float height, int level = 16);

        /**
         * @brief Destructor
         */
        virtual ~GeometryCylinder();

        /**
         * @copydoc Geometry::getFaces
         */
        virtual const std::vector<Face<float> >& getFaces() const;
    private:
        std::vector<Face<float> > _faces;

    };

    /* @} */

}} // end namespaces

#endif // end include guard
