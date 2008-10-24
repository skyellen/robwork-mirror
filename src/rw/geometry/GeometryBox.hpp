/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#ifndef RW_GEOMETRY_GEOMETRYBOX_HPP
#define RW_GEOMETRY_GEOMETRYBOX_HPP

#include "Geometry.hpp"

namespace rw { namespace geometry {

    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Provides a geometric box primitive
     *
     * GeometryBox provides a triangulated box primitive. The centre
     * of any box will be \f$(0,0,0\f$, it will be aligned to the axis
     * and its dimensions as specified in the constructor.
     */
    class GeometryBox: public Geometry {
    public:
        /**
         * @brief Constructs box with the specified dimensions
         *
         * The box axis aligned and goes from
         * \f$(-\frac{dx}{2},-\frac{dy}{2},-\frac{dz}{2})\f$
         * to \f$(\frac{dx}{2},\frac{dy}{2},\frac{dz}{2})\f$.
         *
         * @param dx [in] x size
         * @param dy [in] y size
         * @param dz [in] z size;
         */
        GeometryBox(float dx, float dy, float dz);

        /**
         * @brief Destructor
         */
        virtual ~GeometryBox();

        /**
         * @copydoc Geometry::getFaces
         */
        virtual const std::vector<Face<float> >& getFaces() const;
    private:
        std::vector<Face<float> > _faces;

    };

    /*@}*/

}} // end namespaces

#endif // end include guard
