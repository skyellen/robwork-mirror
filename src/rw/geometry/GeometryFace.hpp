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
