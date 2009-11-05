/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


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
