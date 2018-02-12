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

#ifndef RW_GEOMETRY_BV_HPP_
#define RW_GEOMETRY_BV_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Traits.hpp>

namespace rw{
namespace geometry {
	class GeometryData;
	class Primitive;
	class TriMesh;
	class Shell;

    /**
     * @brief a general bounding volume class for template inheritance. This class
     * defines methods that a deriving class must implement.
     *
     * This defines a bounding volume to have a position.
     */
    template <class DERIVED>
    class BV {
    public:
        typedef DERIVED DerivedType;
        typedef DERIVED BVType;
        typedef typename Traits<DERIVED>::value_type value_type;

        typedef rw::common::Ptr<BV<DERIVED> > Ptr;

        inline const rw::math::Vector3D<value_type>& getPosition() const { return ((DerivedType*) this)->getPosition(); }

        inline value_type calcArea() const { return ((DerivedType*) this)->calcArea(); }

        inline value_type calcVolume() const { return ((DerivedType*) this)->calcVolume(); }

    };

    /**
     * @brief a general oriented bounding volume class
     */
    template <class DERIVED>
    class OBV {
    public:
        typedef DERIVED DerivedType;
        typedef DERIVED BVType;
        typedef typename Traits<DERIVED>::value_type value_type;

        typedef rw::common::Ptr<OBV<DERIVED> > Ptr;

        inline const rw::math::Transform3D<value_type>& getTransform() const { return ((DerivedType*) this)->getTransform(); }

        inline value_type calcArea() const { return ((DerivedType*) this)->calcArea(); }

        inline value_type calcVolume() const { return ((DerivedType*) this)->calcVolume(); }

    };


    /**
     * @brief interface of bounding volume factory
     */
    template<class BV>
    class BVFactory {
    public:
    	//! @brief Smart pointer type to BVFactory<BV>.
        typedef rw::common::Ptr<BVFactory<BV> > Ptr;

        //! @brief Destructor.
        virtual ~BVFactory(){}

        /**
         * @brief Create a bounding volume for a triangle mesh.
         * @param geom [in/out] the mesh to create bounding volume for.
         * @return the bounding volume.
         */
        virtual BV makeBV(rw::geometry::TriMesh& geom) = 0;

        /**
         * @brief Create a bounding volume for any type of geometry.
         * @param geom [in/out] the geometry to create bounding volume for.
         * @return the bounding volume.
         */
        virtual BV makeBV(rw::geometry::GeometryData& geom) = 0;

        /**
         * @brief Create a bounding volume for a primitive.
         * @param geom [in/out] the primitive to create bounding volume for.
         * @return the bounding volume.
         */
        virtual BV makeBV(rw::geometry::Primitive& geom) = 0;

        /**
         * @brief Create a bounding volume for a shell.
         * @param geom [in/out] the shell to create bounding volume for.
         * @return the bounding volume.
         */
        virtual BV makeBV(rw::geometry::Shell& geom) = 0;


        // TODO: at some point a collection of primitives should be supported
    protected:
        //! @brief Constructor.
        BVFactory(){}
    };




}
}




#endif /* BV_HPP_ */
