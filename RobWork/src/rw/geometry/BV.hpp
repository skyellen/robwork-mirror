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
#include <rw/geometry/Primitive.hpp>
#include <rw/geometry/TriMesh.hpp>
#include <rw/geometry/GeometryData.hpp>


namespace rw{
namespace geometry {

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

        inline value_type calcVolumne() const { return ((DerivedType*) this)->calcVolume(); }

    };


    /**
     * @brief interface of bounding volume factory
     */
    template<class BV>
    class BVFactory {
    public:
        typedef rw::common::Ptr<BVFactory<BV> > Ptr;

        virtual ~BVFactory(){};

        //! @brief create a BV
        virtual BV makeBV(rw::geometry::TriMesh& geom) = 0;

        virtual BV makeBV(rw::geometry::GeometryData& geom) = 0;

        virtual BV makeBV(rw::geometry::Primitive& geom) = 0;


        // TODO: at some point a collection of primitives should be supported
    protected:
        BVFactory(){};
    };




}
}




#endif /* BV_HPP_ */
