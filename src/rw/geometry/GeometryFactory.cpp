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


#include "GeometryFactory.hpp"

#include "FaceArrayFactory.hpp"
#include "GeometryBox.hpp"
#include "GeometryCylinder.hpp"
#include "GeometryFace.hpp"

#include <rw/common/Cache.hpp>
#include <rw/common/macros.hpp>
#include <sstream>


using namespace rw::common;
using namespace rw::geometry;

typedef GeometryPtr GeoPtr;

namespace
{
    GeometryPtr constructBox(std::stringstream& sstr)
    {
        float x, y, z;
        sstr >> x >> y >> z;
        return ownedPtr(new GeometryBox(x, y, z));
    }

    GeometryPtr constructCylinder(std::stringstream& sstr)
    {
        float radius, height;
        int divisions;
        if (sstr >> radius >> height >> divisions) {
            if (divisions < 0)
                RW_THROW(
                    "Negative discretization level "
                    << divisions);

            return ownedPtr(new GeometryCylinder(radius, height, divisions));
        } else {
            RW_THROW("Could not read (radius, height, divisions).");
            return GeometryPtr();
        }
    }
}

GeometryPtr GeometryFactory::getGeometry(const std::string& str)
{
    if (str.empty()){
        RW_THROW(
            "String identifier of a geometric "
            "primitive must start with \"#\"");
    }

    if( str[0] != '#' ){
        // assume face array from file
        std::vector<Face<float> > *result = new std::vector<Face<float> >();
        bool res = FaceArrayFactory::getFaceArray(str,*result);
        if(!res)
            return GeometryPtr();
        return ownedPtr(new GeometryFace(str, result));
    }

    std::stringstream sstr(str);
    std::string type;
    sstr >> type;

    if (type == "#Box")
        return constructBox(sstr);
    if (type == "#Cylinder")
        return constructCylinder(sstr);
    else {
        RW_THROW("Unable to construct geometry from string: \"" << str << "\"");

        // To avoid a compiler warning.
        return GeoPtr();
    }
}
