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

#include <rw/common/IOUtil.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/math/Transform3D.hpp>
#include "STLFile.hpp"
#include "Box.hpp"
#include "Cylinder.hpp"
#include "Sphere.hpp"

/*
#include "Line.hpp"
#include "Point.hpp"
#include "Plane.hpp"
#include "Pyramid.hpp"

#include "Triangle.hpp"
*/
#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;

namespace
{
    const std::string extensionsArray[] = {
        /*".TRI", ".AC", ".AC3D", ".3DS", ".IVG",*/ ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);

	GeometryPtr constructBox(std::stringstream& sstr)
	{
		float x, y, z;
		sstr >> x >> y >> z;
		return ownedPtr(new Geometry(ownedPtr(new Box(x, y, z))));
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

			return ownedPtr(new Geometry(ownedPtr(new Cylinder(radius, height))));
		} else {
			RW_THROW("Could not read (radius, height, divisions).");
			return NULL;
		}
	}

	GeometryPtr constructSphere(std::stringstream& sstr){
		float radius;
		if (sstr >> radius) {
			return ownedPtr(new Geometry(ownedPtr(new Sphere(radius))));
		} else {
			RW_THROW("Could not read (radius).");
			return NULL;
		}
		return NULL;
	}

	GeometryPtr constructCone(std::stringstream& sstr){
		RW_THROW("Could not read (radius, height, divisions).");
		return NULL;
	}

	GeometryPtr constructLine(std::stringstream& sstr){
		RW_THROW("Could not read (radius, height, divisions).");
		return NULL;
	}

	GeometryPtr constructPyramid(std::stringstream& sstr){
		RW_THROW("Could not read (radius, height, divisions).");
		return NULL;
	}

	GeometryPtr constructPlane(std::stringstream& sstr){
		RW_THROW("Could not read (radius, height, divisions).");
		return NULL;
	}
}

std::vector<GeometryPtr> GeometryFactory::loadCollisionGeometry(const rw::kinematics::Frame &f){
    std::vector<GeometryPtr> geoms;
    const Frame *frame = &f;
    // std::vector<Face<float> > faces;
    // Log::debug() << "- for all nodes: " << std::endl;
    if( frame==NULL )
        return geoms;
    // check if frame has collision descriptor
    if( !Accessor::collisionModelInfo().has(*frame) )
        return geoms;
    // get the geo descriptor
    std::vector<CollisionModelInfo> infos = Accessor::collisionModelInfo().get(*frame);
    BOOST_FOREACH(CollisionModelInfo &info, infos){
        std::string geofile = info.getId();
        Transform3D<> fTgeo = info.getTransform();
        GeometryPtr geo = GeometryFactory::load(geofile);
        geo->setTransform(fTgeo);
        geoms.push_back(geo);
    }
    return geoms;
}

GeometryPtr GeometryFactory::load(const std::string& raw_filename, bool useCache){
	return getGeometry(raw_filename, useCache);
}

GeometryPtr GeometryFactory::getGeometry(const std::string& raw_filename, bool useCache){

    if( raw_filename[0] != '#' ){
        const std::string& filename = IOUtil::resolveFileName(raw_filename, extensions);
        const std::string& filetype = StringUtil::toUpper(StringUtil::getFileExtension(filename));

        // if the file does not exist then throw an exception
        if (filetype.empty()) {
            RW_THROW(
                "No file type known for file "
                << StringUtil::quote(raw_filename)
                << " that was resolved to file name "
                << filename);
        }

        if (useCache && getCache().isInCache(filename))
            return new Geometry(getCache().get(filename));


		if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
			GeometryDataPtr data = STLFile::load(filename);
			if( data == NULL )
				RW_THROW("Reading of geometry failed!");
			getCache().add(filename, data);
			return ownedPtr(new Geometry(getCache().get(filename)));
		} else {
		    RW_THROW(
				"Unknown extension "
				<< StringUtil::quote(StringUtil::getFileExtension(filename))
				<< " for file "
				<< StringUtil::quote(raw_filename)
				<< " that was resolved to file name "
				<< filename);
		}
    }

    std::stringstream sstr(raw_filename);
    std::string type;
    sstr >> type;

    if (type == "#Box")
        return constructBox(sstr);
    if (type == "#Cylinder")
        return constructCylinder(sstr);
    if (type == "#Cone")
        return constructCone(sstr);
    if (type == "#Line")
        return constructLine(sstr);
    if (type == "#Sphere")
        return constructSphere(sstr);
    if (type == "#Plane")
        return constructPlane(sstr);
    if (type == "#Pyramid")
        return constructPyramid(sstr);
    else {
        RW_THROW("Unable to construct geometry from string: \"" << raw_filename << "\"");
        // To avoid a compiler warning.
        return NULL;
    }

    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}

GeometryFactory::Cache& GeometryFactory::getCache(){
    static Cache cache;
    return cache;
}
