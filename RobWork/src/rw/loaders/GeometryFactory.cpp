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
#include <rw/math/Transform3D.hpp>

#include <rw/graphics/Model3DFactory.hpp>

#include "model3d/STLFile.hpp"
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/Box.hpp>
#include <rw/geometry/Cone.hpp>
#include <rw/geometry/Cylinder.hpp>
#include <rw/geometry/Tube.hpp>
#include <rw/geometry/Sphere.hpp>
#include <rw/geometry/PointCloud.hpp>
#include <rw/geometry/Pyramid.hpp>

/*
#include "Line.hpp"
#include "Point.hpp"
#include "Plane.hpp"

#include "Triangle.hpp"
*/
#include <boost/foreach.hpp>

using namespace rw::loaders;

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::math;


namespace
{
    const std::string extensionsArray[] = {
        ".STL", ".STLA", ".STLB", ".PCD",".TRI", ".DAE", ".AC", ".AC3D", ".3DS", ".OBJ"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);

	Geometry::Ptr constructBox(std::stringstream& sstr)
	{
		float x, y, z;
		sstr >> x >> y >> z;
		return ownedPtr(new Geometry(ownedPtr(new Box(x, y, z))));
	}

	Geometry::Ptr constructCylinder(std::stringstream& sstr)
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

	Geometry::Ptr constructTube(std::stringstream& sstr)
	{
		float radius, height;
		int divisions;
		if (sstr >> radius >> height >> divisions) {
			if (divisions < 0)
				RW_THROW(
					"Negative discretization level "
					<< divisions);

			return ownedPtr(new Geometry(ownedPtr(new Tube(radius, height))));
		} else {
			RW_THROW("Could not read (radius, height, divisions).");
			return NULL;
		}
	}

	Geometry::Ptr constructSphere(std::stringstream& sstr){
		float radius;
		if (sstr >> radius) {
			return ownedPtr(new Geometry(ownedPtr(new Sphere(radius))));
		} else {
			RW_THROW("Could not read (radius).");
			return NULL;
		}
		return NULL;
	}

	Geometry::Ptr constructCone(std::stringstream& sstr){
		float height, radiusTop;
		if (sstr >> radiusTop >> height) {
			return ownedPtr(new Geometry(ownedPtr(new Cone(height, radiusTop, 0))));
		} else {
			RW_THROW("Could not read (radius, height).");
			return NULL;
		}
		return NULL;
	}

	Geometry::Ptr constructLine(std::stringstream& sstr){
		RW_THROW("Could not read (radius, height, divisions).");
		return NULL;
	}

	Geometry::Ptr constructPyramid(std::stringstream& sstr){
		float dx, dy, height;
		if (sstr >> dx >> dy >> height) {
			return ownedPtr(new Geometry(ownedPtr(new Pyramid(dx, dy, height))));
		} else {
			RW_THROW("Could not read (dx, dy, height).");
			return NULL;
		}
		return NULL;
	}

	Geometry::Ptr constructPlane(std::stringstream& sstr){
		return ownedPtr(new Geometry(ownedPtr(new Plane(Vector3D<>::z(),0))));
	}
}

Geometry::Ptr GeometryFactory::load(const std::string& raw_filename, bool useCache){
	return getGeometry(raw_filename, useCache);
}

Geometry::Ptr GeometryFactory::getGeometry(const std::string& raw_filename, bool useCache){

    if( raw_filename[0] != '#' ){
        std::string filename;
        try{
        	filename = IOUtil::resolveFileName(raw_filename, extensions);
        } catch(...){
        	// try loading a model instead, and create geometry from this

        }


        std::string filetype = StringUtil::toUpper(StringUtil::getFileExtension(filename));

        // if the file does not exist then throw an exception
        if (filetype.empty()) {
            RW_WARN(
                "No file type known for file "
                << StringUtil::quote(raw_filename)
                << " that was resolved to file name "
                << filename << ": defaults to STL!");
            filetype = ".STL";
        }

        if (useCache && getCache().isInCache(filename))
            return ownedPtr( new Geometry(getCache().get(filename)) );


		if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
			GeometryData::Ptr data = STLFile::load(filename);
			if( data == NULL )
				RW_THROW("Reading of geometry failed!");
			getCache().add(filename, data);
			return ownedPtr(new Geometry(getCache().get(filename)));
		} else if( filetype==".PCD" ) {
		    GeometryData::Ptr data = PointCloud::loadPCD(filename);

            if( data == NULL )
                RW_THROW("Reading of geometry failed!");
            getCache().add(filename, data);
            return ownedPtr(new Geometry(getCache().get(filename)));

		} else {
			rw::graphics::Model3D::Ptr model = rw::loaders::Model3DFactory::loadModel(filename,"");

			GeometryData::Ptr data = model->toGeometryData();
			getCache().add(filename, data);
			return ownedPtr(new Geometry(getCache().get(filename)));
		}
    }

    std::stringstream sstr(raw_filename);
    std::string type;
    sstr >> type;

    if (type == "#Plane")
        return constructPlane(sstr);
    if (type == "#Box")
        return constructBox(sstr);
    if (type == "#Cylinder")
        return constructCylinder(sstr);
    if (type == "#Tube")
        return constructTube(sstr);
    if (type == "#Cone")
        return constructCone(sstr);
    if (type == "#Line")
        return constructLine(sstr);
    if (type == "#Sphere")
        return constructSphere(sstr);
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
