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

#include <fstream>
#include <cctype>

#include "Model3DFactory.hpp"

#include <rw/math/Constants.hpp>

#include <rw/graphics/3ds/Loader3DS.hpp>
#include <rw/graphics/ac3d/LoaderAC3D.hpp>
//#include <rw/graphics/ivg/LoaderIVG.hpp>
#include <rw/graphics/obj/LoaderOBJ.hpp>
#include <rw/graphics/tri/LoaderTRI.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/STLFile.hpp>

#include <string>
#include <istream>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h>

using namespace rw;
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;

namespace
{
    const std::string extensionsArray[] = {
        ".TRI", ".AC", ".AC3D", ".3DS", ".OBJ", ".IVG", ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);

    std::string getLastModifiedStr(const std::string& file){
        struct stat status;
        stat(file.c_str(), &status);
        //std::cout << "LAST MODIFIED DATE: " << status.st_mtime << std::endl;
        std::stringstream sstr;
        sstr<< status.st_mtime;
        return sstr.str();
    }
}

Model3D::Ptr Model3DFactory::getModel(const std::string& str, const std::string& name)
{
    if (getCache().isInCache(str,"")) {
    	return getCache().get(str);
    }
    if (str[0] == '#') {
        return constructFromGeometry(str, name);
    }
    else {
        return loadModel(str, name);
    }
}

Model3D::Ptr Model3DFactory::constructFromGeometry(const std::string& str, const std::string& name, bool useCache)
{
    if( useCache ){
    	if (getCache().isInCache(str,""))
    		return getCache().get(str);
    }
	Geometry::Ptr geometry = GeometryFactory::getGeometry(str);
	Model3D *model = new Model3D();
	model->addTriMesh( Model3D::Material("stlmat",0.6f,0.6f,0.6f), *geometry->getGeometryData()->getTriMesh() );

    return ownedPtr( model );
}

Model3DFactory::FactoryCache& Model3DFactory::getCache()
{
    static FactoryCache cache;
	return cache;
}

Model3D::Ptr Model3DFactory::loadModel(const std::string &raw_filename, const std::string& name)
{
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

    std::string moddate = getLastModifiedStr(filename);
    if ( getCache().isInCache(filename, moddate) ) {
    	return getCache().get(filename);
    }

    // if not in cache then create new render
    //std::cout<<"File Type = "<<filetype<<std::endl;
    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
    	// create a geometry

        PlainTriMeshN1F::Ptr data = STLFile::load(filename);
        //STLFile::save(*data,"test_badstl_stuff.stl");

        Model3D *model = new Model3D();

        model->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *data);
        model->optimize(45*rw::math::Deg2Rad);

        getCache().add(filename, model, moddate);
        return getCache().get(filename);
    } else if (filetype == ".3DS") {
    	//std::cout << "loading 3ds file!" << std::endl;
    	Loader3DS loader;
		Model3D::Ptr model = loader.load(filename);
        getCache().add(filename, model, moddate);
        //std::cout << "Creating drawable!" << std::endl;
        return getCache().get(filename);
    } else if (filetype == ".AC" || filetype == ".AC3D") {
    	LoaderAC3D loader;
		Model3D::Ptr model = loader.load(filename);
        getCache().add(filename, model, moddate);
        return getCache().get(filename);
    } else if (filetype == ".TRI") {
    	LoaderTRI loader;
		Model3D::Ptr model = loader.load(filename);
        getCache().add(filename, model, moddate);
        return getCache().get(filename);
    } else if (filetype == ".OBJ") {
    	LoaderOBJ loader;
		Model3D::Ptr model = loader.load(filename);
        getCache().add(filename, model, moddate);
        return getCache().get(filename);
    /*
    } else if (filetype == ".IVG") {
    	LoaderIVG loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    */
	} else {
        RW_THROW(
            "Unknown extension "
            << StringUtil::quote(StringUtil::getFileExtension(filename))
            << " for file "
            << StringUtil::quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}
