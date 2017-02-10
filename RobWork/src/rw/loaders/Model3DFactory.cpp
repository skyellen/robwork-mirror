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

#include <RobWorkConfig.hpp>

#include "Model3DFactory.hpp"

#include <rw/geometry/PointCloud.hpp>

#include <rw/loaders/model3d/Loader3DS.hpp>
#include <rw/loaders/model3d/LoaderAC3D.hpp>
//#include <rw/graphics/ivg/LoaderIVG.hpp>
#include <rw/loaders/model3d/LoaderOBJ.hpp>
#include <rw/loaders/model3d/LoaderTRI.hpp>
#if RW_HAVE_ASSIMP
#include <rw/loaders/model3d/LoaderAssimp.hpp>
#endif

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <string>
#include <sstream>

#include <sys/stat.h>

using namespace rw;
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::sensor;
using namespace rw::loaders;
namespace
{
    const std::string extensionsArray[] = {
		".TRI", ".AC", ".AC3D", ".3DS", ".OBJ", ".IVG",
#if RW_HAVE_ASSIMP
		".DAE",
#endif
		".STL", ".STLA", ".STLB", ".PCD"
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
        Model3D::Ptr res = ownedPtr( new Model3D( *getCache().get(str) ) );
        res->setName( name );
        return res;
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
    	if (getCache().isInCache(str,"")){
            Model3D::Ptr res = ownedPtr( new Model3D( *getCache().get(str) ) );
            res->setName( name );
            return res;
    	}
    }
	Geometry::Ptr geometry = GeometryFactory::getGeometry(str);
	Model3D *model = new Model3D(name);
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
        Model3D::Ptr res = ownedPtr( new Model3D( *getCache().get(filename) ) );
        res->setName( name );
        return res;
    }

    // if not in cache then create new render
    //std::cout<<"File Type = "<<filetype<<std::endl;
    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
    	// create a geometry

        PlainTriMeshN1F::Ptr data = STLFile::load(filename);
        //STLFile::save(*data,"test_badstl_stuff.stl");

        Model3D *model = new Model3D(name);

        model->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *data);
        model->optimize(30*rw::math::Deg2Rad);

        getCache().add(filename, model, moddate);
        return getCache().get(filename);
    } else if (filetype == ".PCD") {
        rw::geometry::PointCloud::Ptr img = rw::geometry::PointCloud::loadPCD( filename );
        Geometry::Ptr geom = ownedPtr( new Geometry(img) );
        // convert to model3d
        Model3D::Ptr model = ownedPtr( new Model3D(filename) );
        Model3D::Material mat_gray("gray_pcd",0.7f,0.7f,0.7f);
        model->addGeometry( mat_gray , geom );

        getCache().add(filename, model, moddate);
        //std::cout << "Creating drawable!" << std::endl;
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
        Model3D::Ptr model;
#if RW_HAVE_ASSIMP
        LoaderAssimp loader;
        try {
        	model = loader.load(filename);
        } catch(const Exception&) {
        	RW_WARN("Assimp loader for .obj file failed. Trying internal RobWork loader instead.");
#endif
            LoaderOBJ loader;
            model = loader.load(filename);
#if RW_HAVE_ASSIMP
        }
#endif
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
#if RW_HAVE_ASSIMP
    } else if (filetype == ".DAE") {
    	LoaderAssimp loader;
		Model3D::Ptr model = loader.load(filename);
        getCache().add(filename, model, moddate);
        return getCache().get(filename);
#endif
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
