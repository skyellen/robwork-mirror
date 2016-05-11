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

#include "DrawableFactory.hpp"
#include "RenderGeometry.hpp"
#include "RenderModel3D.hpp"

#include <rw/math/Constants.hpp>


#include <RobWorkConfig.hpp>

#include <rw/loaders/Model3DFactory.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/loaders/model3d/STLFile.hpp>

#include <string>
#include <istream>
#include <sstream>

#include <sys/types.h>
#include <sys/stat.h>

using namespace rw;
using namespace rwlibs::opengl;
using namespace rw::loaders;
using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;

//Due to a name conflict between our Drawable and a Drawable in X11/X.h on Linux, we need a small workaround.
typedef rwlibs::opengl::Drawable::Ptr RWDrawablePtr;
typedef rwlibs::opengl::Drawable RWDrawable;


namespace
{
    const std::string extensionsArray[] = {
        ".TRI", ".AC", ".AC3D", ".3DS", ".OBJ", ".IVG", ".STL", ".STLA", ".STLB", ".DAE", ".DXF"
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

RWDrawablePtr DrawableFactory::getDrawable(const std::string& str, const std::string& name)
{
    if (getCache().isInCache(str,"")) {
    	return ownedPtr( new Drawable(getCache().get(str), name ) );
    }
    if (str[0] == '#') {
        return constructFromGeometry(str, name);
    }
    else {
        return loadDrawableFile(str, name);
    }
}

RWDrawablePtr DrawableFactory::constructFromGeometry(const std::string& str, const std::string& name, bool useCache)
{
    if( useCache ){
    	if (getCache().isInCache(str,""))
    		return ownedPtr(new Drawable(getCache().get(str), name));
    }
	Geometry::Ptr geometry = GeometryFactory::getGeometry(str);
    Render *render = new RenderGeometry(geometry);

    if( useCache ) {
    	getCache().add(str, render, "");
    	return ownedPtr(new Drawable(getCache().get(str), name));
    }

    return ownedPtr(new Drawable( ownedPtr(render), name));
}

DrawableFactory::FactoryCache& DrawableFactory::getCache()
{
    static FactoryCache cache;
	return cache;
}

RWDrawablePtr DrawableFactory::loadDrawableFile(const std::string &raw_filename, const std::string& name)
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
    	return ownedPtr( new Drawable(getCache().get(filename), name) );
    }

    // if not in cache then create new render
    //std::cout<<"File Type = "<<filetype<<std::endl;
    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
    	// create a geometry

        PlainTriMeshN1F::Ptr data = STLFile::load(filename);
        //STLFile::save(*data,"test_badstl_stuff.stl");

        Model3D::Ptr model = ownedPtr(new Model3D(name));

        model->addTriMesh(Model3D::Material("stlmat",0.6f,0.6f,0.6f), *data);

        model->optimize(45*rw::math::Deg2Rad);

        Render *render = new RenderModel3D( model );

        //Geometry::Ptr geom = GeometryFactory::getGeometry(filename);
    	//RenderGeometry *render = new RenderGeometry( geom );

        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable(getCache().get(filename), name) );
    } else {
        Model3D::Ptr model = Model3DFactory::loadModel( filename, name );
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        //std::cout << "Creating drawable!" << std::endl;
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    }
   /*
    if (filetype == ".3DS") {
    	//std::cout << "loading 3ds file!" << std::endl;
    	Loader3DS loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        //std::cout << "Creating drawable!" << std::endl;
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    } else if (filetype == ".AC" || filetype == ".AC3D") {

    	LoaderAC3D loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    } else if (filetype == ".TRI") {
    	LoaderTRI loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    } else if (filetype == ".OBJ") {
    	LoaderOBJ loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    } else if (filetype == ".IVG") {
    	LoaderIVG loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
    */
/* #if RW_HAVE_ASSIMP
    } else if (filetype == ".DAE" || filetype == ".DXF") {
    	LoaderAssimp loader;
		Model3D::Ptr model = loader.load(filename);
        Render *render = new RenderModel3D( model );
        getCache().add(filename, render, moddate);
        return ownedPtr( new Drawable( getCache().get(filename), name ) );
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
*/
    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}
