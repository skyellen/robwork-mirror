/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#include "ImageFactory.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/loaders/image/RGBLoader.hpp>
//#include <rw/loaders/image/PPMLoader.hpp>
#include <rw/common/macros.hpp>

using namespace rw::loaders;
using namespace rw::common;
using namespace rw::sensor;

rw::sensor::ImagePtr ImageFactory::load(const std::string& file)
{
    const std::string ext = StringUtil::getFileExtension( file);

	if (ext == ".pgm" ){
        return PGMLoader::load( file );
	} else if (ext == ".ppm" ){
	    // return PPMLoader::load( file );
	} else if (ext == ".rgb" ){
	    return RGBLoader::load( file );
	} else if (ext == ".tga" ){
	    //return TGALoader::load( file );
    } else if (ext == ".bmp" ){
        //return BMPLoader::load( file );
    }
	std::cout << "FILE: " << file << std::endl;
	std::cout << "EXT : " << ext << std::endl;
	RW_THROW("Unsupported image type!");
	return NULL;
}
