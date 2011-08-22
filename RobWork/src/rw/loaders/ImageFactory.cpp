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


#include "ImageFactory.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/loaders/image/RGBLoader.hpp>
#include <rw/loaders/image/PPMLoader.hpp>
#include <rw/loaders/image/ImageLoader.hpp>
#include <rw/common/macros.hpp>

#include <rw/plugin/PluginRepository.hpp>
#include <rw/plugin/PluginFactory.hpp>
#include <rw/RobWork.hpp>

#include <boost/foreach.hpp>

using namespace rw::loaders;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::plugin;

rw::sensor::Image::Ptr ImageFactory::load(const std::string& file)
{
    const std::string ext2 = StringUtil::getFileExtension( file);
    const std::string ext = StringUtil::toUpper(ext2);
	if (ext == ".PGM" ){
        return PGMLoader::load( file );
	} else if (ext == ".PPM" ){
	    return PPMLoader::load( file );
	} else if (ext == ".RGB" ){
	    return RGBLoader::load( file );
	//} else if (ext == ".TGA" ){
	    //return TGALoader::load( file );
    //} else if (ext == ".BMP" ){
        //return BMPLoader::load( file );
    } else {
        // tjeck if any plugins support the file format
        //std::cout << "CHECKING PLUGINS" << std::endl;
        PluginRepository &prep = RobWork::getInstance()->getPluginRepository();
        std::vector<PluginFactory<ImageLoader>::Ptr> loaderPlugins = prep.getPlugins<ImageLoader>();
        BOOST_FOREACH(PluginFactory<ImageLoader>::Ptr factory, loaderPlugins){
            //std::cout << "PLUGIN: " << factory->identifier() << std::endl;
            ImageLoader::Ptr loader = factory->make();
            // TODO: an image loader or factory should be able to tell what formats it supports
            // perhaps a propertymap on the factory interface could be used
            return loader->loadImage( file );
        }
    }
	RW_THROW("Image file: " << file << " with extension " << ext << " is not supported!");
	return NULL;
}
