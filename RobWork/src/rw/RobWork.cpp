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
#include "RobWork.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/os.hpp>
#include <boost/filesystem.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/common/ExtensionRegistry.hpp>
#include <rw/common/Plugin.hpp>

#include <cstdlib>
using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
//Using the below does not work with MSVS 2010 and MSVS 2012
//using namespace boost::filesystem;
using boost::filesystem::path;
using boost::filesystem::exists;
using boost::filesystem::initial_path;


#if defined(RW_CYGWIN)
    #define RWCFGHOMEDIR std::string(std::getenv("HOME")) + "/.config/robwork/"
    #define RWCFGFILE std::string(std::getenv("HOME")) + "/.config/robwork/robwork-" + RW_BUILD_TYPE + "-"+ RW_VERSION + ".cfg.xml"
#elif defined(RW_WIN32)
    #define RWCFGHOMEDIR std::string(std::getenv("APPDATA")) + "/robwork/"
    #define RWCFGFILE std::string(std::getenv("APPDATA")) + "/robwork/robwork-" + RW_BUILD_TYPE + "-"+ RW_VERSION + ".cfg.xml"
#elif defined(RW_MACOS)
    #define RWCFGHOMEDIR std::string(std::getenv("HOME"))  + "/Library/Preferences/"
    #define RWCFGFILE std::string(std::getenv("HOME")) + "/Library/Preferences/com.robwork-" + RW_BUILD_TYPE + "-"+ RW_VERSION + ".cfg.xml"
#elif defined(RW_LINUX)
    #define RWCFGHOMEDIR std::string(std::getenv("HOME")) + "/.config/robwork/"
    #define RWCFGFILE std::string(std::getenv("HOME")) + "/.config/robwork/robwork-" + RW_BUILD_TYPE + "-" + RW_VERSION + ".cfg.xml"
#endif


RobWork::RobWork(void) 
{
}

RobWork::~RobWork(void)
{
    if(!_settingsFile.empty()) {
        XMLPropertySaver::save( _settings, _settingsFile );
	}
}


void RobWork::initialize(){
	const Log::Ptr logInstance = Log::getInstance();
    // we need to find the settings of robwork so we start searching for rwsettings.xml
    Log::debugLog() << "Initializing ROBWORK" << std::endl;

	// this is the search priority
	// 1. search from execution directory
	// 2. search from user home directory (OS specific)

    path ipath = initial_path();
    std::string rwsettingsPath = ipath.string() + "/robwork-" + RW_BUILD_TYPE + "-" + RW_VERSION + ".cfg.xml";
    if( exists(rwsettingsPath) ){
    	//std::cout << "FOUND CFG FILE IN EXE DIR..." << std::endl;
        _settings = XMLPropertyLoader::load( rwsettingsPath );
        _settings.add("cfgfile", "", rwsettingsPath );
        //std::cout << "loading RobWork settings from: " << rwsettingsPath << std::endl;
    } else if( exists( RWCFGFILE ) ){
    	rwsettingsPath = std::string(RWCFGFILE);
        _settings = XMLPropertyLoader::load( rwsettingsPath );
    	_settings.add("cfgfile", "", rwsettingsPath );
    } else {
        // create the file in default current location
        PropertyMap plugins;

        plugins.add("location-1","Default plugin location",std::string("plugins/"));

        char* rwRootVar = getenv("RW_ROOT");
        if(rwRootVar != NULL)
            plugins.add("location-2","Default plugin location for RobWork",std::string(rwRootVar)+"/libs/"+RW_BUILD_TYPE+"/");

        char* rwsRootVar = getenv("RWS_ROOT");
        if(rwsRootVar != NULL)
            plugins.add("location-3","Default plugin location for RobWorkStudio",std::string(rwsRootVar)+"/libs/"+RW_BUILD_TYPE+"/");

        char* rwsimRootVar = getenv("RWSIM_ROOT");
        if(rwsimRootVar != NULL)
            plugins.add("location-4","Default plugin location for RobWorkSim",std::string(rwsimRootVar)+"/libs/"+RW_BUILD_TYPE+"/");

        char* rwhwRootVar = getenv("RWHW_ROOT");
        if(rwhwRootVar != NULL)
            plugins.add("location-5","Default plugin location for RobWorkHardware",std::string(rwhwRootVar)+"/libs/"+RW_BUILD_TYPE+"/");

        _settings.add("plugins","List of plugins or plugin locations",plugins);

        if( !exists( RWCFGHOMEDIR ) ){
            // create config dir
            boost::filesystem::path dir( RWCFGHOMEDIR );
            if ( boost::filesystem::create_directory(dir) )
                RW_WARN("Could not create cfg directory: " << RWCFGHOMEDIR  << ". Instead using: " <<  ipath.string());
        }

        if( exists( RWCFGHOMEDIR ) ){
            XMLPropertySaver::save( _settings, RWCFGFILE );
        } else {
            XMLPropertySaver::save( _settings, rwsettingsPath );
        }

    }
    _settingsFile = rwsettingsPath;

    // get all plugin directories and files
    std::vector<std::string> cfgDirs;
    PropertyMap pluginsMap = _settings.get<PropertyMap>("plugins",PropertyMap());

    Log::debugLog() << "Looking for RobWork plugins in following directories:\n";
    BOOST_FOREACH( PropertyBase::Ptr prop , pluginsMap.getProperties()){
    	// check if its a
    	Property<std::string>::Ptr propstr = prop.cast<Property<std::string> >();
    	if(propstr==NULL)
    		continue;

    	cfgDirs.push_back( propstr->getValue() );
    	//std::cout << "\t" << propstr->getIdentifier() << " " << propstr->getValue() << std::endl;
    	Log::debugLog() << "\t" << propstr->getValue() << std::endl;
    }

    Log::debugLog() << "Loading plugins:\n";
    BOOST_FOREACH(std::string dir, cfgDirs){
    	path file( dir );
    	Log::debugLog() << dir << std::endl;
#if(BOOST_FILESYSTEM_VERSION==2)
    	if( !file.has_root_path() ){
    		file = path( ipath.string() + "/" + dir );
    	}
#else
    	if( file.is_relative() ){
    		file = path( ipath.string() + "/" + dir );
    	}
#endif
    	Log::debugLog() << file.string() << std::endl;
    	if( !exists(file) )
    		continue;

        // now initialize plugin repository
        ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance();


    	// first check if its a directory or a file
    	if( is_directory(file) ){
    		// find all files in the directory *.rwplugin.xml *.rwplugin.(dll,so)
    		std::vector<std::string> pl_files =
    				IOUtil::getFilesInFolder(file.string(), false, true, "*.rwplugin.*");
    		BOOST_FOREACH(std::string pl_file, pl_files){
    		    Log::debugLog() << "\t Plugin: "<< pl_file<< std::endl;
                rw::common::Ptr<Plugin> plugin = Plugin::load( pl_file );
                reg->registerExtensions(plugin);
                Log::setLog(logInstance);
    		}
    	} else {
    	    Log::debugLog() << "\t Plugin: " <<  file.string() << std::endl;
            rw::common::Ptr<Plugin> plugin = Plugin::load( file.string() );
            reg->registerExtensions(plugin);
    	}

    }

}

namespace {
    RobWork::Ptr _rwinstance;
}

RobWork::Ptr RobWork::getInstance(){
    if(_rwinstance==NULL){
        _rwinstance = ownedPtr( new RobWork() );
    }
    return _rwinstance;
}

void RobWork::setInstance(RobWork::Ptr rw){
    _rwinstance = rw;
}
