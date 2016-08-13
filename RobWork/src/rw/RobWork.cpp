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
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/common/ExtensionRegistry.hpp>
#include <rw/common/Plugin.hpp>
#include <rw/common/StringUtil.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>

#include <cstdlib>
using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
//Using the below does not work with MSVS 2010 and MSVS 2012
//using namespace boost::filesystem;
using boost::filesystem::path;
using boost::filesystem::exists;
using boost::filesystem::initial_path;

using namespace boost::program_options;

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

RobWork::RobWork(void): _initialized(false)
{
}

RobWork::~RobWork(void)
{
    if(!_settingsFile.empty()) {
        DOMPropertyMapSaver::save(_settings, _settingsFile);
    }
}

void RobWork::initialize(const std::vector<std::string>& plugins){
	_initialized = true; // must be set before logging to avoid infinite loop
    Log::debugLog() << "Initializing ROBWORK" << std::endl;

    // user supplied arguments will always be taken into account

	// this is the search priority
    // 1. check if env variables define where robwork and therefore plugins are located
	// 2. search for config file from execution directory
	// 3. search from user home directory (OS specific)
    // 4. check hardcoded build directory
    path ipath = initial_path();
    std::string rwsettingsPath = ipath.string() + "/robwork-" + RW_BUILD_TYPE + "-" + RW_VERSION + ".cfg.xml";
    char* rwRootVar = getenv("RW_ROOT");

    if( rwRootVar != NULL ){
    	Log::debugLog() << "Found RobWork root dir in environment variable RW_ROOT..." << std::endl;
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

    } else if( exists(rwsettingsPath) ){
    	Log::debugLog() << "Found robwork configuration file in execution directory..." << std::endl;
        _settings = DOMPropertyMapLoader::load( rwsettingsPath );
        _settings.add("cfgfile", "", rwsettingsPath );
    } else if( exists( RWCFGFILE ) ){
    	Log::debugLog() << "Found robwork configuration file in global configuration directory:\n\t" << std::string(RWCFGFILE) << std::endl;
    	rwsettingsPath = std::string(RWCFGFILE);
        _settings = DOMPropertyMapLoader::load( rwsettingsPath );
    	_settings.add("cfgfile", "", rwsettingsPath );
    } else if( exists( std::string(RW_BUILD_DIR) ) ){
    	// check if the build directory exist
    	Log::debugLog() << "Found robwork in build directory: \"" << std::string(RW_BUILD_DIR) << "\""<< std::endl;
    	std::string buildDir( RW_BUILD_DIR );

        // create the file in default current location
        PropertyMap plugins;
        plugins.add("location-1","Default plugin location",std::string("plugins/"));
        plugins.add("location-2","Default plugin location for RobWork",buildDir+"/libs/"+RW_BUILD_TYPE+"/");
        plugins.add("location-3","Default plugin location for RobWorkStudio",buildDir+"/../RobWorkStudio/libs/"+RW_BUILD_TYPE+"/");
        plugins.add("location-4","Default plugin location for RobWorkSim",buildDir+"/../RobWorkSim/libs/"+RW_BUILD_TYPE+"/");
        plugins.add("location-5","Default plugin location for RobWorkHardware",buildDir+"/../RobWorkHardware/libs/"+RW_BUILD_TYPE+"/");

        _settings.add("plugins","List of plugins or plugin locations",plugins);
    }
    _settingsFile = rwsettingsPath;

    // get all plugin directories and files
    std::vector<std::string> cfgDirs;
    PropertyMap pluginsMap = _settings.get<PropertyMap>("plugins",PropertyMap());

    // add user defined plugin hints
    Log::debugLog() << "Adding plugins from arguments:\n";
    for(size_t i=0;i<plugins.size();i++){
    	std::stringstream sstr;
    	sstr << "loc-from-arg-" << i;
    	pluginsMap.add(sstr.str(), "Plugin location from init arguments", plugins[i]);
    }

    Log::debugLog() << "Looking for RobWork plugins in following directories:\n";
    BOOST_FOREACH( PropertyBase::Ptr prop , pluginsMap.getProperties()){
    	// check if its a
    	Property<std::string>::Ptr propstr = prop.cast<Property<std::string> >();
    	if(propstr==NULL)
    		continue;

    	cfgDirs.push_back( propstr->getValue() );
    	Log::debugLog() << "\t" << propstr->getValue() << std::endl;
    }

    std::vector<std::string> pluginsFiles;
    Log::debugLog() << "Loading plugins:\n";
    BOOST_FOREACH(std::string dir, cfgDirs){
    	path file( dir );
    	Log::debugLog() << " processing hint: " << dir << std::endl;
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

    	// first check if its a directory or a file
    	if( is_directory(file) ){
    		// find all files in the directory *.rwplugin.xml *.rwplugin.(dll,so)
    		std::vector<std::string> pl_files =
    				IOUtil::getFilesInFolder(file.string(), false, true, "*.rwplugin.*");
    		BOOST_FOREACH(std::string pl_file, pl_files){
				const std::string ext = StringUtil::getFileExtension(pl_file);
				if (ext == ".xml" || ext == ".dll" || ext == ".so")
    				pluginsFiles.push_back(pl_file);
    		}
    	} else {
    		pluginsFiles.push_back(file.string());
    	}
    }

    // make sure not to add duplicates of plugins
    ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance();

    BOOST_FOREACH(std::string pfilename, pluginsFiles){

    	std::time_t time = boost::filesystem::last_write_time(pfilename);
    	// check if plugin was allready added
    	path pfile(pfilename);

    	std::string key = pfile.filename().string();
    	if(_pluginChangedMap.find(key)!=_pluginChangedMap.end()){
    		if( _pluginChangedMap[key]>=time )
    			continue;
    	}

		_pluginChangedMap[key] = time;
		Log::debugLog() << "\t " <<  pfilename << std::endl;
		try {
			rw::common::Ptr<Plugin> plugin = Plugin::load( pfilename );
			if (!plugin.isNull()) {
				reg->registerExtensions(plugin);
			} else {
				Log::errorLog() << "Error loading plugin: \n\t\"" << pfilename << "\" "
					<< "\n\t Please fix error, ignoring plugin for now.." << std::endl;
			}
		} catch (const std::exception& e ){
			Log::errorLog() << "Error loading plugin: \n\t\"" << pfilename << "\" "
							<< "\n\t Error description: " << e.what()
							<< "\n\t Please fix error, ignoring plugin for now.." << std::endl;
		}
    }
}

namespace {
	ExtensionRegistry::Ptr* _newExtensionReg;
    rw::common::Log::Ptr _log;

    RobWork::Ptr* _newInstance = NULL; // must be pointer such that lifetime is controlled with the local static in rwinstance()

    // The instance must be defined as a local static to give us necessary control of the lifetime (RobWork depends on other global static variables):
    RobWork::Ptr rwinstance() {
    	// Local static variable used, such that variables used by DOMPropertyMapSaver is still available at destruction of RobWork
    	static DOMPropertyMapSaver::Initializer init; // make sure that local static variables is destructed after RobWork instance is destructed.
    	static RobWork::Ptr _rwinstance = ownedPtr(new RobWork());
    	if (_newInstance != NULL) {
    		// Allow switching with a new instance
    		// _newInstance can not be of type Ptr as this would cause RobWork to be destructed too late!
    		_rwinstance = *_newInstance;
    		delete _newInstance;
    		_newInstance = NULL;
    	}
    	return _rwinstance;
    }

    // this is used for initializing variables on program startup
    // hence, any program linked with this code will execute the constructor
    // before main(argc,argv) is entered...

    /*struct AutoInitializeRobWork {
       	AutoInitializeRobWork(){
       		//rw::common::Log::debugLog() << " AUTO INITILIZING ROBWORK .... " << std::endl;
        	if(_log==NULL)
        		_log =  rw::common::ownedPtr( new rw::common::Log() );
    		if(_extensionReg==NULL)
    			_extensionReg = rw::common::ownedPtr(new rw::common::ExtensionRegistry());
       		//RobWork::getInstance()->initialize();

       	}
    } _initializer;*/

}

rw::common::Ptr<rw::common::ExtensionRegistry> RobWork::getExtensionRegistry(){
    static ExtensionRegistry::Ptr extensionReg = ownedPtr(new ExtensionRegistry());
    if (_newExtensionReg != NULL) {
    	// Allow switching with a new registry
    	// _newExtensionReg can not be of type Ptr as this would cause the registry to be destructed too late!
    	extensionReg = *_newExtensionReg;
    	delete _newExtensionReg;
    	_newExtensionReg = NULL;
    }
    return extensionReg;
}

void RobWork::setExtensionRegistry(rw::common::Ptr<rw::common::ExtensionRegistry> extreg){
	_newExtensionReg = new rw::common::Ptr<ExtensionRegistry>(extreg);
}

bool RobWork::isInitialized() const {
	return _initialized;
}

void RobWork::init(int argc, const char* const * argv){
	// get log level, plugins, or plugin directories

    options_description desc("RobWork options");
    desc.add_options()
        ("help", "produce help message")
        ("rwloglevel", value<std::string>()->default_value("info"), "Set to debug, info, error, fatal")
        ("rwplugin", value<std::vector<std::string> >()->multitoken(), "Specific RobWork plugins or plugin directories to load. ")
    	("rwroot", value<std::string>(), "Directory of RobWork installation or development environment.")
    ;
    variables_map vm;
    store(command_line_parser(argc, argv).allow_unregistered().
              options(desc).run(), vm);
    notify(vm);

    if (vm.count("help")) {
    	std::cout << "Usage example setting RobWork parameters:\n\n"
                  << "\t" << argv[0] <<" --rwloglevel=debug --rwplugin=/home/user/userplugin/libs/release/userplugin.rwplugin.so \n"
                  << "\t Plugins look like *.rwplugin.(dll,so,xml) \n"
                  << "\n";
    	std::cout << desc << "\n";
        return;
    }

    std::vector<std::string> plugins;
    if( vm.count("rwplugin") ){
    	plugins = vm["rwplugin"].as<std::vector<std::string> >();
    }

	std::string rwloglevel_arg = vm["rwloglevel"].as<std::string>();
	if(rwloglevel_arg=="debug"){ Log::getInstance()->setLevel( Log::Debug ); }
	else if(rwloglevel_arg=="info"){ Log::getInstance()->setLevel( Log::Info ); }
	else if(rwloglevel_arg=="error"){ Log::getInstance()->setLevel( Log::Error ); }
	else if(rwloglevel_arg=="fatal"){ Log::getInstance()->setLevel( Log::Fatal ); }
	else { RW_WARN("rwloglevel set to unknown value!"); }

	// Some plugins have already been loaded through global initialization - before the user was able to set the debug level
	// - so we print the already loaded plugins to the debug log.
    Log::debugLog() << "Initializing ROBWORK with arguments." << std::endl;
	const RobWork::Ptr instance = rwinstance();
	Log::debugLog() << "Already loaded plugins: " << instance->_pluginChangedMap.size() << std::endl;
	for (std::map<std::string, std::time_t>::const_iterator it = instance->_pluginChangedMap.begin(); it != instance->_pluginChangedMap.end(); it++) {
		Log::debugLog() << "\t" << it->first << std::endl;
	}
	instance->initialize(plugins);
}

void RobWork::setLog(rw::common::Log::Ptr log){
	_log = log;
}

rw::common::Log& RobWork::getLog()
{
	// test for NULL, to avoid problems with 'static initialization order fiasco'
	if(_log==NULL)
		_log =  rw::common::ownedPtr( new rw::common::Log() );
    return *_log;
}

rw::common::Log::Ptr RobWork::getLogPtr()
{
	if(_log==NULL)
		_log =  rw::common::ownedPtr( new rw::common::Log() );
	return _log;
}

rw::common::PropertyMap& RobWork::getSettings() {
    return _settings;
}

void RobWork::init(){
	rwinstance()->initialize();
}

RobWork::Ptr RobWork::getInstance(){
	RobWork::Ptr instance = rwinstance();
	if (!instance->isInitialized())
		instance->initialize();
	return instance;
}

void RobWork::setInstance(RobWork::Ptr rw){
	_newInstance = new Ptr(rw);
}
