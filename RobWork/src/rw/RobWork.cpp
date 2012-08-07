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
#include <cstdlib>
using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace boost::filesystem;


#if defined(RW_CYGWIN)
    #define HOMEDIR std::string(std::getenv("HOME")) + "/.config/robwork/robwork.cfg.xml"
#elif defined(RW_WIN32)
    #define HOMEDIR std::string(std::getenv("APPDATA")) + "/robwork/robwork.cfg.xml"
#elif defined(RW_MACOS)
    #define HOMEDIR std::string(std::getenv("HOME")) + "/Library/Preferences/com.robwork.cfg.xml"
#elif defined(RW_LINUX)
    #define HOMEDIR std::string(std::getenv("HOME")) + "/.config/robwork/robwork.cfg.xml"
#endif


RobWork::RobWork(void) 
{
}

RobWork::~RobWork(void)
{
    if(!_settingsFile.empty())
        XMLPropertySaver::save( _settings, _settingsFile );
}


void RobWork::initialize(){
    // we need to find the settings of robwork so we start searching for rwsettings.xml
    path ipath = initial_path();
    std::string rwsettingsPath = ipath.string() + "/robwork.cfg.xml";
    if( exists(rwsettingsPath) ){
        _settings = XMLPropertyLoader::load( rwsettingsPath );
        std::cout << "loading RobWork settings from: " << rwsettingsPath << std::endl;
    } else if( exists( HOMEDIR ) ){
        _settings = HOMEDIR;
    } else {
        // create the file in current location
        PropertyMap plugins;
        plugins.add("location","default plugin location",std::string("plugins/"));

        _settings.add("plugins","List of plugins or plugin locations",plugins);

        //XMLPropertySaver::save( _settings, rwsettingsPath );
    }
    _settingsFile = rwsettingsPath;

    // now initialize plugin repository

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
