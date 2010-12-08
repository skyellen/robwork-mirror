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

#define QT_NO_EMIT

#include "RobWorkStudio.hpp" 

#ifdef __WIN32
#include <windows.h>
#endif //#ifdef __WIN32
#include <QApplication>
#include <QMainWindow>

#include <rw/RobWork.hpp>
#include <rw/common/PropertyMap.hpp>

/*#include <rw/math/Q.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/common/TimerUtil.hpp>
*/

#include <RobWorkStudioConfig.hpp>

#include <RobWorkConfig.hpp>
using namespace rws;
using namespace rw;
using namespace rw::common;

#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/loaders/xml/XMLPropertyFormat.hpp>



using namespace rw::loaders;
using namespace rw::common;


#ifdef RWS_USE_STATIC_LINK_PLUGINS


#include <rws/plugins/log/ShowLog.hpp>
#include <rws/plugins/jog/Jog.hpp>
#include <rws/plugins/treeview/TreeView.hpp>
#include <rws/plugins/playback/PlayBack.hpp>
#include <rws/plugins/planning/Planning.hpp>
#include <rws/plugins/propertyview/PropertyView.hpp>
#include <rws/plugins/sensors/Sensors.hpp>
#include <rws/plugins/lua/Lua.hpp>

#ifdef RWS_HAVE_SANDBOX
//Plugins which are available in the sandbox
#endif




std::vector<rws::RobWorkStudio::PluginSetup> getPlugins()
{
	
    typedef rws::RobWorkStudio::PluginSetup Pl;
    std::vector<Pl> plugins;
    plugins.push_back(Pl(new rws::Jog(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new rws::TreeView(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new rws::PlayBack(), false, Qt::BottomDockWidgetArea));

    plugins.push_back(Pl(new rws::PropertyView(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new rws::ShowLog(), false, Qt::BottomDockWidgetArea));
    plugins.push_back(Pl(new rws::Planning(), false, Qt::LeftDockWidgetArea));

    plugins.push_back(Pl(new rws::Sensors(), false, Qt::RightDockWidgetArea));
    plugins.push_back(Pl(new rws::Lua(), false, Qt::LeftDockWidgetArea));

#if RWS_HAVE_SANDBOX
    //Plugins which are avaible in the sandbox
#endif

    return plugins;
}
#else
std::vector<RobWorkStudio::PluginSetup> getPlugins()
{
    return std::vector<RobWorkStudio::PluginSetup>();
}

std::vector<int> getIntegers() {
	return std::vector<int>();
}

#endif // RW_STATIC_LINK_PLUGINS 


//#ifndef _MSC_VER

#include "ProgramOptions.hpp"
namespace po=boost::program_options;
po::options_description desc("Options");
int opt;

void initOptions(po::options_description& desc){
    desc.add_options()
        ("help", "produce help message")
        ("version,v", "print version string")
		("ini-file", po::value< std::string >()->default_value("RobWorkStudio.ini"), "RobWorkStudio ini-file")
        ("intproperty,i", po::value< IntOptionList >()->composing(),"Add a int property, name=2")
        ("doubleproperty,d", po::value< DoubleOptionList >()->composing(),"Add a double property, name=2.3")
        ("qproperty,q", po::value< QOptionList >()->composing(),"Add a Q property, name=(1.0,2,32.1,2)")
        ("property,P", po::value< StringOptionList >()->composing(),"Add a string property, name=pstring")
        ("input-file", po::value< std::string >(), "Project/Workcell/Device input file")
    ;
}

//#endif //#ifndef _MSC_VER

#include <fstream>
int main(int argc, char** argv)
{

    Q_INIT_RESOURCE(rwstudio_resources);
    int res = 0;
    PropertyMap map;
    std::string inifile, inputfile;

//#ifdef _MSC_VER
	//if (argc > 1)
	//	inputfile = argv[1];
//#else
	try {
        initOptions(desc);

        po::positional_options_description p;
        p.add("input-file", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                  options(desc).positional(p).run(), vm);
		po::notify(vm);

        if (vm.count("help")) {
            std::cout << "Usage:\n\n"
                      << "\tRobWorkStudio [options] <project-file> \n"
                      << "\tRobWorkStudio [options] <workcell-file> \n"
                      << "\tRobWorkStudio [options] <device-file> \n"
                      << "\n";
            std::cout << desc << "\n";
            return 1;
        }

        if (vm.count("version") ){
        	std::cout << "\n\tRobWorkStudio version " << RW_VERSION << std::endl;
            return 1;
        }

        if( vm.count("ini-file") ){
            inifile = vm["ini-file"].as<std::string>();
        }

        if( vm.count("property") ){
            StringOptionList vals = vm["property"].as< StringOptionList >();
            BOOST_FOREACH(Option<std::string>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("intproperty") ){
            IntOptionList vals = vm["intproperty"].as< IntOptionList >();
            BOOST_FOREACH(Option<int>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("doubleproperty") ){
            DoubleOptionList vals = vm["doubleproperty"].as< DoubleOptionList >();
            BOOST_FOREACH(Option<double>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("qproperty") ){
            QOptionList vals = vm["qproperty"].as< QOptionList >();
            BOOST_FOREACH(Option<rw::math::Q>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }

        if( vm.count("input-file") ){
            //std::cout << "input-file: " << vm["input-file"].as<std::string>() << std::endl;
            inputfile = vm["input-file"].as<std::string>();
        }
    } catch (std::exception &e){
    	Log().info() << "Command line input error:\n\t " << e.what() << "\n";
    	Log().info() << "Specify --help for usage. \n";
        return 0;
    }
//#endif //#ifdef MSVS #else

    QApplication app(argc, argv);
    try {
//		std::vector<int> integers;
//		integers = getIntegers();
	
        std::vector<rws::RobWorkStudio::PluginSetup> plugins; 
		
        QPixmap pixmap(":/images/splash.jpg");

        QSplashScreen splash(pixmap); 
        splash.show();
        // Loading some items
        splash.showMessage("Adding static plugins");
        
		plugins = getPlugins();

        app.processEvents();
        // Establishing connections
        splash.showMessage("Loading dynamic plugins");
		inifile="./RobWorkStudio.ini";

        RobWork robwork;
        std::string pluginFolder = "./plugins/";

        Log::infoLog() <<"Input File = "<< inputfile <<std::endl;

		rws::RobWorkStudio rwstudio(&robwork, plugins, map, inifile);
		
        if(!inputfile.empty()){ 
            rwstudio.openFile(inputfile);
        }

        // load configuration into RobWorkStudio
        // Todo: check that the config file exists
        splash.showMessage("Loading settings");
		
        rwstudio.show();
        splash.finish(&rwstudio);
        res = app.exec();
        Log::infoLog() << "Application Ready to Terminate" << std::endl;
    } catch (const Exception& e) {
        std::cout << e.what() << std::endl;
        QMessageBox::critical(NULL, "RW Exception", e.what().c_str());
        return -1;
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        QMessageBox::critical(NULL, "Exception", e.what());
        return -1;
    }

    return 0;
}

