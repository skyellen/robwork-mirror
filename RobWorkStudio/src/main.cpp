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
#define _HAS_ITERATOR_DEBUGGING 0

#define QT_NO_EMIT

#ifdef __WIN32
#include <windows.h>
#endif
#include <QApplication>
#include <rws/RobWorkStudio.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/common/PropertyMap.hpp>

#ifdef _MSC_VER
#include <RobWorkStudioConfigVS.hpp>
#else
#include <RobWorkStudioConfig.hpp>
#endif


#ifdef RW_STATIC_LINK_PLUGINS

#include <plugins/log/ShowLog.hpp>
#include <plugins/jog2/Jog.hpp>
#include <plugins/treeview/TreeView.hpp>
#include <plugins/playback/PlayBack.hpp>
#include <plugins/propertyview/PropertyView.hpp>
#include <RobWorkConfig.hpp>
#include <plugins/planning/Planning.hpp>

#ifdef RWS_HAVE_SANDBOX
	#include <sandbox/plugins/lua/Lua.hpp>
#endif

using namespace std;
using namespace rw::common;

std::vector<RobWorkStudio::PluginSetup> getPlugins()
{
    typedef RobWorkStudio::PluginSetup Pl;
    std::vector<Pl> plugins;
    plugins.push_back(Pl(new Jog(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new TreeView(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new PlayBack(), false, Qt::BottomDockWidgetArea));

    plugins.push_back(Pl(new PropertyView(), false, Qt::LeftDockWidgetArea));
    plugins.push_back(Pl(new ShowLog(), false, Qt::BottomDockWidgetArea));
    plugins.push_back(Pl(new Planning(), false, Qt::LeftDockWidgetArea));

#if RWS_HAVE_SANDBOX
    plugins.push_back(Pl(new Lua(), false, Qt::LeftDockWidgetArea));
#endif

    return plugins;
}
#else
std::vector<RobWorkStudio::PluginSetup> getPlugins()
{
    return std::vector<RobWorkStudio::PluginSetup>();
}
#endif /* RW_STATIC_LINK_PLUGINS */

/*
int exp_handle()
{
       return ExceptionContinueExecution;
}
*/


#ifndef _MSC_VER

#include "ProgramOptions.hpp"
po::options_description desc("Options");
int opt;

#endif //#ifndef _MSC_VER

#include <fstream>
int main(int argc, char** argv)
{
	std::ofstream file("file.txt");
	std::streambuf * old = std::cout.rdbuf(file.rdbuf());
// do here output to std::cout
	std::cout.rdbuf(old); // restore

    Q_INIT_RESOURCE(rwstudio_resources);
    int res = 0;
    PropertyMap map;
    std::string inifile, inputfile;
#ifdef _MSC_VER
	if (argc > 1)
		inputfile = argv[1];
#else
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
            cout << desc << "\n";
            return 1;
        }

        if (vm.count("version") ){
            cout << "\n\tRobWorkStudio version " << RW_VERSION << std::endl;
            return 1;
        }

        if( vm.count("ini-file") ){
            inifile = vm["ini-file"].as<std::string>();
        }

        if( vm.count("property") ){
            vector<MyProperty<string> > vals = vm["property"].as< vector<MyProperty<string> > >();
            BOOST_FOREACH(MyProperty<string>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("intproperty") ){
            vector<MyProperty<int> > vals = vm["intproperty"].as< vector<MyProperty<int> > >();
            BOOST_FOREACH(MyProperty<int>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("doubleproperty") ){
            vector<MyProperty<double> > vals = vm["doubleproperty"].as< vector<MyProperty<double> > >();
            BOOST_FOREACH(MyProperty<double>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }
        if( vm.count("qproperty") ){
            vector<MyProperty<Q> > vals = vm["qproperty"].as< vector<MyProperty<Q> > >();
            BOOST_FOREACH(MyProperty<Q>& prop, vals){
                map.add(prop.name,"",prop.value);
            }
        }

        if( vm.count("input-file") ){
            std::cout << "input-file: " << vm["input-file"].as<string>() << std::endl;
            inputfile = vm["input-file"].as<string>();
        }
    } catch (exception &e){
        cout << "RobWorkStudio Error: " << e.what() << "\n";
        cout << "Specify --help for usage. \n";
        return 0;
    }
#endif //#ifdef MSVS

    //__try1(exp_handle){
    try {
        QApplication app(argc, argv);
        QPixmap pixmap(":/images/splash.jpg");

        QSplashScreen splash(pixmap);
        splash.show();
        // Loading some items
        splash.showMessage("Adding static plugins");
        std::vector<RobWorkStudio::PluginSetup> plugins = getPlugins();
        //rw::common::TimerUtil::sleepMs(500);
        // could be nice to load all dynamic plugins here
        // also perhaps loading configuration file

        app.processEvents();
        // Establishing connections
        splash.showMessage("Loading dynamic plugins");
		inifile="./RobWorkStudio.ini";
        RobWorkStudio rstudio(plugins, map, inifile);
        if(!inputfile.empty()){
            rstudio.openFile(inputfile);
        }
        // load configuration into RobWorkStudio
        // Todo: check that the config file exists
        // splash.showMessage("Loading settings");

        rstudio.show();
        splash.finish(&rstudio);
        res = app.exec();
    } catch (exception& e) {
        std::cout << e.what() << std::endl;
        return 0;
    }
    //}
    //__except1{
    //    exit(0);
    //}

    // remember to save configuration
    // XMLPropertySaver::save(rstudio.getConfig(), "RobWorkStudio.config.xml");
    return res;
}

