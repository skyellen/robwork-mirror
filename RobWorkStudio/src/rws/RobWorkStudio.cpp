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
#include <QApplication>
#include <QAction>
#include <QCloseEvent>
#include <QFileDialog>
#include <QIcon>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QObject>
#include <QPluginLoader>
#include <QSettings>
#include <QStringList>
#include <QToolBar>
#include <QUrl>
#include <QMimeData>

#include "RobWorkStudio.hpp"
#include "AboutBox.hpp"
#include "RobWorkStudioPlugin.hpp"
#include "HelpAssistant.hpp"

#include <rw/common/os.hpp>
#include <rw/common/Exception.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/loaders/dom/DOMWorkCellSaver.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rws/propertyview/PropertyViewEditor.hpp>

#include <RobWorkConfig.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <sstream>

//#include <sandbox/loaders/ColladaLoader.hpp>

using namespace rw;
using namespace rw::common;

using namespace rw::loaders;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;

using namespace rws;

namespace
{
    WorkCell::Ptr emptyWorkCell()
    {
        WorkCell::Ptr workcell = rw::common::ownedPtr(new WorkCell(ownedPtr(new StateStructure())));
        CollisionSetup::set(CollisionSetup(), workcell);
        return workcell;
    }

    CollisionDetector::Ptr makeCollisionDetector(WorkCell::Ptr workcell)
    {
        return rw::common::ownedPtr(
            new CollisionDetector(
                workcell,
                ProximityStrategyFactory::makeDefaultCollisionStrategy()
        ));
    }
}


RobWorkStudio::RobWorkStudio(const PropertyMap& map)
    :
    QMainWindow(NULL),
    _robwork(RobWork::getInstance()),
    _aboutBox(NULL),
    _inStateUpdate(false),
    _settingsMap(NULL)
{
	//Always create the about box.
	_aboutBox = new AboutBox(RW_VERSION, RW_REVISION, this);
	// should load dynamically
    //rw::common::ExtensionRegistry::getInstance()->registerExtensions( ownedPtr( new RWSImageLoaderPlugin() ) );
    //_robwork->getPluginRepository().addPlugin(ownedPtr( new ColladaLoaderPlugin() ), true);
    std::stringstream sstr;
    sstr << " RobWorkStudio v" << RW_VERSION;
    QString qstr(sstr.str().c_str());
    setWindowTitle( qstr );

    // time 50ms
    setWindowIcon( QIcon(":/images/rw_logo_64x64.png") );
    boost::filesystem::path settingsPath("rwsettings.xml");

    PropertyMap settings;
    if( exists(settingsPath) ){
        try {
            //settings = XMLPropertyLoader::load("rwsettings.xml");
            //_propMap.set<std::string>("SettingsFileName", "rwsettings.xml");
            _propMap = XMLPropertyLoader::load("rwsettings.xml");
        } catch(rw::common::Exception &e){
            RW_WARN("Could not load settings from 'rwsettings.xml': " << e.getMessage().getText() << "\n Using default settings!");
        } catch(std::exception &e){
            RW_WARN("Could not load settings from 'rwsettings.xml': " << e.what() << "\n Using default settings!");
            // loading failed so we just go on with an empty map
        }
    }
    _propMap.set("cmdline",map);
    PropertyMap *currentSettings = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");
    if(currentSettings==NULL){
        _propMap.add("RobWorkStudioSettings", "Settings for RobWorkStudio", settings);
        currentSettings = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");
    } /*else {
        *currentSettings = settings;
    }*/

    _assistant = new HelpAssistant();
    _settingsMap = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");

    // set the drag and drop property to true
    setupFileActions();
	setupToolActions();
    setupViewGL();
    _propEditor = new PropertyViewEditor( NULL );
    _propEditor->setPropertyMap( &_propMap  );
    setupPluginsMenu();
    setupHelpMenu();
    int width = _settingsMap->get<int>("WindowWidth", 1024);
    int height = _settingsMap->get<int>("WindowHeight", 800);
    int x = _settingsMap->get<int>("WindowPosX", this->pos().x());
    int y = _settingsMap->get<int>("WindowPosY", this->pos().y());
    std::vector<int> state_vector = _settingsMap->get<std::vector<int> >("QtMainWindowState", std::vector<int>());
    if(state_vector.size()>0){
        QByteArray mainAppState((int)state_vector.size(),0);
        for(int i=0;i<mainAppState.size();i++){
            mainAppState[i] = (char) state_vector[i];
        }
        this->restoreState(mainAppState);
    }
    resize(width, height);
    this->move(x,y);

    //Initialize plugins
    //loadSettingsSetupPlugins(inifile);
    //BOOST_FOREACH(const PluginSetup& plugin, plugins) {
    //    addPlugin(plugin.plugin, plugin.visible, plugin.area);
    //}

    // search for plugins in user specified locations
    //StringList slist;
    //slist.push_back("../../../RobWorkSim/libs/Debug/");
    //std::vector<PluginSetup> userPlugins = searchPlugins(slist);
    //BOOST_FOREACH(const PluginSetup& plugin, plugins) {
    //    addPlugin(plugin.plugin, plugin.visible, plugin.area);
    //}


    _workcell = emptyWorkCell();
    _state = _workcell->getDefaultState();
    _detector = makeCollisionDetector(_workcell);
    // Workcell given to view.
    _view->setWorkCell( _workcell );
    _view->setState(_state);

    // Workcell sent to plugins.
    openAllPlugins();
    //updateHandler(); 

    setAcceptDrops(true);
}

RobWorkStudio::~RobWorkStudio()
{
    delete _assistant;
    delete _propEditor;

    /*
    _propMap = PropertyMap();
    _lastFilesActions.clear();
    _state = State();
    _workcell = NULL;

    typedef std::vector<RobWorkStudioPlugin*>::iterator I;
    for(int i=_plugins.size()-1;i>=0;i--){
    //for (I it = _plugins.begin(); it != _plugins.end(); ++it) {
        std::cout << _plugins[i]->name().toStdString() << std::endl;
        delete _plugins[i];
    }
	*/
}

void RobWorkStudio::propertyChangedListener(PropertyBase* base){
    std::string id = base->getIdentifier();
    //std::cout << "Property Changed Listerner RWSTUDIO: " << id << std::endl;
}

void RobWorkStudio::closeEvent( QCloseEvent * e ){
    // save main window settings
    //std::cout << "closeEvent" << std::endl;
    //settings.setValue("pos", pos());
    //settings.setValue("size", size());
    //settings.setValue("state", saveState());
    QByteArray mainAppState = saveState();
    //std::cout << mainAppState.data() << std::endl;
    std::vector<int> state_vector( mainAppState.size() );
    for(int i=0;i<mainAppState.size();i++){
        state_vector[i] = mainAppState[i];
    }


    //QString myStringState;
    //QTextStream out(&myStringState);
    //out << mainAppState;
    //std::cout << myStringState.toStdString()<< std::endl;

    _settingsMap->set<std::vector<int> >("QtMainWindowState", state_vector);
    _settingsMap->set<int>("WindowPosX", this->pos().x());
    _settingsMap->set<int>("WindowPosY", this->pos().y());
    _settingsMap->set<int>("WindowWidth", this->width());
    _settingsMap->set<int>("WindowHeight", this->height());

    // save the settings of each plugin
    /*
    BOOST_FOREACH(RobWorkStudioPlugin* plugin, _plugins){
        bool visible = plugin->isVisible();

        bool floating = plugin->isFloating();
        int intarea = (int) dockWidgetArea(plugin);
        std::string pname = plugin->name().toStdString();
        _settingsMap->set<bool>( std::string("PluginVisible_")+ pname , visible);
        _settingsMap->set<bool>( std::string("PluginFloating_")+pname , floating);
        _settingsMap->set<int>( std::string("PluginArea_")+ pname , intarea);
    }
    */

    if( !_propMap.get<PropertyMap>("cmdline").has("NoSave") ){
        _propMap.set("cmdline", PropertyMap());
        try {
            XMLPropertySaver::save(_propMap, "rwsettings.xml");
        } catch(const rw::common::Exception& e) {
            RW_WARN("Error saving settings file: " << e);
        } catch(...) {
            RW_WARN("Error saving settings file due to unknown exception!");
        }
    }
    _propMap = PropertyMap();
    _propEditor->close();

    closeAllPlugins();
    _view->clear();
	_view->close();

	// close all plugins
    typedef std::vector<RobWorkStudioPlugin*>::iterator I;
    for (I it = _plugins.begin(); it != _plugins.end(); ++it) {
        //std::cout << "closing PLUGIN: " << (*it)->name().toStdString() << std::endl;
        (*it)->QWidget::close();
    }

	// now call accept
	e->accept();
}


rw::common::Log& RobWorkStudio::log(){
    return _robwork->getLog();  	
}

rw::common::Log::Ptr RobWorkStudio::logPtr(){
    return _robwork->getLogPtr();
}

void RobWorkStudio::updateLastFiles()
{
        QMenu* filemenu = _fileMenu;
        std::vector<std::pair<QAction*,std::string> >& fileactions = _lastFilesActions;
        std::vector<std::string> nfiles = _settingsMap->get<std::vector<std::string> >("LastOpennedFiles", std::vector<std::string>());
        // remove old actions
        for(size_t i=0;i<fileactions.size();i++){
            filemenu->removeAction(fileactions[i].first);
        }
        fileactions.clear();

        // sort nfiles such that multiples are left out
        std::vector<std::string> tmp;
        for(size_t i=0; i<nfiles.size();i++){
            int idx = (int)(nfiles.size()-1-i);
            bool skip = false;
            BOOST_FOREACH(std::string &str, tmp){
                if(str == nfiles[idx]){
                    skip=true;
                    break;
                }
            }
            if(!skip)
                tmp.push_back(nfiles[idx]);
            if(tmp.size()>10)
                break;
        }
        nfiles.resize(tmp.size());

        // now add the new ones
        for(size_t i=0;i<tmp.size();i++){
            nfiles[tmp.size()-1-i] = tmp[i];
            boost::filesystem::path p(tmp[i]);
            std::stringstream sstr;
            sstr << i << ": " << p.filename();
            QAction* nAction = filemenu->addAction( sstr.str().c_str() );

            connect(nAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));
            filemenu->addAction( nAction );
            fileactions.push_back( std::make_pair(nAction, tmp[i]) );
        }

        _settingsMap->set<std::vector<std::string> >("LastOpennedFiles", nfiles);

    }



void RobWorkStudio::setupFileActions()
{
    QAction* newAction =
        new QAction(QIcon(":/images/new.png"), tr("&New"), this); // owned
    connect(newAction, SIGNAL(triggered()), this, SLOT(newWorkCell()));

    QAction* openAction =
        new QAction(QIcon(":/images/open.png"), tr("&Open..."), this); // owned
    connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

    QAction* closeAction =
        new QAction(QIcon(":/images/close.png"), tr("&Close"), this); // owned
    connect(closeAction, SIGNAL(triggered()), this, SLOT(closeWorkCell()));

    QAction* saveAction =
        new QAction(QIcon(":/images/save.png"), tr("&Save"), this); // owned
    connect(saveAction, SIGNAL(triggered()), this, SLOT(saveWorkCell()));

	QAction* exitAction =
        new QAction(QIcon(), tr("&Exit"), this); // owned
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

    QToolBar* fileToolBar = addToolBar(tr("File"));
    fileToolBar->setObjectName("FileToolBar");
    fileToolBar->addAction(newAction);
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(closeAction);
    fileToolBar->addAction(saveAction);
    ////
    _fileMenu = menuBar()->addMenu(tr("&File"));
    _fileMenu->addAction(newAction);
    _fileMenu->addAction(openAction);
    _fileMenu->addAction(closeAction);
    _fileMenu->addAction(saveAction);
    _fileMenu->addSeparator();

    QAction* propertyAction =
        new QAction(tr("&Preferences"), this); // owned
    connect(propertyAction, SIGNAL(triggered()), this, SLOT(showPropertyEditor()));

    _fileMenu->addAction(propertyAction);

    _fileMenu->addSeparator();

	_fileMenu->addAction(exitAction);

	_fileMenu->addSeparator();
    updateLastFiles();
}


void RobWorkStudio::setupToolActions() {
    QAction* printCollisionsAction =
        new QAction(QIcon(""), tr("Print Colliding Frames"), this); // owned
    connect(printCollisionsAction, SIGNAL(triggered()), this, SLOT(printCollisions()));

    _toolMenu = menuBar()->addMenu(tr("&Tools"));
    _toolMenu->addAction(printCollisionsAction);
}



void RobWorkStudio::printCollisions() {
	CollisionDetector::Ptr cd = getCollisionDetector();
	CollisionDetector::QueryResult res;
	cd->inCollision(getState(), &res);
	if (res.collidingFrames.size() > 0) {
		BOOST_FOREACH(const FramePair& pair, res.collidingFrames) {
			std::cout<<"Colliding: "<<pair.first->getName()<<" -- "<<pair.second->getName()<<std::endl;
			Log::infoLog()<<"Colliding: "<<pair.first->getName()<<" -- "<<pair.second->getName()<<std::endl;
		}
	}
}

void RobWorkStudio::setCheckAction(){
    QObject *obj = sender();

    // check if any of the open last file actions where choosen
    for(size_t i=0;i<_lastFilesActions.size();i++){
        if(obj == _lastFilesActions[i].first){
            openFile(_lastFilesActions[i].second);
            break;
        }
    }
}

void RobWorkStudio::showPropertyEditor(){
    // start property editor
   _propEditor->update();
   _propEditor->show();
   _propEditor->resize(400,600);
}



void RobWorkStudio::setupPluginsMenu()
{
	QAction* loadPluginAction = new QAction(QIcon(""), tr("Load plugin"), this);
	connect(loadPluginAction, SIGNAL(triggered()), this, SLOT(loadPlugin()));
	
	_pluginsMenu = menuBar()->addMenu(tr("&Plugins"));
	_pluginsMenu->addAction(loadPluginAction);
	_pluginsMenu->addSeparator();
    _pluginsToolBar = addToolBar(tr("Plugins"));
    _pluginsToolBar->setObjectName("PluginsBar");
    
    /*QAction* printCollisionsAction =
        new QAction(QIcon(""), tr("Print Colliding Frames"), this); // owned
    connect(printCollisionsAction, SIGNAL(triggered()), this, SLOT(printCollisions()));

    _toolMenu = menuBar()->addMenu(tr("&Tools"));
    _toolMenu->addAction(printCollisionsAction);*/
}



void RobWorkStudio::loadPlugin()
{
	QString selectedFilter;
	
	std::string previousOpenDirectory = _settingsMap->get<std::string>("PreviousOpenDirectory","");
    const QString dir(previousOpenDirectory.c_str());
	
	QString pluginfilename = QFileDialog::getOpenFileName(
        this,
        "Open plugin file", // Title
        dir, // Directory
        "Plugin libraries ( *.so *.dll *.dylib )"
        "\n All ( *.* )",
        &selectedFilter);
        
    if (!pluginfilename.isEmpty()) {
		QFileInfo pluginInfo(pluginfilename);
		QString pathname = pluginInfo.absolutePath();
		QString filename = pluginInfo.baseName();

		setupPlugin(pathname, filename, 0, 1);
	}
}



void RobWorkStudio::setupHelpMenu() {

    QAction *assistantAct = new QAction(tr("Help Contents"), this);
    assistantAct->setShortcut(QKeySequence::HelpContents);
    connect(assistantAct, SIGNAL(triggered()), this, SLOT(showDocumentation()));

    QAction* showAboutBox = new QAction("About",this);
    connect(showAboutBox, SIGNAL(triggered()), this, SLOT(showAboutBox()));

    QMenu* pHelpMenu = menuBar()->addMenu(tr("Help"));
    pHelpMenu->addAction(assistantAct);
    pHelpMenu->addAction(showAboutBox);

}
void RobWorkStudio::keyPressEvent(QKeyEvent *e)
{
    keyEvent().fire(e->key(), e->modifiers());
    QWidget::keyPressEvent(e);
}
void RobWorkStudio::showAboutBox(){
    _aboutBox->exec();
}

void RobWorkStudio::showDocumentation()
{
    QStringList filepaths;
    //std::cout << QCoreApplication::applicationFilePath().toStdString() << std::endl;
    filepaths.append( QCoreApplication::applicationDirPath() );

    _assistant->showDocumentation(filepaths);
}


void RobWorkStudio::setupViewGL()
{	
    _view = new RWStudioView3D(this, this);
    setCentralWidget( _view ); // own view
    _view->setupGUI( this );
}


void RobWorkStudio::openAllPlugins()
{
    typedef std::vector<RobWorkStudioPlugin*>::iterator I;
    for (I p = _plugins.begin(); p != _plugins.end(); ++p){
        //RW_WARN( (*p)->name().toStdString() << "4");
        openPlugin(**p);
    }
}


void RobWorkStudio::closeAllPlugins()
{
    typedef std::vector<RobWorkStudioPlugin*>::iterator PI;
    for (PI p = _plugins.begin(); p != _plugins.end(); ++p)
        closePlugin(**p);
}


void RobWorkStudio::openPlugin(RobWorkStudioPlugin& plugin)
{
    RW_ASSERT(_workcell);
    //std::cout<<"Number of devices in workcell in RobWorkStudio::openPlugin: "<<plugin.name().toStdString()<<" = "<< _workcell->getDevices().size()<<std::endl;
    try {
        //std::cout << "1" << plugin.name().toStdString() << std::endl;
        plugin.open(_workcell.get());
    } catch (const Exception& exc) {
        std::stringstream buf;
        buf
            << "Exception in opening of plugin "
            << StringUtil::quote(plugin.name().toStdString());
 
        QMessageBox::information( 
            NULL,
            buf.str().c_str(), 
            exc.getMessage().getText().c_str(),
            QMessageBox::Ok);
    }
    //std::cout << "2" << plugin.name().toStdString() << std::endl;
	
}


void RobWorkStudio::closePlugin(RobWorkStudioPlugin& plugin)
{
    try {
        plugin.close();
    } catch (const Exception& exc) {
        std::stringstream buf;
        buf
            << "Exception in closing of plugin "
            << StringUtil::quote(plugin.name().toStdString());

        QMessageBox::information(
            NULL,
            buf.str().c_str(),
            exc.getMessage().getText().c_str(),
            QMessageBox::Ok);
    }
}


void RobWorkStudio::addPlugin(RobWorkStudioPlugin* plugin,
                              bool visible,
                              Qt::DockWidgetArea area)
{

    plugin->setLog( _robwork->getLogPtr() );
    plugin->setRobWorkStudio(this);
    plugin->setRobWorkInstance(_robwork);
    plugin->setupMenu(_pluginsMenu);
    plugin->setupToolBar(_pluginsToolBar);

    plugin->initialize();

    // The updateSignal does not EXIST on the plugin interface....
    //connect(plugin, SIGNAL(updateSignal()), this, SLOT(updateHandler()));

    _plugins.push_back(plugin);
    std::string pname = plugin->name().toStdString();
    bool isVisible = _settingsMap->get<bool>( std::string("PluginVisible_")+pname, visible);
    bool isFloating = _settingsMap->get<bool>( std::string("PluginFloating_")+pname, false);
    int intarea = _settingsMap->get<int>( std::string("PluginArea_")+pname , (int)area);

    addDockWidget((Qt::DockWidgetArea)intarea, plugin);
    //addDockWidget(area, plugin);
    plugin->setFloating(isFloating);
    //IMPORTANT visibility must be set as the last thing....
    plugin->setVisible(isVisible);
    // Only open the plugin if the work cell is loaded.
    if (_workcell) openPlugin(*plugin);

    std::vector<int> state_vector = _settingsMap->get<std::vector<int> >("QtMainWindowState", std::vector<int>());
    if(state_vector.size()>0){
        QByteArray mainAppState((int)state_vector.size(),0);
        for(int i=0;i<mainAppState.size();i++){
            mainAppState[i] = (char) state_vector[i];
        }
        this->restoreState(mainAppState);
    }

}


void RobWorkStudio::loadSettingsSetupPlugins(const std::string& file)
{
    QSettings settings(file.c_str(), QSettings::IniFormat);
    switch (settings.status()) {
    case QSettings::NoError:
        setupPlugins(settings);
        break;

    case QSettings::FormatError: {
        std::string msg = file + " file not loaded";
        QMessageBox::information(
            NULL,
            "Format error",
            msg.c_str(),
            QMessageBox::Ok);
    }
        break;

    case QSettings::AccessError:
        // Nothing to report here.
        break;
    }

    // TODO: make error reply if necesarry
    //return settings.status();
}
/*
void RobWorkStudio::locatePlugins(QSettings& settings){
    rw::plugin::PluginRepository &prep = RobWork::getInstance()->getPluginRepository();
    prep.getPlugins<rws::MenuExtension>();
    prep.getPlugins<rws::RobWorkStudioPlugin>();

}
*/


void RobWorkStudio::setupPlugin(const QString& pathname, const QString& filename, bool visible, int dock)
{
	Qt::DockWidgetArea dockarea = (Qt::DockWidgetArea)dock;

	QString pfilename = pathname+ "/" + filename + "." + OS::getDLLExtension().c_str();
	bool e1 = boost::filesystem::exists( filename.toStdString() );
	if(!e1){
		pfilename = pathname+ "/" + filename + ".so";
		e1 = boost::filesystem::exists( pfilename.toStdString() );
	}
	if(!e1){
		pfilename = pathname+ "/" + filename + ".dll";
		e1 = boost::filesystem::exists( pfilename.toStdString() );
	}
	if(!e1){
		pfilename = pathname+ "/" + filename + ".dylib";
		e1 = boost::filesystem::exists( pfilename.toStdString() );
	}

	QPluginLoader loader(pfilename);
#if QT_VERSION >= 0x040400
	// Needed to make dynamicly loaded libraries use dynamic
	// cast on each others objects. ONLY on linux though.
	loader.setLoadHints(QLibrary::ResolveAllSymbolsHint |
						QLibrary::ExportExternalSymbolsHint);
#endif

	QObject* pluginObject = loader.instance();
	if (pluginObject != NULL) {
		RobWorkStudioPlugin* testP = dynamic_cast<RobWorkStudioPlugin*>(pluginObject);
		if (testP == NULL) {
			RW_THROW("Loaded plugin is NULL, tried loading \"" << pfilename.toStdString() << "\"" );
		}

		RobWorkStudioPlugin* plugin = qobject_cast<RobWorkStudioPlugin*>(pluginObject);

		if (plugin) {
			addPlugin(plugin, visible, dockarea);
		} else {
			RW_WARN("Unable to load Plugin"<<pfilename.toStdString() <<" was not of type RobWorkStudioPlugin");
			QMessageBox::information(
				this,
				"Unable to load Plugin",
				pfilename + " was not of type RobWorkStudioPlugin",
				QMessageBox::Ok);
		}
	} else {
		RW_WARN("Unable to load Plugin"<< pfilename.toStdString() << " was not loaded: \"" << loader.errorString().toStdString() + "\"");
		QMessageBox::information(
			this,
			"Unable to load Plugin",
			pfilename + " was not loaded: \"" + loader.errorString() + "\"",
			QMessageBox::Ok);
	}
}


void RobWorkStudio::setupPlugins(QSettings& settings)
{

    QStringList groups = settings.childGroups();

    settings.beginGroup("Plugins");
    QStringList plugins = settings.childGroups();
    for (int i = 0; i<plugins.size(); i++) {
        const QString& pluginname = plugins.at(i);


        Log::debugLog()  << "Plugin = " << pluginname.toStdString() << "\n";

        settings.beginGroup(pluginname);

        QString pathname = settings.value("Path").toString();
        QString filename = settings.value("Filename").toString();
        bool visible = settings.value("Visible").toBool();
        //Qt::DockWidgetArea dockarea =
        //    (Qt::DockWidgetArea)settings.value("DockArea").toInt();
        int dock = settings.value("DockArea").toInt();
        
        setupPlugin(pathname, filename, visible, dock);
/*
        QString pfilename = pathname+ "/" + filename + "." + OS::getDLLExtension().c_str();
        bool e1 = boost::filesystem::exists( filename.toStdString() );
        if(!e1){
            pfilename = pathname+ "/" + filename + ".so";
            e1 = boost::filesystem::exists( pfilename.toStdString() );
        }
        if(!e1){
            pfilename = pathname+ "/" + filename + ".dll";
            e1 = boost::filesystem::exists( pfilename.toStdString() );
        }
        if(!e1){
            pfilename = pathname+ "/" + filename + ".dylib";
            e1 = boost::filesystem::exists( pfilename.toStdString() );
        }




        QPluginLoader loader(pfilename);
#if QT_VERSION >= 0x040400
		// Needed to make dynamicly loaded libraries use dynamic
		// cast on each others objects. ONLY on linux though.
		loader.setLoadHints(QLibrary::ResolveAllSymbolsHint |
							QLibrary::ExportExternalSymbolsHint);
#endif


        QObject* pluginObject = loader.instance();
        if (pluginObject != NULL) {
        	RobWorkStudioPlugin* testP = dynamic_cast<RobWorkStudioPlugin*>(pluginObject);
            if (testP == NULL)
                RW_THROW("Loaded plugin is NULL, tried loading \"" << pfilename.toStdString() << "\"" );
            RobWorkStudioPlugin* plugin = qobject_cast<RobWorkStudioPlugin*>(pluginObject);

            if (plugin) {
                addPlugin(plugin, visible, dockarea);
            }
            else
                QMessageBox::information(
                    this,
                    "Unable to load Plugin",
                    pfilename + " was not of type RobWorkStudioPlugin",
                    QMessageBox::Ok);
        } else {
            QMessageBox::information(
                this,
                "Unable to load Plugin",
                pfilename + " was not loaded: \"" + loader.errorString() + "\"",
                QMessageBox::Ok);
        }*/

        settings.endGroup(); //End Specific Plugin Group
    }
    settings.endGroup(); //End the Plugins Group
	
}

void RobWorkStudio::newWorkCell()
{
	try {

		closeWorkCell();
		// Empty workcell constructed.
		_workcell = emptyWorkCell();
		_state = _workcell->getDefaultState();
		_detector = makeCollisionDetector(_workcell);
		// Workcell given to view.
		_view->setWorkCell( _workcell );
		_view->setState(_state);

	} catch (const Exception& exp) {
		QMessageBox::critical(this, tr("RobWorkStudio"), tr("Caught exception while trying to create new work cell: %1").arg(exp.what()));
	}

	// Workcell sent to plugins.
	openAllPlugins();
	updateHandler();
}

void RobWorkStudio::saveWorkCell()
{
  if (_workcell != nullptr)
  {
    std::string wcFilePath = static_cast<std::string>(_workcell->getPropertyMap().get<std::string>(rw::loaders::XMLRWLoader::getWorkCellFileNameId()));
    QString wcFileName = QFileDialog::getSaveFileName(this,
                                            tr("Save Workcell"), QString::fromStdString(wcFilePath), tr("RobWork Workcell (*.wc.xml)"));

    rw::loaders::DOMWorkCellSaver::save(_workcell, _state, wcFileName.toStdString());
  }
}

void RobWorkStudio::dragMoveEvent(QDragMoveEvent *event)
{
    event->accept();
}

void RobWorkStudio::dragEnterEvent(QDragEnterEvent* event)
{
    if (event->mimeData()->hasText()){
        event->acceptProposedAction();
    } else if (event->mimeData()->hasHtml()) {
        event->acceptProposedAction();
    } else if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }
    else {
        event->ignore();
    }

}

void RobWorkStudio::dropEvent(QDropEvent* event)
{
    if (event->mimeData()->hasUrls()) {
		QList<QUrl> urls = event->mimeData()->urls();
		if (urls.size() == 1) {
			openFile(urls[0].toLocalFile().toStdString());
		}
    } else if (event->mimeData()->hasHtml()) {
        Log::debugLog() << "html dropped: "  << std::endl;

    } else if (event->mimeData()->hasText()) {
        QString text = event->mimeData()->text();
        Log::infoLog() << text.toStdString() << std::endl;
    } else {
        event->ignore();
    }
}


void RobWorkStudio::openFile(const std::string& file)
{	
    // We change directory irrespective of whether we can load open the file or
    // not. We change directory also if openFile() was called because of a drop
    // event.
    if (!file.empty()){
        _settingsMap->set<std::string>("PreviousOpenDirectory", StringUtil::getDirectoryName(file));
    }

    try {
        const QString filename(file.c_str());
        //std::cout << filename.toStdString() << std::endl;

        if (!filename.isEmpty()) {
            std::vector<std::string> lastfiles = _settingsMap->get<std::vector<std::string> >("LastOpennedFiles", std::vector<std::string>());
            lastfiles.push_back(file);
            if (filename.endsWith(".STL", Qt::CaseInsensitive) ||
                filename.endsWith(".STLA", Qt::CaseInsensitive) ||
                filename.endsWith(".STLB", Qt::CaseInsensitive) ||
#if RW_HAVE_ASSIMP
                filename.endsWith(".DAE", Qt::CaseInsensitive) ||
#endif
                filename.endsWith(".3DS", Qt::CaseInsensitive) ||
                filename.endsWith(".AC", Qt::CaseInsensitive) ||
                filename.endsWith(".AC3D", Qt::CaseInsensitive) ||
                filename.endsWith(".TRI", Qt::CaseInsensitive) ||
                filename.endsWith(".OBJ", Qt::CaseInsensitive))
            {
                Log::infoLog() << "Opening drawable file: " << filename.toStdString() << "\n";
                openDrawable(filename);
                _settingsMap->set<std::vector<std::string> >("LastOpennedFiles", lastfiles);
                updateLastFiles();
            } else if (filename.endsWith(".WU", Qt::CaseInsensitive)  ||
                       filename.endsWith(".WC", Qt::CaseInsensitive)  ||
                       filename.endsWith(".DEV", Qt::CaseInsensitive) ||
                       filename.endsWith(".XML", Qt::CaseInsensitive) )
            {
                Log::infoLog() << "Opening workcell file: " << filename.toStdString() << "\n";
                openWorkCellFile(filename);
                _settingsMap->set<std::vector<std::string> >("LastOpennedFiles", lastfiles);
                updateLastFiles();
            } else {
                // we try openning a workcell

                openWorkCellFile(filename);
                /*
                Log::infoLog() << "Failed loading file: " << filename.toStdString() << "\n";
                QMessageBox::information(
                    NULL,
                    "Unknown extension",
                    "The file specified has an unknown extension type!",
                    QMessageBox::Ok);
                    */
            }
        }
    }

    catch (const rw::common::Exception& exp) {

        QMessageBox::information(
            NULL,
            "Exception",
            exp.getMessage().getText().c_str(),
            QMessageBox::Ok);

        //closeWorkCell();
    }
    //std::cout << "Update handler!" << std::endl;
    updateHandler();
}

void RobWorkStudio::open()
{
    QString selectedFilter;

    std::string previousOpenDirectory = _settingsMap->get<std::string>("PreviousOpenDirectory","");
    const QString dir(previousOpenDirectory.c_str());
	
	QString assimpExtensions = "";
#if RW_HAVE_ASSIMP
	assimpExtensions = " * .dae";
#endif

    QString filename = QFileDialog::getOpenFileName(
        this,
        "Open WorkCell or Drawable", // Title
        dir, // Directory
        "All supported ( *.wu *.wc *.xml *.wc.xml *.dev *.stl *.stla *.stlb *.3ds *.ac *.ac3d *.obj" + assimpExtensions + ")"
        "\nTUL files ( *.wu *.wc *.dev )"
        "\nRW XML files ( *.wc.xml *.xml)"
        "\nDrawables ( *.stl *.stla *.stlb *.3ds *.ac *.ac3d *.obj" + assimpExtensions + ")"
        "\n All ( *.* )",
        &selectedFilter);

    openFile(filename.toStdString());
}

void RobWorkStudio::openDrawable(const QString& filename)
{
    try {
        _view->getWorkCellScene()->addDrawable( filename.toStdString(), _workcell->getWorldFrame());
    } catch(...){
        const std::string msg =
            "Failed to load " + filename.toStdString() + " as a Drawable";
        QMessageBox::information(this, "Error", msg.c_str(), QMessageBox::Ok);
    }

}

void RobWorkStudio::openWorkCellFile(const QString& filename)
{

    // Always close the workcell.
    closeWorkCell();
    //rw::graphics::WorkCellScene::Ptr wcsene = _view->makeWorkCellScene();

    WorkCell::Ptr wc;

    try{
        wc = WorkCellLoader::Factory::load(filename.toStdString());
        if(wc==NULL){
            RW_THROW("Loading of workcell failed!");
        }
    } catch( const std::exception& e){
        const std::string msg =
            "Failed to load workcell: " + filename.toStdString() + ". \n " + std::string(e.what());
        QMessageBox::information(this, "Error", msg.c_str(), QMessageBox::Ok);
        wc = emptyWorkCell();
    }

    //std::cout<<"Number of devices in workcell in RobWorkStudio::setWorkCell: "<<workcell->getDevices().size()<<std::endl;
    // don't set any variables before we know they are good

    CollisionDetector::Ptr detector = makeCollisionDetector(wc);
    _workcell = wc;
    _state = _workcell->getDefaultState();
    _detector = detector;
    _view->setWorkCell(wc);
    _view->setState(_state);


    openAllPlugins();
}

void RobWorkStudio::setWorkcell(rw::models::WorkCell::Ptr workcell)
{

    // Always close the workcell.
    if (_workcell && _workcell!=_workcell)
		closeWorkCell();

    // Open a new workcell if there is one.<
    if (workcell) {

        //std::cout<<"Number of devices in workcell in RobWorkStudio::setWorkCell: "<<workcell->getDevices().size()<<std::endl;
        // don't set any variables before we know they are good
		CollisionDetector::Ptr detector = makeCollisionDetector(workcell);

        _workcell = workcell;
        _state = _workcell->getDefaultState();
        _detector = detector;
        _view->setWorkCell(_workcell);
        _view->setState(_state);

        openAllPlugins();
    }	
}

rw::models::WorkCell::Ptr RobWorkStudio::getWorkcell(){
    return _workcell;
}

void RobWorkStudio::closeWorkCell()
{
    // Clear everything from the view
    _view->clear();

    // Call close on all modules
    closeAllPlugins();

    updateHandler();
}

void RobWorkStudio::showSolidTriggered()
{
    updateHandler();
}

void RobWorkStudio::showWireTriggered()
{
    updateHandler();
}

void RobWorkStudio::showBothTriggered()
{
    updateHandler();
}

void RobWorkStudio::updateViewHandler()
{
    _view->update();
}

void RobWorkStudio::updateHandler()
{
    update();
    _view->update();
}

/*
void RobWorkStudio::fireStateTrajectoryChangedEvent(const rw::trajectory::TimedStatePath& trajectory)
{

    _timedStatePath = trajectory;
    BOOST_FOREACH(
        const StateTrajectoryChangedEvent::Listener& listener,
        stateTrajectoryChangedEvent().getListeners()) {
        listener.callback(trajectory);
    }
}
*/
void RobWorkStudio::setTStatePath(rw::trajectory::TimedStatePath path){
    _timedStatePath = path;
    stateTrajectoryChangedEvent().fire(_timedStatePath);
}

namespace {

	class RobWorkStudioEvent: public QEvent {
    public:
        static const QEvent::Type SetStateEvent = (QEvent::Type)1200;
        static const QEvent::Type SetTimedStatePathEvent = (QEvent::Type)1201;
        static const QEvent::Type UpdateAndRepaintEvent = (QEvent::Type)1202;
        static const QEvent::Type SaveViewGLEvent = (QEvent::Type)1203;
        static const QEvent::Type ExitEvent = (QEvent::Type)1204;
        static const QEvent::Type SetWorkCell = (QEvent::Type)1205;
        static const QEvent::Type OpenWorkCell = (QEvent::Type)1206;
        static const QEvent::Type GenericEvent = (QEvent::Type)1207;
        static const QEvent::Type GenericAnyEvent = (QEvent::Type)1208;

        boost::any _anyData;
        rw::common::Ptr<bool> _hs;

        RobWorkStudioEvent(QEvent::Type type, rw::common::Ptr<bool> hs, boost::any adata):
        	QEvent(type),_anyData(adata),_hs( hs )
        {
        }

        RobWorkStudioEvent(QEvent::Type type, rw::common::Ptr<bool> hs):
        	QEvent(type),_anyData(NULL),_hs( hs )
        {
        }

        virtual ~RobWorkStudioEvent(){
        	Log::debugLog() << "RobWorkStudioEvent: destruct" << std::endl;
            done();
        }

        void done(){

            if(_hs!=NULL){
            	Log::debugLog() << "Done: " << std::endl;
            	Log::debugLog() << "set hs "<<std::endl;
                *_hs = true;
                Log::debugLog() << "hs " << *_hs <<std::endl;
            } else {
            	Log::debugLog() << "Done: hs==NULL" << std::endl;
            }
        }


    };

    class RobWorkStudioEventHS {
    public:
		RobWorkStudioEventHS(QEvent::Type type, boost::any adata):
        	_hs( ownedPtr( new bool(false)) )
        {
			event = new RobWorkStudioEvent(type, _hs, adata);
        }

        RobWorkStudioEventHS(QEvent::Type type):
        	_hs( ownedPtr( new bool(false)))
        {
        	event = new RobWorkStudioEvent(type, _hs);
        }


        void wait(){
        	int cnt = 0;
            while(_hs!=NULL && *_hs==false && cnt<100){
            	TimerUtil::sleepMs(5);
            	//std::cout << "Wait1: " << *_hs << std::endl;
            	cnt++;
            }
            //std::cout << "Wait done: " << std::endl;
        }

        rw::common::Ptr<bool> _hs;
        RobWorkStudioEvent *event;

    };

}

void RobWorkStudio::setTimedStatePath(const rw::trajectory::TimedStatePath& path)
{
    _timedStatePath = path;
    stateTrajectoryChangedEvent().fire(_timedStatePath);
}

void RobWorkStudio::setState(const rw::kinematics::State& state)
{
    _state = state;
    _view->setState(state);
    stateChangedEvent().fire(_state);
    updateHandler();
}

void RobWorkStudio::postTimedStatePath(const rw::trajectory::TimedStatePath& path){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::SetTimedStatePathEvent, path);
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postExit(){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::ExitEvent, NULL);
	QApplication::postEvent( this, event->event );
	//event->wait();
}

void RobWorkStudio::postState(const rw::kinematics::State& state){
    RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::SetStateEvent, state) ;
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postUpdateAndRepaint(){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::UpdateAndRepaintEvent);
    QApplication::postEvent( this, event->event);
    event->wait();
}

void RobWorkStudio::postSaveViewGL(const std::string& filename){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::SaveViewGLEvent, filename);
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postWorkCell(rw::models::WorkCell::Ptr workcell){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::SetWorkCell, workcell);
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postOpenWorkCell(const std::string& filename){
    RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::OpenWorkCell, filename);
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postGenericEvent(const std::string& id){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::GenericEvent, id);
    QApplication::postEvent( this, event->event );
    event->wait();
}

void RobWorkStudio::postGenericAnyEvent(const std::string& id, boost::any data){
	RobWorkStudioEventHS *event = new RobWorkStudioEventHS(RobWorkStudioEvent::GenericAnyEvent, std::make_pair(id,data));
    QApplication::postEvent( this, event->event );
    event->wait();
}

bool RobWorkStudio::event(QEvent *event)
{
	//RobWorkStudioEvent *rwse_test = dynamic_cast<RobWorkStudioEvent *>(event);
    //if(rwse_test !=NULL)
	//	std::cout << "EVENT: " << event->type() << std::endl;
    //   rwse->done();


	// WARNING: only use this pointer if you know its the right type
    RobWorkStudioEvent *rwse = static_cast<RobWorkStudioEvent *>(event);
	if (event->type() == RobWorkStudioEvent::SetStateEvent ) {
        State state = boost::any_cast<State>(rwse->_anyData);
        setState( state );
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::SetTimedStatePathEvent) {
        TimedStatePath tstate = boost::any_cast<TimedStatePath>(rwse->_anyData);
        setTimedStatePath( tstate );
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::UpdateAndRepaintEvent) {
        updateAndRepaint();
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::SaveViewGLEvent) {
        std::string str = boost::any_cast<std::string>(rwse->_anyData);
        try{
            saveViewGL(QString( str.c_str() ));
        } catch (const Exception& exp) {
            QMessageBox::critical(NULL, "Save View", tr("Failed to grab and save view with message '%1'").arg(exp.what()));
        } catch (std::exception& e) {
            QMessageBox::critical(NULL, "Save View", tr("Failed to grab and save view with message '%1'").arg(e.what()));
        } catch ( ... ){
            QMessageBox::critical(NULL, "Save View", tr("Failed to grab and save view"));

        }
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::SetWorkCell){
        WorkCell::Ptr wc = boost::any_cast<WorkCell::Ptr>(rwse->_anyData);
        setWorkCell( wc);
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::OpenWorkCell){
        std::string str = boost::any_cast<std::string>(rwse->_anyData);
        openWorkCellFile( str.c_str() );
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::GenericEvent){
        std::string id = boost::any_cast<std::string>(rwse->_anyData);
        this->genericEvent().fire(id);
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::GenericAnyEvent){
        std::pair<std::string,boost::any> data = boost::any_cast<std::pair<std::string,boost::any> >(rwse->_anyData);
        this->genericAnyEvent().fire(data.first,data.second);
        rwse->done();
        return true;
    } else if (event->type() == RobWorkStudioEvent::ExitEvent){
        closeWorkCell();
        rwse->done();
        close();
        //QCoreApplication::exit(1);
        //abort();
        return true;
    } else {
        //event->ignore();
    }

    return QMainWindow::event(event);
}


void RobWorkStudio::saveViewGL(const QString& filename) {
    _view->saveBufferToFile(filename);
}

namespace {
    class AnyEventListener {
    public:
        AnyEventListener(const std::string& myid):_id(myid),_eventSuccess(false){}
        void cb(const std::string& id, boost::any data){
        	//std::cout << "Any event recieved in CALLBACK!!!!" << std::endl;
            if(!_eventSuccess){
                if(_id==id){
                    _data = data;
                    _eventSuccess=true;
                }
            }
        }
        std::string _id;
        boost::any _data;
        bool _eventSuccess;
    };
}

boost::any RobWorkStudio::waitForAnyEvent(const std::string& id, double timeout){
	//std::cout << " Wait for ANY event, with id: " << id << std::endl;
    AnyEventListener listener(id);
    genericAnyEvent().add( boost::bind(&AnyEventListener::cb, &listener, _1, _2), &listener );
    //std::cout << "Added event, now wait!" << std::endl;
    // now wait until event is called
    const double starttime = TimerUtil::currentTime();
    bool reachedTimeout = false;
    while(!listener._eventSuccess){
        TimerUtil::sleepMs(10);
        if((timeout>0.0) && (TimerUtil::currentTime()-starttime>timeout)){
            reachedTimeout = true;
            break;
        }
    }
    // remove the listener from the event
    genericAnyEvent().remove( &listener );
    // now return result
    if(reachedTimeout)
        RW_THROW("Timeout!");
    return listener._data;
}
