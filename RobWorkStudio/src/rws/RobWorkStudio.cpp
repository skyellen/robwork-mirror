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
#include <QIcon>
#include <QAction>
#include <QActionGroup>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStringList>
#include <QPluginLoader>
#include <QObject>

#include "RobWorkStudio.hpp"

#include "ViewGL.hpp"

#include <rw/common/os.hpp>
#include <rw/common/Exception.hpp>
#include <rwlibs/drawable/DrawableFactory.hpp>

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>


#include <rw/common/StringUtil.hpp>
#include <rw/common/Exception.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/Accessor.hpp>
#include <RobWorkConfig.hpp>
#include <boost/shared_ptr.hpp>

#include <sstream>

using namespace rw;
using namespace rw::common;

using namespace rw::loaders;
using namespace rw::math;
using namespace rwlibs::drawable;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::proximitystrategies;

using namespace rws;


/*
void RobWorkStudio::sendAllMessages(
    std::string pluginName,
    std::string id,
    rw::common::Message msg)
{
}
*/

RobWorkStudio::RobWorkStudio(RobWork::Ptr robwork,
                             const std::vector<PluginSetup>& plugins,
                             const PropertyMap& map,
                             const std::string& inifile)
    :
    //_stateChangedEvent(boost::bind(&RobWorkStudio::fireStateChangedEvent, this, _1)),
    //_frameSelectedEvent(boost::bind(&RobWorkStudio::fireFrameSelectedEvent, this, _1)),
    //_genericEvent(boost::bind(&RobWorkStudio::fireGenericEvent, this, _1)),
    //_keyEvent(boost::bind(&RobWorkStudio::fireKeyEvent, this, _1, _2)),
    //_stateTrajectoryChangedEvent(boost::bind(&RobWorkStudio::fireStateTrajectoryChangedEvent, this, _1)),
    //_positionSelectedEvent(boost::bind(&RobWorkStudio::firePositionSelectedEvent, this, _1)),
    //_mousePressedEvent(boost::bind(&RobWorkStudio::fireMousePressedEvent, this, _1)),
    _robwork(robwork),
    _inStateUpdate(false),
    _propMap(map),
    _settingsMap(NULL)
{
    std::stringstream sstr;
    sstr << " RobWorkStudio v" << RW_VERSION;
    QString qstr(sstr.str().c_str());
    setWindowTitle( qstr );
    setWindowIcon( *(new QIcon(":/images/rw_logo_64x64.png") ) );
    _aboutBox = new AboutBox(RW_VERSION, RW_REVISION, this);


    PropertyMap settings;
    try {
    	settings = XMLPropertyLoader::load("rwsettings.xml");
    	_propMap.set<std::string>("SettingsFileName", "rwsettings.xml");
    } catch(rw::common::Exception &e){
    	RW_WARN("Could not load settings from 'rwsettings.xml': " << e.getMessage().getText() << "\n Using default settings!");
    } catch(std::exception &e){
        RW_WARN("Could not load settings from 'rwsettings.xml': " << e.what() << "\n Using default settings!");
        // loading failed so we just go on with an empty map
    }

    PropertyMap *currentSettings = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");
    if(currentSettings==NULL){
        _propMap.add("RobWorkStudioSettings", "Settings for RobWorkStudio", settings);
        currentSettings = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");
    } else {
        *currentSettings = settings;
    }

    _settingsMap = _propMap.getPtr<PropertyMap>("RobWorkStudioSettings");

    // set the drag and drop property to true
    setAcceptDrops(TRUE);
    setupFileActions();
    setupViewGL();

    _propEditor = new PropertyViewEditor(NULL);
    _propEditor->setPropertyMap( &_propMap  );

    _pluginsMenu = menuBar()->addMenu(tr("&Plugins"));
    _pluginsToolBar = addToolBar(tr("Plugins"));

    setupHelpMenu();


    int width = _settingsMap->get<int>("WindowWidth", 1024);
    int height = _settingsMap->get<int>("WindowHeight", 800);

    resize(width, height);

    QPoint pos = this->pos();
    pos.setX( _settingsMap->get<int>("WindowPosX", this->pos().x()) );
    pos.setY( _settingsMap->get<int>("WindowPosY", this->pos().y()) );


	//Initialize plugins
	loadSettingsSetupPlugins(inifile);
    BOOST_FOREACH(const PluginSetup& plugin, plugins) {
        addPlugin(plugin.plugin, plugin.visible, plugin.area);
    }

	newWorkCell();
}

RobWorkStudio::~RobWorkStudio()
{
    close();

    _settingsMap->set<int>("WindowPosX", this->pos().x());
    _settingsMap->set<int>("WindowPosY", this->pos().y());

    _settingsMap->set<int>("WindowWidth", this->width());
    _settingsMap->set<int>("WindowHeight", this->height());

    _settingsMap->set<bool>("CheckForCollision", _view->isCheckForCollisionEnabled() );

	try {
		XMLPropertySaver::save(*_settingsMap, "rwsettings.xml");
	} catch(const rw::common::Exception& e) {
		std::cout << "Error saving settings file: " << e << std::endl;
	} catch(...) {
		std::cout << "Error saving settings file due to unknown exception!" << std::endl;
	}

    //std::cout<<"Ready to delete plugins"<<std::endl;
    typedef std::vector<RobWorkStudioPlugin*>::iterator I;
    for (I it = _plugins.begin(); it != _plugins.end(); ++it) {
        delete *it;
    }
}

//typedef boost::function<void(PropertyBase*)> PropertyChangedListener;
void RobWorkStudio::propertyChangedListener(PropertyBase* base){
    std::string id = base->getIdentifier();
    std::cout << "Property Changed Listerner RWSTUDIO: " << id << std::endl;

/*
    if(id=="CheckForCollision"){
        Property<bool> *p = toProperty<bool>(base);
        if(p!=NULL)
            _view->setCheckForCollision( p->getValue() );
    } else if(id=="ShowCollisionModels"){
        std::cout << "ShowCollisionModels" << std::endl;
        Property<bool> *p = toProperty<bool>(base);
        if(p==NULL)
            return;
        if(p->getValue()) _view->setDrawableMask( Drawable::CollisionObject | Drawable::Virtual );
        else _view->setDrawableMask( Drawable::DrawableObject | Drawable::Physical | Drawable::Virtual );
    }
    */
}



void RobWorkStudio::closeEvent( QCloseEvent * e ){
    
	// save the settings of each plugin
    BOOST_FOREACH(RobWorkStudioPlugin* plugin, _plugins){

        bool visible = plugin->isVisible();

        bool floating = plugin->isFloating();
        int intarea = (int) dockWidgetArea(plugin);
        std::string pname = plugin->name().toStdString();
        _settingsMap->set<bool>( std::string("PluginVisible_")+ pname , visible);
        _settingsMap->set<bool>( std::string("PluginFloating_")+pname , floating);
        _settingsMap->set<int>( std::string("PluginArea_")+ pname , intarea);
    }

	_propEditor->close();

    // now call accept
    e->accept();
	
}


rw::common::Log& RobWorkStudio::log(){
    return _robwork->getLog();  	
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
    connect(closeAction, SIGNAL(triggered()), this, SLOT(close()));

    QToolBar* fileToolBar = addToolBar(tr("File"));
    fileToolBar->addAction(newAction);
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(closeAction);

    QMenu* pFileMenu = menuBar()->addMenu(tr("&File"));
    pFileMenu->addAction(newAction);
    pFileMenu->addAction(openAction);
    pFileMenu->addAction(closeAction);


    QAction* propertyAction =
        new QAction(tr("&Preferences"), this); // owned
    connect(propertyAction, SIGNAL(triggered()), this, SLOT(showPropertyEditor()));

    pFileMenu->addAction(propertyAction);
}


void RobWorkStudio::showPropertyEditor(){
    // start property editor
   _propEditor->show();
}

void RobWorkStudio::setupHelpMenu() {
    QAction* showAboutBox = new QAction("About",this);
    connect(showAboutBox, SIGNAL(triggered()), _aboutBox, SLOT(exec()));
    QMenu* pHelpMenu = menuBar()->addMenu(tr("Help"));
    pHelpMenu->addAction(showAboutBox);
}

void RobWorkStudio::setupViewGL()
{	
    _view = new ViewGL(this);
    setCentralWidget(_view); // own view
    _view->setupMenu(menuBar()->addMenu(tr("&View")));
    _view->setupToolBar(addToolBar(tr("View")));
    //_propMap.set("ViewGL", _view->getPropertyMap());
	
}


void RobWorkStudio::openAllPlugins()
{
    typedef std::vector<RobWorkStudioPlugin*>::iterator I;
    for (I p = _plugins.begin(); p != _plugins.end(); ++p){
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
	std::cout<<"Number of devices in workcell in RobWorkStudio::openPlugin: "<<plugin.name().toStdString()<<" = "<< _workcell->getDevices().size()<<std::endl;
    try {
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
    plugin->setupMenu(_pluginsMenu);
    plugin->setupToolBar(_pluginsToolBar);
    plugin->setConvert(&_converter);
    plugin->setRobWorkStudio(this);
    plugin->setRobWorkInstance(_robwork);
    plugin->setLog( Log::getInstance() );
    plugin->initialize();

 //   connect(plugin, SIGNAL(updateSignal()), this, SLOT(updateHandler()));
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
}


QSettings::Status RobWorkStudio::loadSettingsSetupPlugins(const std::string& file)
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
    return settings.status();

}



void RobWorkStudio::setupPlugins(QSettings& settings)
{
    QStringList groups = settings.childGroups();

    settings.beginGroup("Plugins");
    QStringList plugins = settings.childGroups();
    for (int i = 0; i<plugins.size(); i++) {
        const QString& pluginname = plugins.at(i);


        //std::cout << "Plugin = " << pluginname.toStdString() << "\n";

        settings.beginGroup(pluginname);

        QString pathname = settings.value("Path").toString();
        QString filename = settings.value("Filename").toString();
        bool visible = settings.value("Visible").toBool();
        Qt::DockWidgetArea dockarea =
            (Qt::DockWidgetArea)settings.value("DockArea").toInt();

        filename = pathname+ "/" + filename + "." + OS::getDLLExtension().c_str();

        QPluginLoader loader(filename);
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
                RW_THROW("Loaded plugin is NULL, tried loading \"" << filename.toStdString() << "\"" );
            RobWorkStudioPlugin* plugin = qobject_cast<RobWorkStudioPlugin*>(pluginObject);

            if (plugin) {
                addPlugin(plugin, visible, dockarea);
            }
            else
                QMessageBox::information(
                    this,
                    "Unable to load Plugin",
                    filename + " was not of type RobWorkStudioPlugin",
                    QMessageBox::Ok);
        } else {
            QMessageBox::information(
                this,
                "Unable to load Plugin",
                filename + " was not loaded: \"" + loader.errorString() + "\"",
                QMessageBox::Ok);
        }

        settings.endGroup(); //End Specific Plugin Group
    }
    settings.endGroup(); //End the Plugins Group
	
}



namespace
{
	WorkCell::Ptr emptyWorkCell()
    {
		WorkCell::Ptr workcell = rw::common::ownedPtr(new WorkCell(new StateStructure()));
        Accessor::collisionSetup().set(*workcell->getWorldFrame(), CollisionSetup());
        return workcell;
    }

	CollisionDetector::Ptr makeCollisionDetector(WorkCell::Ptr workcell)
    {
        return rw::common::ownedPtr(
            new CollisionDetector(
                workcell,
                ProximityStrategyFactory::makeDefaultCollisionStrategy()
                //ProximityStrategyYaobi::make()
        ));
    }
}

void RobWorkStudio::newWorkCell()
{
    close();

    // Empty workcell constructed.
    _workcell = emptyWorkCell();
    _state = _workcell->getDefaultState();
    _converter = Convert(_workcell.get());
    _detector = makeCollisionDetector(_workcell);

    // Workcell given to view.

    _view->addWorkCell(_workcell.get(), &_state, _detector.get());

    // Workcell sent to plugins.
    openAllPlugins();
    updateHandler();
	
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
    if (event->mimeData()->hasText()) {
        QString text = event->mimeData()->text();
        Log::infoLog() << text.toStdString() << std::endl;
    } else if (event->mimeData()->hasHtml()) {
        // std::cout << "html dropped: "  << std::endl;
    } else if (event->mimeData()->hasUrls()) {
        QList<QUrl> urls = event->mimeData()->urls();
        if (urls.size() == 1) {
            openFile(urls[0].toLocalFile().toStdString());
        }
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

        if (!filename.isEmpty()) {

            if (filename.endsWith(".STL", Qt::CaseInsensitive) ||
                filename.endsWith(".STLA", Qt::CaseInsensitive) ||
                filename.endsWith(".STLB", Qt::CaseInsensitive) ||
                filename.endsWith(".3DS", Qt::CaseInsensitive) ||
                filename.endsWith(".AC", Qt::CaseInsensitive) ||
                filename.endsWith(".AC3D", Qt::CaseInsensitive) ||
                filename.endsWith(".TRI", Qt::CaseInsensitive) ||
                filename.endsWith(".OBJ", Qt::CaseInsensitive))
            {
                openDrawable(filename);
            } else if (filename.endsWith(".WU", Qt::CaseInsensitive)  ||
                       filename.endsWith(".WC", Qt::CaseInsensitive)  ||
                       filename.endsWith(".DEV", Qt::CaseInsensitive) ||
                       filename.endsWith(".XML", Qt::CaseInsensitive) )
            {
                openWorkCellFile(filename);
            } else {
                QMessageBox::information(
                    NULL,
                    "Unknown extension",
                    "The file specified has an unknown extension type!",
                    QMessageBox::Ok);
            }
        }
    }

    catch (const rw::common::Exception& exp) {
    	std::cout << "Exception was caught!" << std::endl;
        QMessageBox::information(
            NULL,
            "Exception",
            exp.getMessage().getText().c_str(),
            QMessageBox::Ok);
        std::cout << "Closing application!" << std::endl;
        close();

    }

    //std::cout << "Update handler!" << std::endl;
    updateHandler();
}

void RobWorkStudio::open()
{
    QString selectedFilter;

    std::string previousOpenDirectory = _settingsMap->get<std::string>("PreviousOpenDirectory","");
    const QString dir(previousOpenDirectory.c_str());

    QString filename = QFileDialog::getOpenFileName(
        this,
        "Open Drawable", // Title
        dir, // Directory
        "All supported ( *.wu *.wc *.xml *.dev *.stl *.stla *.stlb *.3ds *.ac *.ac3d *.obj)"
        " \nTUL files ( *.wu *.wc *.dev )"
        " \nRW XML files ( *.xml )"
        " \nDrawables ( *.stl *.stla *.stlb *.3ds *.ac *.ac3d *.obj)"
        " \n All ( *.* )",
        &selectedFilter);

    openFile(filename.toStdString());
}

void RobWorkStudio::openDrawable(const QString& filename)
{
    Drawable::Ptr drawable = DrawableFactory::loadDrawableFile(filename.toStdString());

    if (drawable) {
        _drawables.push_back(drawable);
        _view->addDrawable(drawable);
    }
    else {
        const std::string msg =
            "Failed to load " + filename.toStdString() + " as a Drawable";
        QMessageBox::information(this, "Error", msg.c_str(), QMessageBox::Ok);
    }
}

void RobWorkStudio::openWorkCellFile(const QString& filename)
{
    setWorkcell(WorkCellLoader::load(filename.toStdString()));
}

void RobWorkStudio::setWorkcell(rw::models::WorkCell::Ptr workcell)
{
    // Always close the workcell.
    if (_workcell) 
		close();

    // Open a new workcell if there is one.<
	std::cout<<"WorkCell = "<<workcell.get()<<std::endl;
    if (workcell) {
		std::cout<<"Number of devices in workcell in RobWorkStudio::setWorkCell: "<<workcell->getDevices().size()<<std::endl;
        // don't set any variables before we know they are good
		CollisionDetector::Ptr detector = makeCollisionDetector(workcell);

        _workcell = workcell;
        _state = _workcell->getDefaultState();
        _detector = detector;
        _converter = Convert(_workcell.get());
        _view->addWorkCell(_workcell.get(), &_state, _detector.get());
        openAllPlugins();
    }	
}

rw::models::WorkCell::Ptr RobWorkStudio::getWorkcell(){
    return _workcell;
}

void RobWorkStudio::close()
{
    // Clear everything from the view
    _view->clear();

    // Call close on all modules
    closeAllPlugins();

    // Delete all loaded Drawables
    {
        //typedef std::vector<Drawable*>::iterator DI;
        //for (DI it = _drawables.begin(); it != _drawables.end(); ++it)
        //    delete *it;
        _drawables.clear();
    }

    _workcellGLDrawer.clearCache();

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
    _view->updateGL();
}

void RobWorkStudio::updateHandler()
{
    update();
    _view->updateGL();
}


void RobWorkStudio::fireStateTrajectoryChangedEvent(const rw::trajectory::TimedStatePath& trajectory)
{

    _timedStatePath = trajectory;
    BOOST_FOREACH(
        const StateTrajectoryChangedEvent::Listener& listener,
        stateTrajectoryChangedEvent().getListeners()) {
        listener.callback(trajectory);
    }
}

void RobWorkStudio::setTStatePath(rw::trajectory::TimedStatePath path){
    std::cout << "Set t state path" << std::endl;
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

        //static QEvent::Type SetStateEvent = 1200;
        RobWorkStudioEvent(QEvent::Type type):QEvent(type){}

        RobWorkStudioEvent(QEvent::Type type, const std::string& string):
            QEvent(type), _str(string){}

        RobWorkStudioEvent(const State& state):
            QEvent(SetStateEvent)
        {
            _data = ownedPtr( new rw::common::Property<State>("State","",state) );
        }

        RobWorkStudioEvent(const TimedStatePath& path):
            QEvent(SetTimedStatePathEvent)
        {
            _data = ownedPtr( new rw::common::Property<TimedStatePath>("TimedStatePath","",path) );
        }
		rw::common::PropertyBase::Ptr _data;
        std::string _str;
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
    stateChangedEvent().fire(_state);
    updateHandler();
}

void RobWorkStudio::postTimedStatePath(const rw::trajectory::TimedStatePath& path){
    QApplication::postEvent( this, new RobWorkStudioEvent(path) );
}

void RobWorkStudio::postState(const rw::kinematics::State& state){
    QApplication::postEvent( this, new RobWorkStudioEvent(state) );
}

void RobWorkStudio::postUpdateAndRepaint(){
    QApplication::postEvent( this, new RobWorkStudioEvent(RobWorkStudioEvent::UpdateAndRepaintEvent) );
}

void RobWorkStudio::postSaveViewGL(const std::string& filename){
    QApplication::postEvent( this, new RobWorkStudioEvent(RobWorkStudioEvent::SaveViewGLEvent, filename) );
}

bool RobWorkStudio::event(QEvent *event)
{
    if (event->type() == RobWorkStudioEvent::SetStateEvent ) {
        RobWorkStudioEvent *rwse = static_cast<RobWorkStudioEvent *>(event);
        Property<State> *p = toProperty<State>( rwse->_data.get() );
        if (p!=NULL) {
            setState( p->getValue() );
        }
        return true;
    } else if (event->type() == RobWorkStudioEvent::SetTimedStatePathEvent) {
        RobWorkStudioEvent *rwse = static_cast<RobWorkStudioEvent *>(event);
        Property<TimedStatePath> *p = toProperty<TimedStatePath>( rwse->_data.get() );
        if (p!=NULL) {
            setTimedStatePath( p->getValue() );
        }
        return true;
    } else if (event->type() == RobWorkStudioEvent::UpdateAndRepaintEvent) {
        updateAndRepaint();
        return true;
    } else if (event->type() == RobWorkStudioEvent::SaveViewGLEvent) {
        RobWorkStudioEvent *rwse = static_cast<RobWorkStudioEvent *>(event);
        saveViewGL(QString( rwse->_str.c_str() ));
        return true;
    }



    return QWidget::event(event);
}


void RobWorkStudio::saveViewGL(const QString& filename) {
    _view->saveBufferToFile(filename);
}
