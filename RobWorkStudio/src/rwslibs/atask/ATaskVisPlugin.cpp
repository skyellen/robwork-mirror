#include "ATaskVisPlugin.hpp"

#include "StrategyLibraryDialog.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rwlibs/opengl/RenderForceTorque.hpp>
#include <rwlibs/opengl/RenderPointCloud.hpp>
#include <rwlibs/opengl/RenderLines.hpp>
#include <rwlibs/assembly/AssemblyResult.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/task/Task.hpp>

#include <rws/RobWorkStudio.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>

#include <QFileDialog>

#include <boost/bind.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rwlibs::assembly;
using namespace rwlibs::opengl;
using namespace rwlibs::task;

using namespace rwslibs;

ATaskVisPlugin::ATaskVisPlugin():
    RobWorkStudioPlugin("ATaskVisPlugin", QIcon(":/assemblytaskvisplugin/atask_icon.png")),
    _wc(NULL),
    _showReal(true)
{
    setupUi(this);
    _editor = new PropertyViewEditor(this);
    _taskBoxLayout->addWidget(_editor);

    connect(_loadTasksBtn,       SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_loadResultsBtn,     SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btnStrategyLibrary, SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_real,               SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_assumed,            SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_assumed,            SIGNAL(toggled(bool)), this, SLOT(btnPressed()) );
}

ATaskVisPlugin::~ATaskVisPlugin() {
}

void ATaskVisPlugin::initialize() {
    getRobWorkStudio()->genericAnyEvent().add(
          boost::bind(&ATaskVisPlugin::genericAnyEventListener, this, _1, _2), this);

    Log::setLog( _log );
}

void ATaskVisPlugin::open(WorkCell* workcell)
{
    if(workcell==NULL)
        return;
    _wc = workcell;
}

void ATaskVisPlugin::btnPressed() {
    QObject *obj = sender();
    if (obj == _loadTasksBtn)
    	loadTasks();
    else if (obj == _loadResultsBtn)
    	loadResults();
    else if (obj == _btnStrategyLibrary) {
    	StrategyLibraryDialog* const dialog = new StrategyLibraryDialog();
    	dialog->setAttribute(Qt::WA_DeleteOnClose);
    	dialog->setWorkCell(getRobWorkStudio()->getWorkCell());
    	dialog->show();
    	dialog->raise();
    	dialog->activateWindow();
    } else if (obj == _real || obj == _assumed) {
    	if (_real->isChecked())
    		_showReal = true;
    	else
    		_showReal = false;
    	if (_currentTask != NULL && _currentResult != NULL && _wc != NULL)
    		constructPlayback();
    }
}

void ATaskVisPlugin::genericAnyEventListener(const std::string& event, boost::any data){
    // used as control from ither plugins or scripts
    try{
    	if (event == "PlayBack::TimeRelative") {
    		if (_currentTask == NULL)
    			return;
    		double timeRelative = boost::any_cast<double>(data);
    		Path<Timed<AssemblyState> > path;
    		if (_showReal)
    			path = _currentResult->realState;
    		else
    			path = _currentResult->assumedState;
    		double endTime = path.back().getTime();
    		double time = timeRelative*endTime;
    		Timed<AssemblyState>& foundState = path.front();
    		if (time >= endTime) {
    			foundState = path.back();
    		} else {
    			for (std::size_t i = 1; i < path.size()-1; i++) {
    				if (time >= path[i].getTime() && time < path[i+1].getTime()) {
    					foundState = path[i];
    					break;
    				}
    			}
    		}
    		AssemblyState& astate = foundState.getValue();
    		_phase->setText(QString::fromStdString("Phase: " + astate.phase));
    		_timeLabel->setText(QString::fromStdString("Time: ") + QString::number(foundState.getTime()));
    		if (_maleFTrender != NULL) {
    			_maleFTrender->setForce(astate.ftSensorMale.force());
    			_maleFTrender->setTorque(astate.ftSensorMale.torque());
    		}
    		if (_femaleFTrender != NULL) {
    			_femaleFTrender->setForce(astate.ftSensorFemale.force());
    			_femaleFTrender->setTorque(astate.ftSensorFemale.torque());
    		}
    		_ftMaleLabel->setText(QString::fromStdString("Force/Torque (male): ") + QString::number(astate.ftSensorMale.force().norm2()) + QString::fromStdString("N / ") + QString::number(astate.ftSensorMale.torque().norm2()) + QString::fromStdString("Nm"));
    		_ftFemaleLabel->setText(QString::fromStdString("Force/Torque (female): ") + QString::number(astate.ftSensorFemale.force().norm2()) + QString::fromStdString("N / ") + QString::number(astate.ftSensorFemale.torque().norm2()) + QString::fromStdString("Nm"));
    		std::stringstream sstr;
    		sstr << "Contact: ";
    		if (astate.contact)
    			sstr << "Yes";
    		else
    			sstr << "No";
    		if (astate.contacts.size() > 0)
    			sstr << " - " << astate.contacts.size() << " contacts detected - max force is " << astate.maxContactForce.norm2() << "N";
    		_contactLabel->setText(QString::fromStdString(sstr.str()));
    		if (_contactPointRender != NULL) {
    			_contactPointRender->clear();
    			BOOST_FOREACH(const Transform3D<> contact, astate.contacts) {
    				const Vector3D<> p = contact.P();
        			_contactPointRender->addPoint(p);
    			}
    		}
    		if (_contactNormalRender != NULL) {
    			_contactNormalRender->clear();
    			BOOST_FOREACH(const Transform3D<> contact, astate.contacts) {
    				const Vector3D<> p = contact.P();
    				const Vector3D<> n = contact.R().getCol(2);
    				_contactNormalRender->addLine(p,p+0.01*n);
    			}
    		}
    	} else if(event=="AVis::LoadFile"){
            std::string file = boost::any_cast<std::string>(data);
            QMetaObject::invokeMethod( this, "loadTasks", Qt::QueuedConnection, Q_ARG( QString, QString(file.c_str()) ) );
        } else if(event=="AVis::Update"){
            QMetaObject::invokeMethod( this, "updateVis", Qt::QueuedConnection);
        } else if(event=="AVis::SelectGrasp"){
            int grasp = (int)boost::any_cast<double>(data);
            QMetaObject::invokeMethod( this, "selectGrasp", Qt::QueuedConnection, Q_ARG( int, grasp ) );
        }


    } catch (...){
        Log::warningLog() << "ATaskVisPlugin: Event \"" << event << "\" did not have the correct datatype or an error occured!\n";
    }
}

void ATaskVisPlugin::loadTasks(){
    std::string prevDir = settings().get<std::string>("LastDir","");
    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("AssemblyTask", "");
    std::string taskFile = filename;

    QString selectedFilter;
    if(filename=="")
    	filename = prevDir;
    const QString dir(filename.c_str());

    QString file = QFileDialog::getOpenFileName(
    		this,
    		"Open Assembly Task file", // Title
    		dir, // Directory
    		"All supported ( *.xml )"
    		" \nRW Assembly Task files ( *.assembly.xml )"
    		" \n All ( *.* )",
    		&selectedFilter);

    taskFile = file.toStdString();

    if(taskFile=="")
    	return;

    log().info() << "Loading tasks: ";
    log().info() << "\t-Filename: " << taskFile;

    _tasks = AssemblyTask::load(taskFile);
    settings().set<std::string>("LastDir",StringUtil::getDirectoryName(taskFile));

    selectTask(1);
}

void ATaskVisPlugin::loadResults(){
    std::string prevDir = settings().get<std::string>("LastDir","");
    std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("AssemblyResult", "");
    std::string taskFile = filename;

    QString selectedFilter;
    if(filename=="")
    	filename = prevDir;
    const QString dir(filename.c_str());

    QString file = QFileDialog::getOpenFileName(
    		this,
    		"Open Assembly Result file", // Title
    		dir, // Directory
    		"All supported ( *.xml )"
    		" \nRW Assembly Result files ( *.assembly.xml )"
    		" \n All ( *.* )",
    		&selectedFilter);

    taskFile = file.toStdString();

    if(taskFile=="")
    	return;

    log().info() << "Loading results: ";
    log().info() << "\t-Filename: " << taskFile;

    _results = AssemblyResult::load(taskFile);
    settings().set<std::string>("LastDir",StringUtil::getDirectoryName(taskFile));

    selectResult(1);
}

void ATaskVisPlugin::selectTask(std::size_t i) {
	_currentTask = _tasks[i-1];

	getRobWorkStudio()->getWorkCellScene()->removeDrawable("MaleFTRender");
	getRobWorkStudio()->getWorkCellScene()->removeDrawable("FemaleFTRender");
	getRobWorkStudio()->getWorkCellScene()->removeDrawable("ContactPointRender");
	getRobWorkStudio()->getWorkCellScene()->removeDrawable("ContactNormalRender");
	_maleFTrender = NULL;
	_femaleFTrender = NULL;
	_contactPointRender = NULL;
	_contactNormalRender = NULL;
	if (_currentTask->maleFTSensor != "") {
		// For now just use the base frame of the peg
		Frame* ftFrame = _wc->findFrame(_currentTask->maleID);
		if (ftFrame != NULL) {
			_maleFTrender = ownedPtr(new RenderForceTorque());
			_maleFTrender->setScales(0.01f,1);
			getRobWorkStudio()->getWorkCellScene()->addRender("MaleFTRender", _maleFTrender, ftFrame);
		}
	}
	if (_currentTask->femaleFTSensor != "") {
		// For now just use the base frame of the peg
		Frame* ftFrame = _wc->findFrame(_currentTask->femaleID);
		if (ftFrame != NULL) {
			_femaleFTrender = ownedPtr(new RenderForceTorque());
			_femaleFTrender->setScales(0.01f,1);
			getRobWorkStudio()->getWorkCellScene()->addRender("FemaleFTRender", _femaleFTrender, ftFrame);
		}
	}
	if (_currentTask->bodyContactSensors.size() > 0) {
		_contactPointRender = ownedPtr(new RenderPointCloud());
		_contactPointRender->setPointSize(5);
		getRobWorkStudio()->getWorkCellScene()->addRender("ContactPointRender", _contactPointRender, _wc->getWorldFrame());
		_contactNormalRender = ownedPtr(new RenderLines());
		getRobWorkStudio()->getWorkCellScene()->addRender("ContactNormalRender", _contactNormalRender, _wc->getWorldFrame());
	}

    PropertyMap::Ptr map = ownedPtr(new PropertyMap());
    *map = _currentTask->toCartesianTask()->getPropertyMap();
    _editor->setPropertyMap(map);
    std::string taskDesc = "Assemble <b>" + _currentTask->maleID + "</b> in <b>" + _currentTask->femaleID + "</b>.";
    _taskDescription->setText(QString::fromStdString(taskDesc));
}

void ATaskVisPlugin::selectResult(std::size_t i) {
    _currentResult = _results[i-1];

    // Save previous value (or prefer real if none set)
    bool real = _real->isChecked();
    if (!_real->isEnabled() && !_assumed->isEnabled())
    	real = true;

    // Disable both and enable the ones possible
	_real->setEnabled(false);
	_assumed->setEnabled(false);
    if (_currentResult->realState.size() > 0)
    	_real->setEnabled(true);
    if (_currentResult->assumedState.size() > 0)
    	_assumed->setEnabled(true);

    if (!_real->isEnabled() && _assumed->isEnabled())
    	_assumed->setChecked(true);
    if (_real->isEnabled() && !_assumed->isEnabled())
    	_real->setChecked(true);
    if (_real->isEnabled() && _assumed->isEnabled()) {
    	if (real)
    		_real->setChecked(true);
    	else
    		_assumed->setChecked(true);
    }

    constructPlayback();
}

void ATaskVisPlugin::constructPlayback() {
	WorkCell* wc = _wc;
	if (wc != NULL) {
		const State& defState = wc->getDefaultState();
		MovableFrame* male = wc->findFrame<MovableFrame>(_currentTask->maleID);
		MovableFrame* female = wc->findFrame<MovableFrame>(_currentTask->femaleID);
		std::vector<MovableFrame*> maleFlexFrames;
		std::vector<MovableFrame*> femaleFlexFrames;
		BOOST_FOREACH(const std::string &name, _currentTask->maleFlexFrames) {
			maleFlexFrames.push_back(_wc->findFrame<MovableFrame>(name));
		}
		BOOST_FOREACH(const std::string &name, _currentTask->femaleFlexFrames) {
			femaleFlexFrames.push_back(_wc->findFrame<MovableFrame>(name));
		}
		if (male == NULL)
			RW_THROW("Male frame could not be found (the frame must be Movable)!");
		if (female == NULL)
			RW_THROW("Female frame could not be found (the frame must be Movable)!");
		Frame* maleTCP = male;
		Frame* femaleTCP = female;
		if (_currentTask->maleTCP != "")
			maleTCP = wc->findFrame(_currentTask->maleTCP);
		if (_currentTask->femaleTCP != "")
			femaleTCP = wc->findFrame(_currentTask->femaleTCP);
		if (maleTCP == NULL)
			RW_THROW("MaleTCP frame could not be found!");
		if (femaleTCP == NULL)
			RW_THROW("FemaleTCP frame could not be found!");
		if (male != NULL && female != NULL) {
			TimedStatePath rwpath;
			Path<Timed<AssemblyState> > apath;
			if (_showReal)
				apath = _currentResult->realState;
			else
				apath = _currentResult->assumedState;
			BOOST_FOREACH(Timed<AssemblyState> &tastate, apath) {
				AssemblyState &astate = tastate.getValue();
				double time = tastate.getTime();
				State state = defState;
				if (!(astate.femaleOffset == Transform3D<>::identity())) {
					const Transform3D<> femaleTtcp = Kinematics::frameTframe(female,femaleTCP,defState);
					female->setTransform(female->getTransform(defState)*femaleTtcp*astate.femaleOffset*inverse(femaleTtcp),state);
				}
				const Transform3D<> maleTtcp = Kinematics::frameTframe(male,maleTCP,defState);
				if (!(astate.maleOffset == Transform3D<>::identity())) {
					male->setTransform(male->getTransform(defState)*maleTtcp*astate.maleOffset*inverse(maleTtcp),state);
				}
				// For now the male objects is moved relative to the female object - could be opposite
				const Transform3D<> wTfemaleTCP = Kinematics::worldTframe(femaleTCP,state);
				const Transform3D<> wTmaleTCPNew = wTfemaleTCP*astate.femaleTmale*inverse(maleTtcp)*astate.maleOffset*maleTtcp;
				if (maleFlexFrames.size() > 0 && astate.maleflexT.size() > 0) {
					RW_ASSERT(maleFlexFrames.size() == astate.maleflexT.size());
					// Find the world transform of the first flexible frame
					Transform3D<> wTflex = wTmaleTCPNew;
					BOOST_REVERSE_FOREACH(const Transform3D<> &flexT, astate.maleflexT) {
						wTflex = wTflex*inverse(flexT);
					}
					// Find the transform to the first flexible frame from the parent frame
					MovableFrame* cur = maleFlexFrames.front();
					Transform3D<> parentTflex = inverse(Kinematics::worldTframe(cur->getParent(),state))*wTflex;
					cur->setTransform(parentTflex,state);
					for (std::size_t i = 1; i < maleFlexFrames.size(); i++) {
						const Transform3D<> flexT = astate.maleflexT[i-1];
						const MovableFrame* prev = maleFlexFrames[i-1];
						cur = maleFlexFrames[i];
						const Frame* curParent = cur->getParent();
						const Transform3D<> prevTparent = Kinematics::frameTframe(prev,curParent,state);
						parentTflex = inverse(prevTparent)*flexT;
						cur->setTransform(parentTflex,state);
					}
				}
				// Now set the transform of the male object relative to its parent
				const Transform3D<> wTmaleParent = Kinematics::worldTframe(male->getParent(),state);
				male->setTransform(inverse(wTmaleParent)*wTmaleTCPNew*inverse(maleTtcp),state);
				// Add state to path
				const TimedState tstate(time,state);
				rwpath.push_back(tstate);
			}
			getRobWorkStudio()->setTimedStatePath(rwpath);
		}
	}
}

PropertyMap& ATaskVisPlugin::settings() {
	PropertyMap& rwsMap = getRobWorkStudio()->getPropertyMap();
	if (!rwsMap.has("ATaskVisPlugin")) {
		PropertyMap::Ptr map = ownedPtr(new PropertyMap());
		rwsMap.set<PropertyMap>("ATaskVisPlugin",*map);
	}
	return rwsMap.get<PropertyMap>("ATaskVisPlugin");
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(ATaskVisPlugin)
#endif
