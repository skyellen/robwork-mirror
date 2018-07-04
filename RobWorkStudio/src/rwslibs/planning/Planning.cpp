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

#include "Planning.hpp"

#include <RobWorkStudio.hpp>

#include <rw/common/Exception.hpp>
#include <rw/common/Message.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/models/CompositeDevice.hpp>
#include <rw/models/Models.hpp>
#include <rw/pathplanning/PathAnalyzer.hpp>
#include <rw/proximity/DistanceCalculator.hpp>
#include <rw/trajectory/TimedUtil.hpp>

#include <rwlibs/pathoptimization/clearance/ClearanceOptimizer.hpp>
#include <rwlibs/pathoptimization/clearance/MinimumClearanceCalculator.hpp>
#include <rwlibs/pathoptimization/pathlength/PathLengthOptimizer.hpp>
#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

//#include <rws/components/propertyview/PropertyInspector.hpp>

#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QMessageBox>
#include <QComboBox>
#include <QCheckBox>
#include <QFileDialog>

#include <vector>

using namespace rw::common;
using rw::kinematics::State;
using rw::loaders::PathLoader;
using namespace rw::models;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using rw::trajectory::TimedUtil;

using namespace rwlibs::pathoptimization;
using namespace rwlibs::pathplanners;
using rwlibs::proximitystrategies::ProximityStrategyFactory;

using namespace rws;

namespace {
    const QString CLEARANCE = "Clearance";
    const QString PATHPRUNING = "Length (Path Pruning)";
    const QString SHORTCUT = "Length (ShortCut)";
    const QString PARTIALSHORTCUT = "Length (Partial ShortCut)";

    const QString RRT = "RRT";
    const QString SBL = "SBL";
    const QString PRM = "PRM";

    QMetric::CPtr getMetric()
    {
        return ownedPtr(new EuclideanMetric<Q>());
    }

	QToQPlanner::Ptr getPlanner(
        WorkCell* workcell,
		Device::Ptr device,
        const State& state,
        CollisionDetector* collisionDetector,
        const QString& type)
    {
        const PlannerConstraint constraint = PlannerConstraint::make(
            collisionDetector, device, state);

        if (type == RRT) {
            return RRTPlanner::makeQToQPlanner(constraint, device);
        } else if (type == SBL) {
			QConstraint::Ptr qconstraint = QConstraint::make(collisionDetector, device, state);
			return SBLPlanner::makeQToQPlanner(SBLSetup::make(qconstraint, QEdgeConstraintIncremental::makeDefault(qconstraint, device), device));
        } else if (type == PRM) {
            Ptr<PRMPlanner> prm = ownedPtr(
                new PRMPlanner(
                    device.get(), state, collisionDetector, 0.01));

            prm->setCollisionCheckingStrategy(PRMPlanner::NODECHECK);
            prm->setShortestPathSearchStrategy(PRMPlanner::DIJKSTRA);
            prm->setNeighSearchStrategy(PRMPlanner::PARTIAL_INDEX_TABLE);
            prm->setAStarTimeOutTime(1);
            prm->buildRoadmap(2000);
            return prm;
        }
        else {
            RW_THROW("Unknown planner " << StringUtil::quote(type.toStdString()));
        }
    }

}


Planning::Planning():
    RobWorkStudioPlugin("Planning", QIcon(":/planning.png"))
{
    int row = 0;

    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    _planAllDev = new QCheckBox("Plan for all devices");
    pLayout->addWidget(_planAllDev, row++, 0);
    connect(_planAllDev, SIGNAL(stateChanged(int)), this, SLOT(setPlanAll(int)));

    pLayout->addWidget(new QLabel("Device: "), row, 0);

    _cmbDevices = new QComboBox();
    pLayout->addWidget(_cmbDevices, row++, 1);
    connect(_cmbDevices, SIGNAL(activated(int)), this, SLOT(deviceChanged(int)));

    QPushButton* btnStart = new QPushButton("Set Start");
    connect(btnStart, SIGNAL(clicked()), this, SLOT(setStart()));
    pLayout->addWidget(btnStart, row, 0);

    QPushButton* btnGotoStart = new QPushButton("Goto Start");
    connect(btnGotoStart, SIGNAL(clicked()), this, SLOT(gotoStart()));
    pLayout->addWidget(btnGotoStart, row++, 1);
    /*
    _lblStart = new QLabel();
    pLayout->addWidget(_lblStart, row++, 1);
*/
    QPushButton* btnGoal = new QPushButton("Set Goal");
    connect(btnGoal, SIGNAL(clicked()), this, SLOT(setGoal()));
    pLayout->addWidget(btnGoal, row, 0);

    QPushButton* btnGotoGoal = new QPushButton("Goto Goal");
    connect(btnGotoGoal, SIGNAL(clicked()), this, SLOT(gotoGoal()));
    pLayout->addWidget(btnGotoGoal, row++, 1);

    /*
    _lblGoal = new QLabel();
    pLayout->addWidget(_lblGoal, row++, 1);
*/
    pLayout->addWidget(new QLabel("Planner"), row, 0);
    _cmbPlanners = new QComboBox();
    _cmbPlanners->addItem(RRT);
    _cmbPlanners->addItem(SBL);
    _cmbPlanners->addItem(PRM);
    _cmbPlanners->setCurrentIndex(2);
    pLayout->addWidget(_cmbPlanners, row++, 1);

    pLayout->addWidget(new QLabel("Collision Detector"), row, 0);
    _cmbCollisionDetectors = new QComboBox();
    std::vector<std::string> ids = ProximityStrategyFactory::getCollisionStrategyIDs();
    BOOST_FOREACH(std::string id, ids){
        _cmbCollisionDetectors->addItem(id.c_str());
    }

    pLayout->addWidget(_cmbCollisionDetectors, row++, 1);


    QPushButton* btnPlan = new QPushButton("Plan");
    connect(btnPlan, SIGNAL(clicked()), this, SLOT(plan()));
    pLayout->addWidget(btnPlan, row++, 0);


    pLayout->addWidget(new QLabel("Optimization"), row++, 0);
    pLayout->addWidget(new QLabel("Strategy"), row, 0);
    _cmbOptimization = new QComboBox();


    _cmbOptimization->addItem(CLEARANCE);
    _cmbOptimization->addItem(PATHPRUNING);
    _cmbOptimization->addItem(SHORTCUT);
    _cmbOptimization->addItem(PARTIALSHORTCUT);
    pLayout->addWidget(_cmbOptimization, row++, 1);

    QPushButton* btnOptimize = new QPushButton("Optimize");
    connect(btnOptimize, SIGNAL(clicked()), this, SLOT(optimize()));
    pLayout->addWidget(btnOptimize, row++, 0);

    QPushButton* btnSavePath = new QPushButton("Save");
    connect(btnSavePath, SIGNAL(clicked()), this, SLOT(savePath()));
    pLayout->addWidget(btnSavePath, row, 0);

    QPushButton* btnLoadPath = new QPushButton("Load");
    connect(btnLoadPath, SIGNAL(clicked()), this, SLOT(loadPath()));
    pLayout->addWidget(btnLoadPath, row++, 1);


    QPushButton* btnStatistics = new QPushButton("Statistics");
    connect(btnStatistics, SIGNAL(clicked()), this, SLOT(performStatistics()));
    pLayout->addWidget(btnStatistics, row++, 0);


    pLayout->setRowStretch(row, 1);
}


Planning::~Planning() {

}


void Planning::open(WorkCell* workcell) {
    _workcell = workcell;
    const State _state = getRobWorkStudio()->getState();
    _starts.clear();
    _goals.clear();
	std::vector<Device::Ptr> devices = workcell->getDevices();
    _cmbDevices->clear();
	for (std::vector<Device::Ptr>::iterator it = devices.begin(); it != devices.end(); ++it) {
        _cmbDevices->addItem((*it)->getName().c_str());
        _starts.push_back( (*it)->getQ(_state) );
        _goals.push_back( (*it)->getQ(_state) );
    }
}

void Planning::close() {
    _cmbDevices->clear();
}



void Planning::setPlanAll(int state){

}


void Planning::setStart() {
	/*
    Device::Ptr device = _workcell->getDevices().front();
	Jacobian jac = device->baseJend(getRobWorkStudio()->getState());
	std::cout<<"Jacobian = "<<jac<<std::endl;
	std::cout<<"Determinant = "<<LinearAlgebra::det(jac.m())<<std::endl;
	return;
*/


    const State _state = getRobWorkStudio()->getState();
    size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
    if (deviceIndex>=_workcell->getDevices().size())
        RW_THROW("Index out of bounds");

    Device::Ptr device = _workcell->getDevices().at(deviceIndex);
    _starts[deviceIndex] = device->getQ(_state);

    std::stringstream str;
    str<<_starts[deviceIndex];
    //_lblStart->setText(str.str().c_str());
    //rw::common::Message msg( __FILE__, __LINE__, "Start set to: " + str.str());
  //  emitMessage("Planning","Info", msg);

}


void Planning::gotoStart(){
    State _state = getRobWorkStudio()->getState();
    size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
    if (deviceIndex>=_workcell->getDevices().size())
        RW_THROW("Index out of bounds");

	Device::Ptr device = _workcell->getDevices().at(deviceIndex);
    device->setQ(_starts[deviceIndex],_state);
    getRobWorkStudio()->setState(_state);
    //getRobWorkStudio()->fireStateChanged();
}


void Planning::setGoal() {
    const State _state = getRobWorkStudio()->getState();
    size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
    if (deviceIndex>=_workcell->getDevices().size())
        RW_THROW("Index out of bounds");

	Device::Ptr device = _workcell->getDevices().at(deviceIndex);
    _goals[deviceIndex] = device->getQ(_state);

    std::stringstream str;
    str << _goals[deviceIndex];
    //_lblGoal->setText(str.str().c_str());
    rw::common::Message msg( __FILE__, __LINE__, "Goal set to: " + str.str());
 //   emitMessage("Planning","Info", msg);
}


void Planning::gotoGoal(){
    State _state = getRobWorkStudio()->getState();
    size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
    if (deviceIndex>=_workcell->getDevices().size())
        RW_THROW("Index out of bounds");

	Device::Ptr device = _workcell->getDevices().at(deviceIndex);
    device->setQ(_goals[deviceIndex],_state);
    getRobWorkStudio()->setState(_state);
}


void Planning::deviceChanged(int index){
    /*
    std::stringstream startstr,goalstr;
    startstr << _starts[index];
    _lblStart->setText(startstr.str().c_str());
    goalstr << _goals[index];
    _lblGoal->setText(goalstr.str().c_str());
    */
}


Device::Ptr Planning::getDevice() {
    const State _state = getRobWorkStudio()->getState();
    //Get the device model
    size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
    if (deviceIndex>=_workcell->getDevices().size())
        RW_THROW("Index out of bounds");

    bool planForAll = _planAllDev->isChecked();

    if(planForAll){
        // create a composite device that contain all other devices
		std::vector<Device::Ptr> all = _workcell->getDevices();
		std::vector<Device::Ptr> devices(all.begin(), all.end());
                _compositeDevice = ownedPtr(
            new rw::models::CompositeDevice(
                devices.front()->getBase(),
                devices,
                devices.back()->getEnd(),
                "Composite",
                _state));

        return _compositeDevice;
    } else {
        return _workcell->getDevices().at(deviceIndex);
    }
}

void Planning::plan()
{
    const State _state = getRobWorkStudio()->getState();

    bool planForAll = _planAllDev->isChecked();

    Q start,goal;
	Device::Ptr device = getDevice();
    if(planForAll) {
        // create start and goal config
        Q startTmp(device->getDOF());
        Q goalTmp(device->getDOF());
        size_t offset = 0;
        for(size_t j=0; j<_starts.size(); j++){
            size_t dof = _starts[j].size();
            for(size_t i=0; i<dof; i++){
                startTmp(offset+i) = _starts[j](i);
                goalTmp(offset+i) = _goals[j](i);
            }
            offset += dof;
        }
        start = startTmp;
        goal = goalTmp;
    } else {
        size_t deviceIndex = (size_t)_cmbDevices->currentIndex();
        start = _starts[deviceIndex];
        goal = _goals[deviceIndex];
    }

    const size_t dof = device->getDOF();
    if (dof != start.size() || dof != goal.size()) {
        QMessageBox::information(
            NULL,
            "Error",
            "Start and/or goal configuration does not match device",
            QMessageBox::Ok);
        return;
    }

    //Get the CDStrategy
	const rw::common::Ptr<CollisionStrategy> cdstrategy =
        ProximityStrategyFactory::makeCollisionStrategy(_cmbCollisionDetectors->currentText().toStdString());
        //getCollisionStrategy(_cmbCollisionDetectors->currentIndex());

    if (!cdstrategy) {
        QMessageBox::information(
            NULL,
            "Error",
            "The selected cd-strategy could not be found",
            QMessageBox::Ok);
        return;
    }

    CollisionDetector collisionDetector(_workcell, cdstrategy);

    //Construct the PathPlanner
	QToQPlanner::Ptr planner = getPlanner(
        _workcell, device, _state, &collisionDetector, _cmbPlanners->currentText());

    //Query
    _path.clear();
    if (planner->query(start, goal, _path, 60)) {
        log().info() << "Path Node Count = "<<_path.size()<<std::endl;
    } else {
        log().info() <<"Could not find a solution"<<std::endl;
    }

    setAsTimedStatePath();

}


void Planning::setAsTimedStatePath() {
    State state = getRobWorkStudio()->getState();
    std::vector<State> states;
	Device::Ptr device = getDevice();
    if (_path.size() > 0 && _path.front().size() != device->getDOF()) {
        QMessageBox::information(
            NULL,
            "Error",
            "Dimensions of device and loaded path does not match",
            QMessageBox::Ok);
        return;
    }

    getRobWorkStudio()->setTimedStatePath(
        TimedUtil::makeTimedStatePath(
            *_workcell,
            Models::getStatePath(*device, _path, state)));
}


void Planning::optimize() {
    const State _state = getRobWorkStudio()->getState();
    //TODO Need to make a selfcontaining PropertyInspector, such that QDialog is not needed
    /*PropertyInspector* props = new PropertyInspector();
    QDialog* dialog = new QDialog(this);
    QGridLayout* pLayout = new QGridLayout(dialog);
    dialog->setLayout(pLayout);
    pLayout->addWidget(props);*/

	Device::Ptr device = getDevice();

    if (_cmbOptimization->currentText() == CLEARANCE) {
        try {
			DistanceStrategy::Ptr strat = ProximityStrategyFactory::makeDefaultDistanceStrategy();

            DistanceCalculator::CPtr distanceCalculator = ownedPtr(
                new DistanceCalculator(
                    _workcell->getWorldFrame(),
					_workcell,
                        strat,
                    _state));

			ClearanceCalculator::CPtr clearanceCalculator = ownedPtr(new MinimumClearanceCalculator(distanceCalculator));
            ClearanceOptimizer optimizer(//_workcell,
                                         device,
                                         _state,
                                         getMetric(),
                                         clearanceCalculator);

         /*   props->setPropertyMap(&(optimizer.getPropertyMap()));
            dialog->exec();
            props->setPropertyMap(NULL);
			*/

            _path = optimizer.optimize(_path);

            log().info()<<"Optimization Finished"<<std::endl;
        } catch (const Exception& e) {
            log().info()<<"Exception while Optimizing: "<<e.getMessage().getText()<<std::endl;
        }

    } else {
        CollisionDetector collisionDetector(
            _workcell,
            ProximityStrategyFactory::makeDefaultCollisionStrategy()
            );

        PlannerConstraint constraint = PlannerConstraint::make(
            &collisionDetector, device, _state);

        PathLengthOptimizer optimizer(
            constraint, MetricFactory::makeEuclidean<Q>());

        if (_cmbOptimization->currentText() == PATHPRUNING) {
            _path = optimizer.pathPruning(_path);
            log().info()<<"Optimization Finished"<<std::endl;
        }

        else if (_cmbOptimization->currentText() == SHORTCUT) {
/*            props->setPropertyMap(&(optimizer.getPropertyMap()));
            dialog->exec();
            props->setPropertyMap(NULL);
*/
            _path = optimizer.shortCut(_path);
        }

        else if (_cmbOptimization->currentText() == PARTIALSHORTCUT) {
            /*props->setPropertyMap(&(optimizer.getPropertyMap()));
            dialog->exec();
            props->setPropertyMap(NULL);
*/
            _path = optimizer.partialShortCut(_path);
        }
    }
    if (_path.size() > 2) {
        setAsTimedStatePath();
    }
//    delete dialog;
}


void Planning::performStatistics() {
    const State _state = getRobWorkStudio()->getState();
	Device::Ptr device = getDevice();
    PathAnalyzer analyzer( device, _state);
    PathAnalyzer::JointSpaceAnalysis jointanalysis = analyzer.analyzeJointSpace(_path);
    log().info()<<"JointSpaceAnalysis.Length = "<<jointanalysis.length<<std::endl;
    log().info()<<"JointSpaceAnalysis.NodeCount = "<<jointanalysis.nodecount<<std::endl;

    PathAnalyzer::CartesianAnalysis cartanalysis = analyzer.analyzeCartesian(_path, device->getEnd());
    log().info()<<"CartesianAnalysis.length "<<cartanalysis.length<<std::endl;
    log().info()<<"CartesianAnalysis.lengths = "<<cartanalysis.distances<<std::endl;
    log().info()<<"CartesianAnalysis.lower = "<<cartanalysis.lower<<std::endl;
    log().info()<<"CartesianAnalysis.upper = "<<cartanalysis.upper<<std::endl;

    PathAnalyzer::TimeAnalysis timeanalysis = analyzer.analyzeTime(_path);
    log().info()<<"TimeAnalysis.time1 = "<<timeanalysis.time1<<std::endl;

	DistanceStrategy::Ptr strat = ProximityStrategyFactory::makeDefaultDistanceStrategy();

    DistanceCalculator distanceCalculator(
        _workcell->getWorldFrame(),
        _workcell,
        strat,
        _state);

    PathAnalyzer::ClearanceAnalysis clearanceanalysis = analyzer.analyzeClearance(_path, &distanceCalculator);
    log().info()<<"ClearanceAnalysis.average = "<<clearanceanalysis.average<<std::endl;
    log().info()<<"ClearanceAnalysis.min = "<<clearanceanalysis.min<<std::endl;

}


void Planning::savePath() {
    const QString dir(_previousOpenSaveDirectory.c_str());
    QString selectedFilter;
    QString filename = QFileDialog::getSaveFileName(this, "Save playback file", dir, "Path files ( *.pth )");

    if (!filename.isEmpty()) {
        _previousOpenSaveDirectory = StringUtil::getDirectoryName(filename.toStdString());
        PathLoader::storePath(_path, filename.toStdString());
    }
}


void Planning::loadPath() {
    QString filename = QFileDialog::getOpenFileName(this, "Open path file", // Title
                                                    "", // Directory
                                                    "Path files ( *.pth )\n All ( *.* )");

    if (!filename.isEmpty()) {
        _previousOpenSaveDirectory = StringUtil::getDirectoryName(filename.toStdString());
        _path = PathLoader::loadPath(filename.toStdString());
        setAsTimedStatePath();

    }
}


namespace {
    Q getRandQ(Device* device, CollisionDetector& collisionDetector, State& state) {
        Q q(device->getDOF());
        do {
            std::pair<Q, Q> bounds = device->getBounds();
            for (size_t i = 0; i<device->getDOF(); i++) {
                q(i) = Math::ran(bounds.first(i), bounds.second(i));
            }
            device->setQ(q, state);
        }
        while (collisionDetector.inCollision(state));
        return q;
    }
}


#ifndef RWS_USE_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(Planning)
#endif
#endif
