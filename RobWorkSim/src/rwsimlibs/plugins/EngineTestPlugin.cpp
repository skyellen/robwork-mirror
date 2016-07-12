/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "EngineTestPlugin.hpp"

#include <rw/common/LogFileWriter.hpp>
#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadTask.hpp>

#include <RobWorkStudioConfig.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/simulator/PhysicsEngine.hpp>
#include <rwsimlibs/gui/log/SimulatorLogWidget.hpp>
#include <rwsimlibs/gui/log/MathematicaPlotWidget.hpp>

#include "ui_EngineTestPlugin.h"

#include <QListWidgetItem>

using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsim::simulator;
using namespace rwsimlibs::gui;
using namespace rwsimlibs::plugins;
using namespace rwsimlibs::test;

namespace {
class SimulationTimeEvent: public QEvent {
public:
	double time;
	bool failure;
	bool done;

public:
	static QEvent::Type Type;
	SimulationTimeEvent(double time, bool failure, bool done): QEvent(Type), time(time), failure(failure), done(done) {}
	virtual ~SimulationTimeEvent() {}
};
QEvent::Type SimulationTimeEvent::Type = static_cast<QEvent::Type>(QEvent::registerEventType());

static const QString STYLE_PROGRESS_COMMON = "QProgressBar {border: 2px solid grey; border-radius: 5px; text-align: center;} ";
static const QString STYLE_PROGRESS_SUCCESS = STYLE_PROGRESS_COMMON + "QProgressBar::chunk {background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0,stop: 0 #78d,stop: 1 #238 );}";

static QString styleProgressFail(double fraction) {
	return STYLE_PROGRESS_COMMON + "QProgressBar::chunk {background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0,stop: 0 #78d,stop: "+QString::number(fraction*0.9999)+" #238,stop: "+QString::number(fraction*1.0001)+" #FF0000,stop: 1 #FF7777 );}";
}

}

EngineTestPlugin::EngineTestPlugin():
	RobWorkStudioPlugin("EngineTestPlugin", QIcon(":/rwsimplugin/SimulationIcon.png")),
	_ui(new Ui::EngineTestPlugin()),
	_inputEditor(new PropertyViewEditor(this)),
	_logWidget(NULL),
	_simFailed(false)
{
	_ui->setupUi(this);

    _inputEditor->setHeaderVisible(false);
    _ui->parameterBox->layout()->addWidget(_inputEditor);

    connect(_ui->toolBox, SIGNAL(currentChanged(int)), this, SLOT(toolBoxChanged(int)) );
    //connect(_ui->engineList, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(engineChanged(QListWidgetItem*, QListWidgetItem*)) );
    //connect(_ui->testList, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), this, SLOT(testChanged(QListWidgetItem*, QListWidgetItem*)) );
    connect(_ui->engineList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(engineChanged(QListWidgetItem*)) );
    connect(_ui->testList, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(testChanged(QListWidgetItem*)) );

    connect(_ui->run, SIGNAL(clicked()), this, SLOT(run()) );
    connect(_ui->runVerbose, SIGNAL(clicked()), this, SLOT(verbose()) );
    connect(_ui->verboseCheck, SIGNAL(stateChanged(int)), this, SLOT(logCheck(int)) );
    connect(_ui->logCheck, SIGNAL(stateChanged(int)), this, SLOT(logCheck(int)) );
    connect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));
    connect(_ui->predefined, SIGNAL(currentIndexChanged(int)), this, SLOT(predefinedChoice(int)));

    _ui->toolBox->setCurrentWidget(_ui->selectionBox);
    toolBoxChanged(_ui->toolBox->currentIndex());

    QStringList header;
    header.push_back("Name");
    header.push_back("View");
    _ui->results->setHorizontalHeaderLabels(header);
#if RWS_USE_QT5
	_ui->results->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else
	_ui->results->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
#endif
	_ui->results->horizontalHeader()->setStretchLastSection(true);

    // Test
    //_ui->results->setRowCount(1);
    //_ui->results->item(0,0)->setText("Velocity");
    //_ui->results->setCellWidget(0,1,new QProgressBar());
    //_ui->results->item(0,2)->setText("Description of results...");
}

EngineTestPlugin::~EngineTestPlugin() {
	delete _ui;
}

void EngineTestPlugin::initialize() {
    getRobWorkStudio()->genericAnyEvent().add(boost::bind(&EngineTestPlugin::genericAnyEventListener, this, _1, _2), this);
    RobWorkStudioPlugin::initialize();
}

void EngineTestPlugin::toolBoxChanged(int index) {
	_ui->selectionBox->setEnabled(false);
	_ui->parameterBox->setEnabled(false);
	_ui->executionBox->setEnabled(false);
	if (_ui->toolBox->widget(index) == _ui->selectionBox) {
		message("Constructing lists of tests and engines...");
		_ui->engineList->clear();
		_ui->testList->clear();
		_test = NULL;
		_engine = "";
		_input = NULL;
		_dwc = NULL;
		const std::vector<std::string> engines = PhysicsEngine::Factory::getEngineIDs();
		BOOST_FOREACH(const std::string& engine, engines) {
			QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(engine));
			item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
			_ui->engineList->addItem(item);
		}
		const std::vector<std::string> tests = EngineTest::Factory::getTests();
		BOOST_FOREACH(const std::string& test, tests) {
			QListWidgetItem* item = new QListWidgetItem(QString::fromStdString(test));
			item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
			_ui->testList->addItem(item);
		}
		message("Please select which engine to use and which test to perform.");
		_ui->selectionBox->setEnabled(true);
	} else if (_ui->toolBox->widget(index) == _ui->parameterBox) {
		if (_ui->testList->currentItem() == NULL) {
			error("No test has been selected.");
			return;
		}

		message("Creating test...");
		const std::string testName = _ui->testList->currentItem()->text().toStdString();
		_test = EngineTest::Factory::getTest(testName);
		if (_test == NULL) {
			error("Could not create the test.");
			return;
		}

		message("Fetching parameters of test...");
		if (_input.isNull()) {
			const std::vector<PropertyMap::Ptr> predefined = _test->getPredefinedParameters();
			_ui->predefined->clear();
			_ui->predefined->addItem("Default",0);
			for (int i = 0; i < static_cast<int>(predefined.size()); i++) {
				_ui->predefined->addItem(QString::number(i+1),i+1);
			}
			_input = _test->getDefaultParameters();
		}
	    disconnect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));
		_inputEditor->setPropertyMap(_input);
	    connect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));

		updateInput(); // load the workcell

		message("Adjust the default parameters if required.");

		_ui->parameterBox->setEnabled(true);
	} else if (_ui->toolBox->widget(index) == _ui->executionBox) {
    	_ui->progress->setStyleSheet(STYLE_PROGRESS_SUCCESS);
    	_ui->progress->setValue(0);
    	_simFailed = false;

		if (_ui->testList->currentItem() == NULL) {
			error("No test has been selected.");
			return;
		}
		if (_ui->engineList->currentItem() == NULL) {
			error("No engine has been selected.");
			return;
		}

		if (_test.isNull()) {
			message("Creating test...");
			const std::string testName = _ui->testList->currentItem()->text().toStdString();
			_test = EngineTest::Factory::getTest(testName);
			if (_test == NULL) {
				error("Could not create the test.");
				return;
			}
		}

		if (_input.isNull()) {
			_input = _test->getDefaultParameters();
			updateInput(); // load the workcell
		}

		_engine = _ui->engineList->currentItem()->text().toStdString();
		if (!PhysicsEngine::Factory::hasEngineID(_engine)) {
			error("Could not find the engine in factory.");
			return;
		}
		message("Test and dynamic workcell loaded.");
		_ui->executionBox->setEnabled(true);
	}
}

void EngineTestPlugin::engineChanged(QListWidgetItem* current) {
	if (current == NULL)
		return;
	if (!(current->flags() && Qt::ItemIsEnabled)) {
		_ui->testList->setCurrentItem(NULL);
		_ui->testList->clearSelection();
		current->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
		current->setSelected(true);
	}
	const std::string engine = current->text().toStdString();
	for (int i = 0; i < _ui->engineList->count(); i++) {
		QListWidgetItem* const item = _ui->engineList->item(i);
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
	}
	for (int i = 0; i < _ui->testList->count(); i++) {
		QListWidgetItem* const item = _ui->testList->item(i);
		const std::string testName = item->text().toStdString();
		const EngineTest::Ptr test = EngineTest::Factory::getTest(testName);
		bool disable = false;
		if (test == NULL)
			disable = true;
		else if (!test->isEngineSupported(engine))
			disable = true;
		if (disable) {
			if (item == _ui->testList->currentItem()) {
				_ui->testList->setCurrentItem(NULL);
				_ui->testList->clearSelection();
			}
			item->setFlags(Qt::NoItemFlags);
		} else {
			item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
		}
	}
	if (_ui->testList->currentItem() == NULL)
		message("Engine \"" + engine + "\" selected, please select which test to perform.");
	else
		message("Proceed to Execution.");
}

void EngineTestPlugin::testChanged(QListWidgetItem* current) {
	if (current == NULL)
		return;
	if (!(current->flags() && Qt::ItemIsEnabled)) {
		_ui->engineList->setCurrentItem(NULL);
		_ui->engineList->clearSelection();
		current->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
		current->setSelected(true);
	}
	const std::string testName = current->text().toStdString();
	const EngineTest::Ptr test = EngineTest::Factory::getTest(testName);
	for (int i = 0; i < _ui->testList->count(); i++) {
		QListWidgetItem* const item = _ui->testList->item(i);
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
	}
	for (int i = 0; i < _ui->engineList->count(); i++) {
		QListWidgetItem* const item = _ui->engineList->item(i);
		const std::string engine = item->text().toStdString();
		if (!test->isEngineSupported(engine)) {
			if (item == _ui->engineList->currentItem()) {
				_ui->engineList->setCurrentItem(NULL);
				_ui->engineList->clearSelection();
			}
			item->setFlags(Qt::NoItemFlags);
		} else {
			item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
		}
	}
	if (_ui->engineList->currentItem() == NULL)
		message("Test \"" + testName + "\" selected, please select which engine to use.");
	else
		message("Proceed to Execution.");
}

void EngineTestPlugin::run() {
	if (_ui->run->text() == "Run") {
		_ui->run->setEnabled(false);
		_ui->verboseCheck->setEnabled(false);
		_ui->runVerbose->setEnabled(false);
		_ui->logCheck->setEnabled(false);
		_ui->logFile->setEnabled(false);
    	_ui->progress->setValue(0);
		_simFailed = false;
		_log = NULL;
		if (_ui->verboseCheck->isChecked()) {
			_log = ownedPtr(new SimulatorLogScope());
		}
		message("Running simulation...");
		_threadPool = ownedPtr(new ThreadPool());
		_runTask = ownedPtr(new ThreadTask(_threadPool));
		_testHandle = _test->runThread(_engine,*_input,_log,_runTask);
		RW_ASSERT(!_testHandle.isNull());
		_testHandle->setTimeCallback(boost::function<void(double,bool,bool)>(boost::bind(&EngineTestPlugin::simulatorCallBack, this, _1, _2, _3)));
		_runTask->execute();
		_ui->run->setText("Abort");
		_ui->run->setEnabled(true);
	} else if (_ui->run->text() == "Abort") {
		_ui->run->setEnabled(false);
		_testHandle->abort();
		_runTask->waitUntilDone();
		_runTask = NULL;
		_threadPool = NULL;
		bool verbose = false;
		if (_log != NULL) {
			if (_log->children() > 0)
				verbose = true;
		}
		_ui->verboseCheck->setEnabled(true);
		_ui->verboseCheck->setChecked(verbose);
		if (verbose) {
			_ui->runVerbose->setEnabled(true);
			_ui->logCheck->setEnabled(true);
			if (_ui->logCheck->isChecked())
				_ui->logFile->setEnabled(true);
		}
		_ui->run->setText("Run");
		_ui->run->setEnabled(true);
	}
}

void EngineTestPlugin::verbose() {
	if (_logWidget == NULL)
		_logWidget = new SimulatorLogWidget(NULL);
	_logWidget->setDWC(_dwc);
	_logWidget->setLog(_log);
	_logWidget->show();
	_logWidget->raise();
	_logWidget->activateWindow();
}

void EngineTestPlugin::inputChanged() {
	// Called if user modifies the standard parameters from the test
	if (_ui->predefined->itemText(0) != "(User)") {
	    disconnect(_ui->predefined, SIGNAL(currentIndexChanged(int)), this, SLOT(predefinedChoice(int)));
		_ui->predefined->insertItem(0,"(User)",-1);
	    connect(_ui->predefined, SIGNAL(currentIndexChanged(int)), this, SLOT(predefinedChoice(int)));
	}
	_ui->predefined->setCurrentIndex(0);

	updateInput();
}

void EngineTestPlugin::updateInput() {
	RW_ASSERT(!_test.isNull());
	message("Updating workcell with new parameters...");

	_dwc = _test->getDWC(*_input);
	if (_dwc.isNull()) {
		error("Could not retrieve a valid dynamic workcell from test.");
		return;
	}

    getRobWorkStudio()->getPropertyMap().add<DynamicWorkCell::Ptr>(
            "DynamicWorkcell",
            "A workcell with dynamic description",
            _dwc );
    getRobWorkStudio()->genericEvent().fire("DynamicWorkCellLoaded");
    getRobWorkStudio()->setWorkcell( _dwc->getWorkcell() );

	message("Workcell updated with new parameters.");
}

void EngineTestPlugin::predefinedChoice(int choice) {
	if (choice < 0) // Combobox was cleared
		return;
	const QString text = _ui->predefined->itemText(choice);
	if (text == "(User)") {
		return;
	} else {
		if (_ui->predefined->itemText(0) == "(User)")
			_ui->predefined->removeItem(0);
		if (text == "Default") { // Choose Default parameter set
			_input = _test->getDefaultParameters();
		} else {
			const unsigned int id = _ui->predefined->itemData(choice).toUInt();
			RW_ASSERT(id >= 1);
			const std::vector<PropertyMap::Ptr> pars = _test->getPredefinedParameters();
			RW_ASSERT(id <= pars.size());
			_input = pars[id-1];
		}
	    disconnect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));
		_inputEditor->setPropertyMap(_input);
	    connect(_inputEditor, SIGNAL(propertyChanged(const std::string&)), this, SLOT(inputChanged()));
		updateInput(); // load the workcell
	}
}

void EngineTestPlugin::message(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("");
}

void EngineTestPlugin::error(const std::string& msg) {
	_ui->status->setText(QString::fromStdString(msg));
	_ui->status->setStyleSheet("QLabel { color : red; }");
}

void EngineTestPlugin::simulatorCallBack(double time, bool failure, bool done) {
	QCoreApplication::postEvent(this,new SimulationTimeEvent(time,failure,done));
}

bool EngineTestPlugin::event(QEvent *event) {
    if (event->type() == SimulationTimeEvent::Type) {
		const SimulationTimeEvent* const simEvent = static_cast<const SimulationTimeEvent*>(event);
    	const double fraction = simEvent->time / _test->getRunTime();
    	const double pct = fraction * 100;
    	_ui->progress->setValue(static_cast<int>(pct));
    	if (!_simFailed) {
    		if (simEvent->failure) {
    			_simFailed = true;
    			_ui->progress->setStyleSheet(styleProgressFail(fraction));
    			error("Simulation failed: " + _testHandle->getError());
    		} else {
    			_ui->progress->setStyleSheet(STYLE_PROGRESS_SUCCESS);
    			if (simEvent->done) {
    				message("Simulation done.");
    			}
    		}
    	}
    	if (simEvent->done) {
            getRobWorkStudio()->setTimedStatePath(_testHandle->getTimedStatePath());
    		_ui->run->setText("Run");
    		_ui->run->setEnabled(true);
    		bool verbose = false;
    		if (_log != NULL) {
    			if (_log->children() > 0)
    				verbose = true;
    		}
    		if (verbose)
    			_ui->runVerbose->setEnabled(true);

    		const std::vector<EngineTest::Result>& results = _testHandle->getResults();
    	    _ui->results->setRowCount(results.size());
    	    for (std::size_t i = 0; i < results.size(); i++) {
    			_ui->results->setItem(i,0,new QTableWidgetItem(QString::fromStdString(results[i].name)));
    			_ui->results->item(i,0)->setData(Qt::ToolTipRole,QString::fromStdString(results[i].description));
    	    	QPushButton* const button = new QPushButton("Show");
    	        connect(button, SIGNAL(pressed()), this, SLOT(resultShow()) );
    	    	_ui->results->setCellWidget(i,1,button);
    	    }
    	}
    	return true;
    }

    return QDockWidget::event(event);
}

void EngineTestPlugin::resultShow() {
	for (int i = 0; i < _ui->results->rowCount(); i++) {
		const QWidget* const widget = _ui->results->cellWidget(i,1);
		const QPushButton* const button = dynamic_cast<const QPushButton*>(widget);
		RW_ASSERT(button);
		if (button->isDown()) {
			const EngineTest::Result& result = _testHandle->getResults()[i];
			MathematicaPlotWidget* const widget = new MathematicaPlotWidget(this);
			widget->setWindowFlags(Qt::Dialog);
			widget->resize(550,350);
			widget->show();
			std::vector<double> x;
			std::vector<double> y;
			BOOST_FOREACH(const TimedQ& tq, result.values) {
				const Q& q = tq.getValue();
				for (std::size_t k = 0; k < q.size(); k++) {
					x.push_back(tq.getTime());
					y.push_back(q[k]);
				}
			}
			widget->listPlot(x,y);
			break;
		}
	}
}

void EngineTestPlugin::logCheck(int state) {
	const QObject* const sender = QObject::sender();
	if (sender == _ui->verboseCheck) {
		if (state == Qt::Checked) {
			_ui->logCheck->setEnabled(true);
			if (_ui->logCheck->isChecked())
				_ui->logFile->setEnabled(true);
		} else {
			_ui->runVerbose->setEnabled(false);
			_ui->logCheck->setEnabled(false);
			_ui->logFile->setEnabled(false);
		}
	} else if (sender == _ui->logCheck) {
		if (state == Qt::Checked) {
			_ui->logFile->setEnabled(true);
		} else {
			_ui->logFile->setEnabled(false);
		}
	}
}

void EngineTestPlugin::genericAnyEventListener(const std::string& event, boost::any data) {
	if (_logWidget == NULL || getRobWorkStudio()->getTimedStatePath().size() == 0)
		return;
    try{
    	if (event == "PlayBack::TimeRelative") {
    		const double timeRelative = boost::any_cast<double>(data);
    		const double endTime = getRobWorkStudio()->getTimedStatePath().back().getTime();
    		const double time = timeRelative*endTime;
    		_logWidget->setSelectedTime(time);
    	}
    } catch (...){
        Log::warningLog() << "EngineTestPlugin: Event \"" << event << "\" did not have the correct datatype or an error occured!\n";
    }
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(EngineTestPlugin);
#endif
