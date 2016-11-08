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

#include "SimulatorLogViewer.hpp"

#include <rw/common/BINArchive.hpp>
#include <rw/common/INIArchive.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsimlibs/gui/log/SimulatorLogWidget.hpp>

#include "ui_SimulatorLogViewer.h"

#include <QMessageBox>
#include <QFileDialog>

#include <fstream>

using namespace rw::common;
using rwsim::dynamics::DynamicWorkCell;
using rwsim::loaders::DynamicWorkCellLoader;
using rwsim::log::SimulatorLogScope;
using rwsimlibs::gui::SimulatorLogWidget;
using rwsimlibs::tools::SimulatorLogViewer;

SimulatorLogViewer::SimulatorLogViewer():
	QMainWindow(),
	_ui(new Ui::SimulatorLogViewer()),
	_dwc(NULL),
	_log(NULL),
	_widget(new SimulatorLogWidget(this))
{
	_ui->setupUi(this);
	setCentralWidget(_widget);
}

SimulatorLogViewer::~SimulatorLogViewer() {
}

void SimulatorLogViewer::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
	_dwc = dwc;
	if (_dwc == NULL) {
		_ui->_actionClose->setEnabled(false);
		_ui->_actionOpen->setEnabled(true);
	} else {
		_ui->_actionClose->setEnabled(true);
		_ui->_actionOpen->setEnabled(false);
	}
	_widget->setDWC(_dwc);
	emit dwcChanged(_dwc);
}

void SimulatorLogViewer::setLog(SimulatorLogScope::Ptr log) {
	_log = log;
	if (!_log.isNull())
		_ui->_actionCompare->setEnabled(true);
	else
		_ui->_actionCompare->setEnabled(false);
	_widget->setLog(_log);
}

void SimulatorLogViewer::openDWC() {
	QString selectedFilter;
	const QString filename = QFileDialog::getOpenFileName(
			this,
			"Open Dynamic Workcell", // Title
			QDir::currentPath(), // Directory
			"DWC XML files ( *.dwc.xml )"
			"\nAll supported ( *.xml )"
			"\n All ( *.* )",
			&selectedFilter);

	const std::string dwcFile = filename.toStdString();
	if (dwcFile.empty())
		return;

	rw::common::Ptr<DynamicWorkCell> dwc = NULL;
	try {
		dwc = DynamicWorkCellLoader::load(dwcFile);
	} catch (const Exception& exp) {
		QMessageBox::information(NULL, "Exception",	exp.getMessage().getText().c_str(), QMessageBox::Ok);
		return;
	}
	if( dwc==NULL )
		RW_THROW("Dynamic workcell is null");

	setDWC(dwc);
}

void SimulatorLogViewer::closeDWC() {
	setDWC(NULL);
}

void SimulatorLogViewer::openCompare() {
	QString selectedFilter;
	const QString filename = QFileDialog::getOpenFileName(
			this,
			"Open log for comparison", // Title
			QDir::currentPath(), // Directory
			"Binary log files ( *.bin )"
			"Ini log files ( *.ini )"
			"\nAll supported ( *.bin *.ini )"
			"\n All ( *.* )",
			&selectedFilter);

	const std::string file = filename.toStdString();
	if (file.empty())
		return;

	SimulatorLogScope::Ptr scope = NULL;
	if (StringUtil::toUpper(StringUtil::getFileExtension(file)) == "BIN") {
        std::ifstream fstr(file.c_str(), std::ios::in);
		BINArchive archive;
		archive.open(fstr);
		if (!archive.isOpen())
			QMessageBox::information(NULL, "Exception",	"Could not open the given file: " + QString::fromStdString(file), QMessageBox::Ok);
		else {
			scope = ownedPtr(new SimulatorLogScope());
			scope->read(archive,"");
		}
	} else {
        std::ifstream fstr(file.c_str(), std::ios::in);
		INIArchive archive;
		archive.open(fstr);
		if (!archive.isOpen())
			QMessageBox::information(NULL, "Exception",	"Could not open the given file: " + QString::fromStdString(file), QMessageBox::Ok);
		else {
			scope = ownedPtr(new SimulatorLogScope());
			scope->read(archive,"");
		}
	}
	if (scope.isNull())
		return;

	_widget->compare(scope);
}

// Main Program

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/filesystem.hpp>

#include <iomanip>

using namespace boost::program_options;

int main(int argc, char** argv) {
    QApplication app(argc,argv);

	options_description desc("Allowed options");
	desc.add_options()
	        		("help", "Produce this help message.")
	        		("dwc,d", value<std::string>(), "The dynamic workcell (optional).")
	        		("file,f", value<std::string>(), "The input file (optional).")
					("ini,i", "Input file is in ini format (default).")
	        		("bin,b", "Input file is in binary format (optional).")
	        		;

	// Let QApplication parse arguments first, and then parse remaining arguments
    QStringList qargs = app.arguments();
    std::vector<std::string> args(qargs.size());
    for (int i = 0; i < qargs.size(); i++) {
    	args[i] = qargs[i].toStdString();
    }

	variables_map vm;
    command_line_parser parser(args);
    parser.options(desc).style(0).extra_parser(ext_parser());
    store(parser.run(), vm);
	notify(vm);

	if (vm.count("help")) {
		std::cout << "Usage:\n\n"
				<< "\t" << argv[0] <<" [options]\n"
				<< "\n";
		std::cout << desc << "\n";
		return 1;
	}

	rw::common::Ptr<DynamicWorkCell> dwc = NULL;
	if (vm.count("dwc")) {
		try {
			dwc = DynamicWorkCellLoader::load(vm["dwc"].as<std::string>());
		} catch (const Exception& exp) {
			QMessageBox::information(NULL, "Exception",	exp.getMessage().getText().c_str(), QMessageBox::Ok);
		}
	}

    std::string file;
	SimulatorLogScope::Ptr scope = NULL;
    if(vm.count("file")) {
    	file = IOUtil::getAbsoluteFileName(vm["file"].as<std::string>());
    	const bool bin = vm.count("bin") > 0;
	    if( !boost::filesystem::exists(file) ) {
			QMessageBox::information(NULL, "No such file", "File does not exist: " + QString::fromStdString(file), QMessageBox::Ok);
	    }
    	try {
    		if (bin) {
    	        std::ifstream fstr(file.c_str(), std::ios::in);
    			BINArchive archive;
    			archive.open(fstr);
    			if (!archive.isOpen())
    				QMessageBox::information(NULL, "Exception",	"Could not open the given file: " + QString::fromStdString(file), QMessageBox::Ok);
    			else {
    				scope = ownedPtr(new SimulatorLogScope());
    				scope->read(archive,"");
    			}
    		} else {
    	        std::ifstream fstr(file.c_str(), std::ios::in);
    	        fstr.precision(17);
    			INIArchive archive;
    			archive.open(fstr);
    			if (!archive.isOpen())
    				QMessageBox::information(NULL, "Exception",	"Could not open the given file: " + QString::fromStdString(file), QMessageBox::Ok);
    			else {
    				scope = ownedPtr(new SimulatorLogScope());
    				scope->read(archive,"");
    			}
    		}
    	} catch(const Exception& e) {
    		std::stringstream msg;
    		msg << "Could not open the given file: " << e.what() << std::endl;
			QMessageBox::information(NULL, "Exception",	QString::fromStdString(msg.str()), QMessageBox::Ok);
    	}
    }

    SimulatorLogViewer viewer;
    if (dwc != NULL)
    	viewer.setDWC(dwc);
    if (scope != NULL)
    	viewer.setLog(scope);
    viewer.show();
    return app.exec();
}
