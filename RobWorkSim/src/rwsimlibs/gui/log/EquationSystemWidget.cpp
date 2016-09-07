/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "EquationSystemWidget.hpp"
#include "ui_EquationSystemWidget.h"

#include <RobWorkStudioConfig.hpp>

#include <rwsim/log/LogEquationSystem.hpp>

#include <QMenu>
#include <QClipboard>

using namespace rwsim::log;
using namespace rwsimlibs::gui;

EquationSystemWidget::EquationSystemWidget(rw::common::Ptr<const LogEquationSystem> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::EquationSystemWidget()),
	_system(entry)
{
	_ui->setupUi(this);

#if RWS_USE_QT5
	_ui->_A->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	_ui->_A->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	_ui->_b->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	_ui->_b->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	_ui->_x->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	_ui->_x->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#else
	_ui->_A->verticalHeader()->setResizeMode(QHeaderView::Stretch);
	_ui->_A->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	_ui->_b->verticalHeader()->setResizeMode(QHeaderView::Stretch);
	_ui->_b->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
	_ui->_x->verticalHeader()->setResizeMode(QHeaderView::Stretch);
	_ui->_x->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#endif

	_ui->_A->setContextMenuPolicy(Qt::CustomContextMenu);
	_ui->_b->setContextMenuPolicy(Qt::CustomContextMenu);
	_ui->_A->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(_ui->_A, SIGNAL(customContextMenuRequested(const QPoint&)),
	    this, SLOT(showContextMenu(const QPoint&)));
	connect(_ui->_b, SIGNAL(customContextMenuRequested(const QPoint&)),
	    this, SLOT(showContextMenu(const QPoint&)));
	connect(_ui->_x, SIGNAL(customContextMenuRequested(const QPoint&)),
	    this, SLOT(showContextMenu(const QPoint&)));
}

EquationSystemWidget::~EquationSystemWidget() {
}

void EquationSystemWidget::setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc) {
	// we do not use any DWC in this widget
}

void EquationSystemWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogEquationSystem> system = entry.cast<const LogEquationSystem>();
	if (!(system == NULL)) {
		_system = system;
	} else {
		RW_THROW("EquationSystemWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> EquationSystemWidget::getEntry() const {
	return _system;
}

void EquationSystemWidget::updateEntryWidget() {
	const Eigen::MatrixXd& A = _system->A();
	const Eigen::VectorXd& x = _system->x();
	const Eigen::VectorXd& b = _system->b();

	// Set dimension labels
	_ui->_Adim->setText(QString::number(A.rows())+" x "+QString::number(A.cols()));
	_ui->_bDim->setText(QString::number(b.rows())+" x 1");
	if (x.rows() == 0)
		_ui->_xDim->setText("x");
	else
		_ui->_xDim->setText(QString::number(x.rows())+" x 1");

	// Populate tables with data
	_ui->_A->setRowCount(A.rows());
	_ui->_A->setColumnCount(A.cols());
	_ui->_x->setRowCount(x.rows());
	_ui->_b->setRowCount(b.rows());
	for (Eigen::MatrixXd::Index i = 0; i < A.rows(); i++) {
		for (Eigen::MatrixXd::Index j = 0; j < A.cols(); j++) {
			QTableWidgetItem* item = new QTableWidgetItem(QString::number(A(i,j)));
			item->setData(Qt::ToolTipRole,QString::number(A(i,j)));
			_ui->_A->setItem(i,j,item);
		}
	}
	for (Eigen::VectorXd::Index i = 0; i < x.rows(); i++) {
		QTableWidgetItem* item = new QTableWidgetItem(QString::number(x[i]));
		item->setData(Qt::ToolTipRole,QString::number(x[i]));
		_ui->_x->setItem(i,0,item);
	}
	for (Eigen::VectorXd::Index i = 0; i < b.rows(); i++) {
		QTableWidgetItem* item = new QTableWidgetItem(QString::number(b[i]));
		item->setData(Qt::ToolTipRole,QString::number(b[i]));
		_ui->_b->setItem(i,0,item);
	}

	// Illustrate dimensions through scaling
	{
		QSizePolicy policy = _ui->_AHor->sizePolicy();
		policy.setHorizontalStretch(A.cols());
		_ui->_AHor->setSizePolicy(policy);
	}
	{
		QSizePolicy policy = _ui->_A->sizePolicy();
		policy.setVerticalStretch(A.rows());
		_ui->_A->setSizePolicy(policy);
		if (A.rows() >= A.cols())
			policy.setVerticalStretch(0);
		else
			policy.setVerticalStretch((A.cols()-A.rows())/2);
		_ui->_AVertTop->setSizePolicy(policy);
		_ui->_AVertBot->setSizePolicy(policy);
	}
	{
		QSizePolicy policy = _ui->_b->sizePolicy();
		policy.setVerticalStretch(b.rows());
		_ui->_b->setSizePolicy(policy);
		if (A.rows() >= A.cols())
			policy.setVerticalStretch(0);
		else
			policy.setVerticalStretch((A.cols()-A.rows())/2);
		_ui->_bVertTop->setSizePolicy(policy);
		_ui->_bVertBot->setSizePolicy(policy);
	}
	{
		QSizePolicy policy = _ui->_x->sizePolicy();
		if (x.rows() == 0)
			policy.setVerticalStretch(A.cols());
		else
			policy.setVerticalStretch(x.rows());
		_ui->_x->setSizePolicy(policy);
		if (A.rows() <= A.cols())
			policy.setVerticalStretch(0);
		else
			policy.setVerticalStretch((A.rows()-A.cols())/2);
		_ui->_xVertTop->setSizePolicy(policy);
		_ui->_xVertBot->setSizePolicy(policy);
	}
}

void EquationSystemWidget::showGraphics(rw::common::Ptr<rw::graphics::GroupNode> root, rw::common::Ptr<rw::graphics::SceneGraph> graph) {
	// no graphics in this widget
}

std::string EquationSystemWidget::getName() const {
	return "Equation System";
}

EquationSystemWidget::Dispatcher::Dispatcher() {
}

EquationSystemWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* EquationSystemWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	rw::common::Ptr<const LogEquationSystem> const system = entry.cast<const LogEquationSystem>();
	if (!(system == NULL))
		return new EquationSystemWidget(system, parent);
	RW_THROW("EquationSystemWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool EquationSystemWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogEquationSystem>() == NULL))
		return true;
	return false;
}

void EquationSystemWidget::showContextMenu(const QPoint& pos) {
	const QTableWidget* const widget = dynamic_cast<const QTableWidget*>(QObject::sender());
	RW_ASSERT(widget);
	const QPoint globalPos = widget->mapToGlobal(pos);
	QMenu myMenu;
#if RWS_USE_QT5
	if (_ui->_A->verticalHeader()->sectionResizeMode(0) == QHeaderView::Stretch)
		myMenu.addAction("&Expanded Mode");
	else
		myMenu.addAction("&Compact Mode");
#else
	if (_ui->_A->verticalHeader()->resizeMode(0) == QHeaderView::Stretch)
		myMenu.addAction("&Expanded Mode");
	else
		myMenu.addAction("&Compact Mode");
#endif
	myMenu.addAction("Copy as &Mathematica");
	myMenu.addAction("Copy as MAT&LAB");

	const QAction* const selectedItem = myMenu.exec(globalPos);
	if (selectedItem == myMenu.actions()[0]) {
#if RWS_USE_QT5
		if (_ui->_A->verticalHeader()->sectionResizeMode(0) == QHeaderView::Stretch) {
			// To expanded mode
			_ui->_A->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			_ui->_A->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			_ui->_b->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			_ui->_b->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			_ui->_x->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
			_ui->_x->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		} else {
			// To compact mode
			_ui->_A->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
			_ui->_A->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
			_ui->_b->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
			_ui->_b->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
			_ui->_x->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
			_ui->_x->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
		}
#else
		if (_ui->_A->verticalHeader()->resizeMode(0) == QHeaderView::Stretch) {
			// To expanded mode
			_ui->_A->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
			_ui->_A->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
			_ui->_b->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
			_ui->_b->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
			_ui->_x->verticalHeader()->setResizeMode(QHeaderView::ResizeToContents);
			_ui->_x->horizontalHeader()->setResizeMode(QHeaderView::ResizeToContents);
		} else {
			// To compact mode
			_ui->_A->verticalHeader()->setResizeMode(QHeaderView::Stretch);
			_ui->_A->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
			_ui->_b->verticalHeader()->setResizeMode(QHeaderView::Stretch);
			_ui->_b->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
			_ui->_x->verticalHeader()->setResizeMode(QHeaderView::Stretch);
			_ui->_x->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
		}
#endif
	} else if (selectedItem == myMenu.actions()[1]) {
		// Output the equation system in Mathematica format to clipboard
		const Eigen::MatrixXd& A = _system->A();
		const Eigen::VectorXd& x = _system->x();
		const Eigen::VectorXd& b = _system->b();
		std::stringstream fmt;
		fmt << "A={";
		for (Eigen::MatrixXd::Index i = 0; i < A.rows(); i++) {
			fmt << "{";
			for (Eigen::MatrixXd::Index j = 0; j < A.cols(); j++) {
				if (j == A.cols()-1)
					fmt << A(i,j);
				else
					fmt << A(i,j) << ",";
			}
			if (i != A.rows()-1)
				fmt << "}," << std::endl;
			else
				fmt << "}";
		}
		fmt << "};" << std::endl;
		fmt << "b={";
		for (Eigen::MatrixXd::Index i = 0; i < b.rows(); i++) {
			if (i == b.rows()-1)
				fmt << b[i];
			else
				fmt << b[i] << ",";
		}
		fmt << "};" << std::endl;
		if (x.size() > 0) {
			fmt << "x={";
			for (Eigen::MatrixXd::Index i = 0; i < x.rows(); i++) {
				if (i == x.rows()-1)
					fmt << x[i];
				else
					fmt << x[i] << ",";
			}
			fmt << "};" << std::endl;
		}
		std::string mstr = fmt.str();
		size_t pos = mstr.find("e");
		while (pos != std::string::npos) {
			mstr.replace(pos, 1, "*^");
			pos = mstr.find("e");
		}
		QClipboard* clip = QApplication::clipboard();
		clip->setText(QString::fromStdString(mstr));
	} else if (selectedItem == myMenu.actions()[2]) {
		// Output the equation system in MATLAB format to clipboard
		const Eigen::MatrixXd& A = _system->A();
		const Eigen::VectorXd& x = _system->x();
		const Eigen::VectorXd& b = _system->b();
		std::stringstream fmt;
		fmt << "A=[";
		for (Eigen::MatrixXd::Index i = 0; i < A.rows(); i++) {
			for (Eigen::MatrixXd::Index j = 0; j < A.cols(); j++) {
				if (j == A.cols()-1)
					fmt << A(i,j);
				else
					fmt << A(i,j) << " ";
			}
			if (i != A.rows()-1)
				fmt << ";" << std::endl;
		}
		fmt << "];" << std::endl;
		fmt << "b=[";
		for (Eigen::MatrixXd::Index i = 0; i < b.rows(); i++) {
			if (i == b.rows()-1)
				fmt << b[i];
			else
				fmt << b[i] << ";";
		}
		fmt << "];" << std::endl;
		if (x.size() > 0) {
			fmt << "x=[";
			for (Eigen::MatrixXd::Index i = 0; i < x.rows(); i++) {
				if (i == x.rows()-1)
					fmt << x[i];
				else
					fmt << x[i] << ";";
			}
			fmt << "];" << std::endl;
		}
		std::string mstr = fmt.str();
		/*size_t pos = mstr.find("e");
		while (pos != std::string::npos) {
			mstr.replace(pos, 1, "*^");
			pos = mstr.find("e");
		}*/
		QClipboard* clip = QApplication::clipboard();
		clip->setText(QString::fromStdString(mstr));
	}
}
