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

#include "SimulatorStatisticsWidget.hpp"
#include "MathematicaPlotWidget.hpp"

#include <rw/common/macros.hpp>
#include <rwsim/log/SimulatorStatistics.hpp>

#include "ui_SimulatorStatisticsWidget.h"

using namespace rwsim::log;
using namespace rwsimlibs::gui;
#include <boost/foreach.hpp>
SimulatorStatisticsWidget::SimulatorStatisticsWidget(rw::common::Ptr<const SimulatorStatistics> stats, QWidget* parent):
	QWidget(parent),
	_ui(new Ui::SimulatorStatisticsWidget()),
	_stats(stats),
	_mathematica(new MathematicaPlotWidget(this))
{
	_ui->setupUi(this);
	_ui->_image->setLayout(new QGridLayout());
	_ui->_image->layout()->addWidget(_mathematica);

	typedef SimulatorStatistics::DataSeries Data;
	const Data& data = stats->getSeries();
	_ui->_list->clear();
	for (Data::const_iterator it = data.begin(); it != data.end(); it++) {
		_ui->_list->addItem(QString::fromStdString(it->first));
	}
	connect(_ui->_list->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(changed(const QItemSelection &, const QItemSelection &)));

}

SimulatorStatisticsWidget::~SimulatorStatisticsWidget() {
}

std::string SimulatorStatisticsWidget::getName() const {
	return "Statistics";
}

void SimulatorStatisticsWidget::changed(const QItemSelection&, const QItemSelection&) {
	typedef SimulatorStatistics::DataSeries Data;
	const QModelIndexList indexes = _ui->_list->selectionModel()->selectedIndexes();
	const Data& data = _stats->getSeries();
	Data selectedData;
	foreach (const QModelIndex& index, indexes) {
		RW_ASSERT(index.column() == 0);
		const std::string name = index.data().toString().toStdString();
		RW_ASSERT(data.find(name) != data.end());
		selectedData[name] = data.find(name)->second;
	}
	if (selectedData.size() == 1) {
		const std::vector<double> values = selectedData.begin()->second;
		std::vector<double> x(values.size());
		for (std::size_t i = 0; i < values.size(); i++)
			x[i] = i;
		_mathematica->listPlot(x,values);
	}
}
