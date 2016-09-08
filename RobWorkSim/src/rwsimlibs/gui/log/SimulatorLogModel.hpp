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

#ifndef RWSIMLIBS_GUI_SIMULATORLOGMODEL_HPP_
#define RWSIMLIBS_GUI_SIMULATORLOGMODEL_HPP_

/**
 * @file SimulatorLogModel.hpp
 *
 * \copydoc rwsimlibs::gui::SimulatorLogModel
 */

#include <rw/common/Ptr.hpp>

#include <QAbstractItemModel>
#include <QColor>

namespace rwsim { namespace log { class SimulatorLog; } }

namespace rwsimlibs {
namespace gui {
//! @addtogroup rwsimlibs_gui

//! @{
//! @brief Model of a simulator log for Qt Tree view.
class SimulatorLogModel: public QAbstractItemModel {
	Q_OBJECT
public:
	/**
	 * @brief Construct new item model for the log tree.
	 * @param parent [in] the parent Qt widget. Ownership is shared by the caller and the parent widget.
	 */
	SimulatorLogModel(QObject *parent);

	//! @brief Destructor.
	virtual ~SimulatorLogModel();

	/**
	 * @brief Set the log root.
	 * @param root [in] the log root.
	 */
	void setRoot(rw::common::Ptr<const rwsim::log::SimulatorLog> root);

	/**
	 * @brief Compare with a different log.
	 * @param info [in] the other simulation log.
	 */
	void compare(rw::common::Ptr<const rwsim::log::SimulatorLog> info);

	/**
	 * @brief Find a log entity from index.
	 * @param index [in] the index.
	 * @return the log entity.
	 */
    const rwsim::log::SimulatorLog* nodeFromIndex(const QModelIndex &index) const;

    /** @name Implementation of Qt interface.
     *  Implementation of Qt functions. See http://doc.qt.io/qt-4.8/qabstractitemmodel.html .
     */
    ///@{
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent = QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const;
    QModelIndex parent(const QModelIndex &child) const;
    void update();
    bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole);
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
    ///@}

private:
	void compare(rw::common::Ptr<const rwsim::log::SimulatorLog> a, rw::common::Ptr<const rwsim::log::SimulatorLog> b);
	void compareFailSubTree(rw::common::Ptr<const rwsim::log::SimulatorLog> a);

private:
    rw::common::Ptr<const rwsim::log::SimulatorLog> _root;
    std::map<const rwsim::log::SimulatorLog*, QColor> _bgColor;
};
//! @}
} /* namespace gui */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_GUI_SIMULATORLOGMODEL_HPP_ */
