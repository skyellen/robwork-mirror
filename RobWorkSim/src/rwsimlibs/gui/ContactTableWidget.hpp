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

#ifndef RWSIMLIBS_GUI_CONTACTTABLEWIDGET_HPP_
#define RWSIMLIBS_GUI_CONTACTTABLEWIDGET_HPP_

/**
 * @file ContactTableWidget.hpp
 *
 * \copydoc ContactTableWidget
 */

#include <QTableWidget>

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw { namespace graphics { class GroupNode; } }
namespace rw { namespace graphics { class SceneGraph; } }
namespace rwsim { namespace contacts { class Contact; } }

QT_BEGIN_NAMESPACE

/**
 * @brief A table widget customised for rwsim::contacts::Contact types.
 */
class ContactTableWidget: public QTableWidget {
	Q_OBJECT
public:
	/**
	 * @brief Constructor.
	 * @param parent [in] (optional) the owner of this widget.
	 */
	ContactTableWidget(QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~ContactTableWidget();

	/**
	 * @brief Update the widget with a new contact set.
	 * @param contacts [in] a vector of contacts.
	 */
	virtual void setContacts(const std::vector<rwsim::contacts::Contact>& contacts);

	/**
	 * @brief Add graphics as drawables to a scene-graph.
	 * @param root [in] the node to add drawables to.
	 * @param graph [in] the scene graph.
	 */
	virtual void showGraphics(rw::common::Ptr<rw::graphics::GroupNode> root, rw::common::Ptr<rw::graphics::SceneGraph> graph);

	//! @brief Select all contacts.
	virtual void selectAll();

signals:
	//! @brief Signal is emitted if the graphics is updated.
	void graphicsUpdated();

public slots:
	void clearContents();

private slots:
	void contactSetChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);

private:
	QString toQString(const rw::math::Vector3D<>& vec);
	QString toQString(const rwsim::contacts::Contact& contact);

private:
	std::vector<rwsim::contacts::Contact> _contacts;
    rw::common::Ptr<rw::graphics::GroupNode> _root;
    rw::common::Ptr<rw::graphics::SceneGraph> _graph;
};

QT_END_NAMESPACE

#endif /* RWSIMLIBS_GUI_CONTACTTABLEWIDGET_HPP_ */
