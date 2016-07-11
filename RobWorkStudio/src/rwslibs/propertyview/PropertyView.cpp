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


#include "PropertyView.hpp"

#include <QGridLayout>
#include <QComboBox>

#include <rws/RobWorkStudio.hpp>

#include <rws/propertyview/PropertyViewEditor.hpp>

#include <boost/bind.hpp>

using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::common;
using namespace rws;

PropertyView::PropertyView() :
    RobWorkStudioPlugin("PropertyView", QIcon(":/propertyview.png")),
    _workcell(NULL)
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    int row = 0;

/// pLayout->addWidget(new QPushButton("Tst"), row++, 1);

	_cmbFrames = new QComboBox();
	_cmbFrames->setMinimumHeight(10);
	connect(_cmbFrames, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(frameChanged(const QString& )));
	pLayout->addWidget(_cmbFrames, row++, 0);

	_inspector = new PropertyViewEditor(this);
	pLayout->addWidget(_inspector, row++, 0);
	connect(_inspector, SIGNAL(propertyChanged(const std::string&)), this, SLOT(propertyChanged(const std::string&)));


	// Set widget to QDockArea
    setWidget(base);
}

PropertyView::~PropertyView()
{
}

void PropertyView::initialize() {
    getRobWorkStudio()->frameSelectedEvent().add(boost::bind(&PropertyView::frameSelectedListener, this, _1), this);
//    getRobWorkStudio()->addFrameSelectedListener(boost::bind(&PropertyView::frameSelectedListener, this, _1));
}


void PropertyView::open(WorkCell* workcell)
{
	_workcell = workcell;
	_state = getRobWorkStudio()->getState();
	_cmbFrames->clear();
	_inspector->setPropertyMap(NULL);
	if (workcell != NULL) {
		_updating = true;
		addFrame(workcell->getWorldFrame());
		_updating = false;
	}
	if (_cmbFrames->count() > 0)
		frameChanged(_cmbFrames->itemText(0));

}

void PropertyView::addFrame(const Frame* frame) {
    State state = getRobWorkStudio()->getState();
	_cmbFrames->addItem(frame->getName().c_str());
	
	Frame::const_iterator_pair frames = frame->getChildren(state);
	
	for (Frame::const_iterator it = frames.first; it != frames.second; ++it) {		
		addFrame(&(*it));
	}
}
 


void PropertyView::close()
{
}

void PropertyView::frameSelectedListener(Frame* frame) {
    _updating = true;
    if (frame != NULL) {
        if (_cmbFrames->currentText() != frame->getName().c_str()) {
            int index = _cmbFrames->findText(frame->getName().c_str());
            _cmbFrames->setCurrentIndex(index);
        }
        //rw::common::Ptr< PropertyMap > map = rw::common::ownedPtr( new PropertyMap( frame->getPropertyMap() ) );
        _inspector->setPropertyMap( &frame->getPropertyMap() );
        //_inspector->setPropertyMap(NULL);
    }
    else
        _inspector->setPropertyMap(NULL);
    _updating = false;

}

/*void PropertyView::frameSelectedHandler(Frame* frame, RobWorkStudioPlugin* sender) {
    if (frame != NULL) {
        if (_cmbFrames->currentText() != frame->getName().c_str()) {
            int index = _cmbFrames->findText(frame->getName().c_str());
            _cmbFrames->setCurrentIndex(index);
        }
        _inspector->setPropertyMap(&(frame->getPropertyMap()));
    }
    else
        _inspector->setPropertyMap(NULL);
}
*/
void PropertyView::frameChanged(const QString& item) {
	if (_updating)
		return;

    Frame* frame = _workcell->findFrame(item.toStdString());
    getRobWorkStudio()->frameSelectedEvent().fire(frame);
}


void PropertyView::propertyChanged(const std::string& identifier) {
    // getRobWorkStudio()->getWorkCellGLDrawer()->clearCache();
    // getRobWorkStudio()->getCollisionDetector()->clearCache();

    getRobWorkStudio()->updateAndRepaint();
//    updateSignal();
}

#ifndef RWS_USE_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN2(PropertyView, PropertyView)
#endif
#endif
