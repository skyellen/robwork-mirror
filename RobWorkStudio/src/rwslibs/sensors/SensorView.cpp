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


#include "SensorView.hpp"

#include <QVBoxLayout>

#include <rw/sensor/Camera.hpp>
#include <rw/sensor/ImageUtil.hpp>

#include <rwlibs/opengl/RenderScan.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>

#include <rws/ImageView.hpp>

using namespace rws;
using namespace rw::common;
using namespace rw::sensor;
using namespace rwlibs::simulation;
using namespace rwlibs::opengl;
using namespace rw::graphics;


void SensorView::closeEvent(QCloseEvent* event) {
    emit viewClosed(this);
}

CameraView::CameraView(Camera::Ptr camera, QWidget* parent):
    SensorView(parent),
    _camera(camera)
{
    _pImageView = new ImageView();

    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
    //layout->addWidget(new QLabel("Test Label"));
    layout->addWidget(_pImageView);
}

void CameraView::update() {
    const Image* img = _camera->getImage();
    _pImageView->display(*img);
    _camera->acquire();
}



Scan25DView::Scan25DView(QWidget* parent):
    SensorView(parent),
    _scanner(NULL)
{
    _pImageView = new ImageView();

    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
    //layout->addWidget(new QLabel("Test Label"));
    layout->addWidget(_pImageView);
}

void Scan25DView::initialize(rwlibs::simulation::SimulatedScanner25D::Ptr scanner) {
    _scanner = scanner;
}

void Scan25DView::makeCurrent() {
}

void Scan25DView::update() {

    if(_scanner != NULL) {
    	if (_scanner->isScanReady() ){
    		//Image::Ptr img = _scanner->getImage().asImage(_scanner->getRange().first, _scanner->getRange().second);
    		Image::Ptr img = ImageUtil::makeDepthImage(_scanner->getScan());

    		//_scanRender->setScan(img);
    		// convert to depth image
    		_pImageView->display(*img);

    	}
    	_scanner->acquire();
	}
}







Scan2DView::Scan2DView(QWidget* parent):
    SensorView(parent),
    _scanner(NULL)
{
    _pImageView = new ImageView();

    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
    //layout->addWidget(new QLabel("Test Label"));
    layout->addWidget(_pImageView);
}

void Scan2DView::initialize(rw::common::Ptr<SimulatedScanner2D> scanner) {
    _scanner = scanner;
}

void Scan2DView::makeCurrent() {

}

void Scan2DView::update() {
	if(_scanner != NULL) {
		if (_scanner->isScanReady() ){
			//Image::Ptr img = _scanner->getImage().asImage(_scanner->getRange().first, _scanner->getRange().second);
			Image::Ptr img = ImageUtil::makeDepthImage(_scanner->getScan());

			//_scanRender->setScan(img);
			// convert to depth image
			_pImageView->display(*img);

		}
		_scanner->acquire();
	}
}

