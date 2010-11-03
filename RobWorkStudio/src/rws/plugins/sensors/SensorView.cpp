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

using namespace rws;
using namespace rw::common;
using namespace rw::sensor;
using namespace rwlibs::drawable;
using namespace rwlibs::simulation;


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
    _camera->acquire();
    const Image* img = _camera->getImage();
    _pImageView->display(*img);
}



Scan25DView::Scan25DView(QWidget* parent):
    SensorView(parent),
    _scanner(NULL)
{
    _pGLView = new GLView(NULL, this);

    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
    //layout->addWidget(new QLabel("Test Label"));
    layout->addWidget(_pGLView);


    _scanRender = ownedPtr( new RenderScan() );
    _pGLView->addDrawable(new Drawable(_scanRender));

}

void Scan25DView::initialize(rw::sensor::Scanner25D::Ptr scanner) {
    _scanner = scanner;
}

void Scan25DView::makeCurrent() {
    _pGLView->makeCurrent();
}

void Scan25DView::update() {

    if(_scanner != NULL && _scanner->isScanReady() ){
        const Image25D& img = _scanner->getImage();
        _scanRender->setScan(img);
    }

    _scanner->acquire();
    _pGLView->update();
}







Scan2DView::Scan2DView(QWidget* parent):
    SensorView(parent),
    _scanner(NULL)
{
    _pGLView = new GLView(NULL, this);

    QVBoxLayout* layout = new QVBoxLayout(this);
    setLayout(layout);
    layout->addWidget(_pGLView);

    _scanRender = ownedPtr( new RenderScan() );
    _pGLView->addDrawable(new Drawable(_scanRender));

}

void Scan2DView::initialize(SimulatedScanner2D::Ptr scanner) {
    _scanner = scanner;
}

void Scan2DView::makeCurrent() {
    _pGLView->makeCurrent();
}

void Scan2DView::update() {

    if(_scanner != NULL && _scanner->isScanReady() ){
        const Scan2D& scan = _scanner->getScan();
        _scanRender->setScan(scan);
    }

    _scanner->acquire();
    _pGLView->update();
}

