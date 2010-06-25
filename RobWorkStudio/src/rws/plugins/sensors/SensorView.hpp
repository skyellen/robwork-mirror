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

#ifndef RW_STUDIO_SENSORVIEW
#define RW_STUDIO_SENSORVIEW

#include <QWidget>

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Scanner25D.hpp>
#include <rw/sensor/Scanner2D.hpp>
#include <rwlibs/drawable/RenderScan.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>



#include <rws/components/imageview/ImageView.hpp>
#include <rws/components/glview/GLView.hpp>

namespace rws {

class SensorView: public QWidget {
    Q_OBJECT
public:
    SensorView(QWidget* parent = NULL):
        QWidget(parent)
    {

    }

    ~SensorView() {
        
    }

    virtual void update() = 0;

    virtual void makeCurrent() = 0;
protected:
    void closeEvent(QCloseEvent* event);

signals:
    void viewClosed(SensorView* widget);
};

typedef rw::common::Ptr<SensorView> SensorViewPtr;


class CameraView: public SensorView {
public:
    CameraView(rw::sensor::CameraPtr camera, QWidget* parent = NULL);

    virtual void update();

    virtual void makeCurrent() {};

private:
    rw::sensor::CameraPtr _camera;
    ImageView* _pImageView;    
};


class Scan25DView: public SensorView {
public:
    Scan25DView(QWidget* parent = NULL);

    virtual void initialize(rw::sensor::Scanner25DPtr scanner);

    virtual void update();

    virtual void makeCurrent();

private:
    rw::sensor::Scanner25DPtr _scanner;
    rwlibs::drawable::RenderScanPtr _scanRender;
    GLView* _pGLView;

};

class Scan2DView: public SensorView {
public:
    Scan2DView(QWidget* parent = NULL);

    virtual void initialize(rwlibs::simulation::SimulatedScanner2DPtr scanner);

    virtual void update();

    virtual void makeCurrent();

private:
    rwlibs::simulation::SimulatedScanner2DPtr _scanner;
    rwlibs::drawable::RenderScanPtr _scanRender;
    GLView* _pGLView;

};

} //end namespace rws;

#endif
