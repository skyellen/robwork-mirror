#include "TactileSensorDialog.hpp"

#include <iostream>
#include <fstream>

#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <dynamics/RigidBody.hpp>

#include <simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <loaders/ScapePoseFormat.hpp>


using namespace dynamics;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace boost::numeric::ublas;
using namespace rwlibs::simulation;


#define RW_DEBUGS( str ) //std::cout << str  << std::endl;

namespace {


}

TactileSensorDialog::TactileSensorDialog(
                                    dynamics::DynamicWorkcell *dwc,
                                    QWidget *parent):
    QDialog(parent),
    _dwc(dwc),
    _nrOfPadsH(3)
{

	RW_ASSERT( _dwc );
    setupUi(this);
    std::vector<SimulatedSensorPtr> sensors = _dwc->getSensors();
    BOOST_FOREACH(SimulatedSensorPtr& sensor, sensors){
    	if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>(sensor.get())){
    	    _tsensors.push_back(tsensor);
    	}
    }
    _scene = _gview->scene();
    if(_scene==NULL){
        _scene = new QGraphicsScene();
        _gview->setScene(_scene);
    }



    initTactileInput();

    connect(_updateBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_saveViewBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_loadDataBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_saveDataBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));

    _gview->update();

    connect(_maxPressureSpin,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
}

void TactileSensorDialog::initTactileInput(){
    if(_tsensors.size()==0)
        return;
    // we just try drawing one of the tactile sensors here

    double maxPressure = 0;
    int padOffsetX = 0;
    int padOffsetY = 0;

    qreal rectW(10),rectH(10);
    int maxWidth = 0;
    // Create rectangles using the matrixMask
    for(size_t sensorIdx=0; sensorIdx<_tsensors.size();sensorIdx++){
        matrix<float> data = _tsensors[sensorIdx]->getTexelData();
        maxPressure = std::max(maxPressure, _tsensors[sensorIdx]->getPressureLimit().second);
        _w = data.size1();
        _h = data.size2();
        int w = _w;
        int h = _h;
        std::vector<QGraphicsRectItem*> rectItems(w*h+1);
        maxWidth = std::max(maxWidth,w);
        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
                //float val = data(x,y);
                rectItems[x+y*w] = _scene->addRect(padOffsetY+y*rectH, padOffsetX+x*rectW, rectH, rectW);
                rectItems[x+y*w]->setBrush(QBrush(QColor(0,0,254), Qt::SolidPattern) );
            }
        }
        padOffsetY += (int) ( h*rectH+4*rectH );
        if( (sensorIdx+1)%_nrOfPadsH == 0){
            padOffsetX += (int)( maxWidth*rectW+4*rectW );
            padOffsetY = 0;
            maxWidth = 0;
        }
        _rectItems.push_back(rectItems);
    }
    _maxPressureSpin->setMaximum(maxPressure);
    _maxPressureSpin->setValue(maxPressure);
    _gview->update();
}

void TactileSensorDialog::drawTactileInput(){
    double maxPressure = _maxPressureSpin->value();
    if(_autoScaleBox->isChecked()){
        maxPressure = 100;
        for(size_t sensorIdx=0; sensorIdx<_values.size();sensorIdx++){
            matrix<float> data = _values[sensorIdx];
            int w = _w;
            int h = _h;
            for(int x=0;x<w;x++){
                for(int y=0;y<h;y++){
                    float val = data(x,y);
                    maxPressure = std::max((double)val,maxPressure);
                }
            }
        }
        _maxPressureSpin->setValue(maxPressure);
    }

    for(size_t sensorIdx=0; sensorIdx<_values.size();sensorIdx++){
        matrix<float> data = _values[sensorIdx];
        int w = _w;
        int h = _h;
        std::vector<QGraphicsRectItem*> &rectItems = _rectItems[sensorIdx];
        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
                float val = data(x,y);
                int mval = std::min((int)( (val/maxPressure)*254),254);
                rectItems[x+y*w]->setBrush(QBrush(QColor(mval,0,255-mval), Qt::SolidPattern) );
            }
        }
    }
    _gview->update();
    /*
    matrix<float> data = _tsensors[0]->getTexelData();
    std::cout << data << std::endl;

    int w = data.size1();
    int h = data.size2();
    std::cout << "Size: " << w << " " << h << std::endl;



    // Create rectangles using the matrixMask
    for(int x=0;x<w;x++){
        for(int y=0;y<h;y++){
            float val = data(x,y);
            int mval = std::max((int)(val/maxPressure*255),254);
            _rectItems[x+y*w]->setBrush(QBrush(QColor(mval,0,0), Qt::SolidPattern) );
        }
    }

    */
}

void TactileSensorDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _updateBtn){
        _values.clear();
        BOOST_FOREACH(TactileArraySensor *sensor, _tsensors){
            _values.push_back(sensor->getTexelData());
        }
        drawTactileInput();

    } else if( obj==_saveViewBtn ){
        std::cout << "Save btn pushed!" << std::endl;
        QImage img(640,480,QImage::Format_RGB32);
        QPainter painter(&img);
        _gview->scene()->render(&painter);
        double time = TimerUtil::currentTime();
        std::stringstream sstr;
        int s = (int)(time);
        sstr << "c:/tmp/ImageSave_"<< s << ".png";
        img.save(sstr.str().c_str());
    } else if(obj==_loadDataBtn){
        QString selectedFilter;
        QString filename = QFileDialog::getOpenFileName(
            this,
            "Open Drawable", // Title
            ".", // Directory
            "All supported ( *.txt )"
            " \n( *.txt )"
            " \n All ( *.* )",
            &selectedFilter);
        std::string file = filename.toStdString();
        _values.clear();
        std::ifstream ifile(file.c_str(), std::ifstream::in);


        matrix<float> mat(13,6);
        for(int i=0;i<13;i++){
            for(int j=0;j<6;j++)
                ifile >> mat(i,j);
        }
        _values.push_back(mat);
        std::cout << mat;
        for(int i=0;i<13;i++){
            for(int j=0;j<6;j++)
                ifile >>mat(i,j);
        }
        _values.push_back(mat);
        std::cout << mat;
        for(int i=0;i<13;i++){
            for(int j=0;j<6;j++)
                ifile >>mat(i,j);
        }
        _values.push_back(mat);
        std::cout << mat;


        ifile.close();
        drawTactileInput();
    }
}

void TactileSensorDialog::changedEvent(){
    QObject *obj = sender();
    if( obj == _maxPressureSpin ){
        drawTactileInput();
    }
}

void TactileSensorDialog::updateState(){
    _values.clear();
    BOOST_FOREACH(TactileArraySensor *sensor, _tsensors){
        _values.push_back(sensor->getTexelData());
    }
    drawTactileInput();
    if(_saveCheckBox->isChecked()){
        Frame *world = _dwc->getWorkcell()->getWorldFrame();
        std::string gqual = world->getPropertyMap().get<std::string>("GraspQuality",std::string("NO"));
        QImage img(640,480,QImage::Format_RGB32);
        QPainter painter(&img);
        _gview->scene()->render(&painter);
        double time = TimerUtil::currentTime();
        std::stringstream sstr;
        int s = (int)(time);
        sstr << "c:/tmp/ImageSave_"<< s << "_" << gqual <<  ".png";
        img.save(sstr.str().c_str());
    }
}
