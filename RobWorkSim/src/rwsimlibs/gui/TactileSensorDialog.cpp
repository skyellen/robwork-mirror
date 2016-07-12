#include "TactileSensorDialog.hpp"

#include <sstream>
#include <fstream>

#include <QGraphicsTextItem>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QFileDialog>
#include <QWheelEvent>

#include <boost/foreach.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>

#include "ui_TactileSensorDialog.h"

using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rwlibs::simulation;

namespace {

Moment calcMoments2D(Eigen::MatrixXf& mat, double low_thres){
		Eigen::MatrixXf covar(Eigen::MatrixXf::Zero(2, 2));
        Vector2D<float> centroid(0,0);

        //float totalval = 0;
        // we only use triangle centers the vertices directly
        for(Eigen::DenseIndex x=0; x<mat.rows();x++){
        	for(Eigen::DenseIndex y=0; y<mat.cols(); y++){
        		if(mat(x,y)>low_thres){
        			Vector2D<float> val(x*mat(x,y),y*mat(x,y));
        			centroid += val;
                    for(size_t j=0;j<2;j++)
                        for(size_t k=0;k<2;k++)
                            covar(j,k) += val(j)*val(k);

        		}
        	}
        }
        if( centroid(0)<0.01 && centroid(1)<0.01)
        	return Moment();

        const size_t n = mat.rows()*mat.cols();
        for(size_t j=0;j<2;j++)
            for(size_t k=0;k<2;k++)
                covar(j,k) = covar(j,k)-centroid[j]*centroid[k]/n;

        typedef std::pair<Eigen::MatrixXf, Eigen::VectorXf > ResultType;
        //std::cout << "COVAR: " << covar << std::endl;
        ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

        size_t maxEigIdx=1, minEigIdx=0;
        double maxEigVal = res.second(maxEigIdx);
        double minEigVal = res.second(minEigIdx);
        if(maxEigVal<minEigVal)
        	std::swap(minEigIdx,maxEigIdx);

        Vector2D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx) );
        Vector2D<> minAxis( res.first(0,minEigIdx), res.first(1,minEigIdx) );

        // now scale the axes to the actual extend
        Rotation2D<> rot(normalize(maxAxis),normalize(minAxis));
        Rotation2D<> rotInv = inverse(rot);

        Vector2D<> max(-100,-100),min(100,100);
        for(Eigen::DenseIndex x=0; x<mat.rows();x++){
        	for(Eigen::DenseIndex y=0; y<mat.cols(); y++){
        		if(mat(x,y)>low_thres){
        			Vector2D<> p = rotInv*Vector2D<>((double)x,(double)y);
                    if( p(0)>max(0) ) max(0) = p(0);
                    else if( p(0)<min(0) ) min(0) = p(0);
                    if( p(1)>max(1) ) max(1) = p(1);
                    else if( p(1)<min(1) ) min(1) = p(1);
        		}
        	}
        }
        maxAxis = normalize(maxAxis)*(max(0)-min(0));
        minAxis = normalize(minAxis)*(max(1)-min(1));
        Moment mom;
        mom.first = minAxis/2;
        mom.second = maxAxis/2;
        mom.center = rot*Vector2D<>( (max(0)-min(0))/2+min(0) , (max(1)-min(1))/2+min(1) );
        return mom;
	}

}

TactileSensorDialog::TactileSensorDialog(
                                    DynamicWorkCell *dwc,
                                    QWidget *parent):
    QDialog(parent),
    _dwc(dwc),
    _renderingToImage(false),
    _saveCnt(0),
    _nrOfPadsH(3)
{

	RW_ASSERT( _dwc );
	_ui = new Ui::TactileSensorDialog();
	_ui->setupUi(this);
    std::vector<SimulatedSensor::Ptr> sensors = _dwc->getSensors();
    BOOST_FOREACH(SimulatedSensor::Ptr& sensor, sensors){
    	if( TactileArraySensor *tsensor = dynamic_cast<TactileArraySensor*>(sensor.get())){
    	    _tsensors.push_back(tsensor);
    	}
    }
    _scene = _ui->_gview->scene();
    if(_scene==NULL){
        _scene = new QGraphicsScene();
        _ui->_gview->setScene(_scene);
    }

    initTactileInput();

    connect(_ui->_updateBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_ui->_saveViewBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_ui->_loadDataBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));
    connect(_ui->_saveDataBtn, SIGNAL(pressed()), this, SLOT(btnPressed()));

    connect(_ui->_uppBoundSpin,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
    connect(_ui->_lowBoundSpin,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );

    _ui->_gview->update();

    connect(_ui->_maxPressureSpin,SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
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
    _dims.resize(_tsensors.size());
    for(size_t sensorIdx=0; sensorIdx<_tsensors.size();sensorIdx++){
        maxPressure = std::max(maxPressure, _tsensors[sensorIdx]->getPressureLimit().second);
        _dims[sensorIdx].first = (int)_tsensors[sensorIdx]->getWidth();
        _dims[sensorIdx].second = (int)_tsensors[sensorIdx]->getHeight();

        int w = _dims[sensorIdx].first;
        int h = _dims[sensorIdx].second;
        std::vector<QGraphicsRectItem*> rectItems(w*h+1);
        maxWidth = std::max(maxWidth,w);
        std::string sname = _tsensors[sensorIdx]->getName();
        QGraphicsTextItem *titem = _scene->addText(QString(sname.c_str()));
        titem->setPos(padOffsetY, padOffsetX-25);
        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
                //float val = data(x,y);
                rectItems[x+y*w] = _scene->addRect(padOffsetY+y*rectH, padOffsetX+x*rectW, rectH, rectW);
                rectItems[x+y*w]->setBrush(QBrush(QColor(0,0,254), Qt::SolidPattern) );
                rectItems[x+y*w]->setZValue(5);
            }
        }
        padOffsetY += (int) ( h*rectH+4*rectH );
        if( (sensorIdx+1)%_nrOfPadsH == 0){
            padOffsetX += (int)( maxWidth*rectW+4*rectW );
            padOffsetY = 0;
            maxWidth = 0;
        }
        _rectItems.push_back(rectItems);

        // also add stuff to visualize center mass
        QGraphicsEllipseItem *item = _scene->addEllipse(QRectF(0,0,5,5));
        item->setBrush(QBrush(QColor(0,254,0), Qt::SolidPattern) );
        item->setZValue(10);
        _centerItems.push_back(item);

        // and vizualization of moments
        QGraphicsLineItem *ritem = _scene->addLine(QLineF(0,0,1,1));
        //ritem->setBrush(QBrush(QColor(0,128,128), Qt::SolidPattern) );
        ritem->setPen(QPen( QBrush(QColor(0,128,128), Qt::SolidPattern),2) );
        ritem->setZValue(10);
        _momentItems.push_back(ritem);

        QGraphicsLineItem *r2item = _scene->addLine(QLineF(0,0,1,1));
        r2item->setPen(QPen( QBrush(QColor(128,128,0), Qt::SolidPattern),2) );
        r2item->setZValue(10);
        _momentSecItems.push_back(r2item);


    }
    _ui->_maxPressureSpin->setMaximum(maxPressure);
    _ui->_maxPressureSpin->setValue(maxPressure);
    _ui->_gview->update();
}

void TactileSensorDialog::drawTactileInput(){
    double maxPressure = _ui->_maxPressureSpin->value();
    if(_ui->_autoScaleBox->isChecked()){
        maxPressure = 100;
        for(size_t sensorIdx=0; sensorIdx<_values.size();sensorIdx++){
            Eigen::MatrixXf data = _values[sensorIdx];
            int w = _dims[sensorIdx].first;
            int h = _dims[sensorIdx].second;
            for(int x=0;x<w;x++){
                for(int y=0;y<h;y++){
                    float val = data(x,y);
                    maxPressure = std::max((double)val,maxPressure);
                }
            }
        }
        _ui->_maxPressureSpin->setValue(maxPressure);
    }

    for(size_t sensorIdx=0; sensorIdx<_values.size();sensorIdx++){
        Eigen::MatrixXf data = _values[sensorIdx];
        int w = _dims[sensorIdx].first;
        int h = _dims[sensorIdx].second;

    	if(data.rows()!= (Eigen::DenseIndex)w){
    		RW_WARN(data.rows()<<"!="<<w);
    		continue;
    	}
    	if(data.cols()!= (Eigen::DenseIndex)h){
    		RW_WARN(data.cols()<<"!="<<h);
    		continue;
    	}

        std::vector<QGraphicsRectItem*> &rectItems = _rectItems[sensorIdx];
        for(int x=0;x<w;x++){
            for(int y=0;y<h;y++){
            	if(rectItems.size()<= (size_t)(x+y*w)){
            		RW_WARN(rectItems.size()<<"<=" << x+y*w);
            		continue;
            	}

                float val = data(x,y);
                int mval = Math::clamp((val/maxPressure)*254,0,254);
                rectItems[x+y*w]->setBrush(QBrush(QColor(mval,0,255-mval), Qt::SolidPattern) );
            }
        }
    }
    // if features is enabled then we draw them as well
    if( _ui->_centerOfMassBtn->isChecked() ){

    	for(std::size_t i=0;i<_centers.size();i++){
    		//std::cout << "center : " << _centers[i] << std::endl;
    		// we need to map the coordinates into graphics view coordinates

    		// we get the coordinate of the first texel in the view
    		qreal offsetx = _rectItems[i][0]->rect().x();
    		qreal offsety = _rectItems[i][0]->rect().y();
    		std::cout << offsetx << " " <<  offsety << std::endl;
    		// 10 is the width of the tactile sensors so we use that
    		_centerItems[i]->setPos(_centers[i](1)*10+2.5+offsetx,_centers[i](0)*10+2.5+offsety);
    		std::cout << "Pos: " << _centers[i](1)*10+5+offsetx << " " << _centers[i](0)*10+5+offsety << std::endl;
    	}
    }

    if( _ui->_momentsBtn->isChecked() ){
    	for(std::size_t i=0;i<_moments.size();i++){

    		qreal offsetx = _rectItems[i][0]->rect().x();
    		qreal offsety = _rectItems[i][0]->rect().y();

    		Moment m = _moments[i];
    		_momentItems[i]->setPos(m.center(1)*10+2.5+offsetx, m.center(0)*10+2.5+offsety);
    		//_momentItems[i]->setPos(_centers[i](1)*10+2.5+offsetx, _centers[i](0)*10+2.5+offsety);
    		_momentItems[i]->setLine(-m.second(1)*10+2.5,-m.second(0)*10+2.5,
    				m.second(1)*10+2.5,m.second(0)*10+2.5);
    		//std::cout << "Pos: " << _centers[i](1)*10+5+offsetx << " " << _centers[i](0)*10+5+offsety << std::endl;

    		_momentSecItems[i]->setPos(m.center(1)*10+2.5+offsetx, m.center(0)*10+2.5+offsety);
    		//_momentSecItems[i]->setPos(_centers[i](1)*10+2.5+offsetx, _centers[i](0)*10+2.5+offsety);
    		_momentSecItems[i]->setLine(-m.first(1)*10+2.5,-m.first(0)*10+2.5,
										 m.first(1)*10+2.5,m.first(0)*10+2.5);

    	}
    }

    _ui->_gview->update();
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
    if( obj == _ui->_updateBtn){
        //_values.clear();
        //BOOST_FOREACH(TactileArraySensor *sensor, _tsensors){
        //    _values.push_back(sensor->getTexelData());
        //}
        drawTactileInput();

    } else if( obj==_ui->_saveViewBtn ){
        std::cout << "Save btn pushed!" << std::endl;
        QImage img(640,480,QImage::Format_RGB32);
        QPainter painter(&img);
        _ui->_gview->render(&painter);
        double time = TimerUtil::currentTime();
        std::stringstream sstr;
        int s = (int)(time);
        sstr << "TactileImageSave_"<< s << ".png";
        img.save(sstr.str().c_str());
    } else if(obj==_ui->_loadDataBtn){
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


        Eigen::MatrixXf mat(13,6);
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
    if( obj == _ui->_maxPressureSpin ){
        drawTactileInput();
    } else if(obj == _ui->_lowBoundSpin){
    	if(_ui->_lowBoundSpin->value()>_ui->_uppBoundSpin->value()){
    	    _ui->_uppBoundSpin->setValue(_ui->_lowBoundSpin->value());
    	}
    } else if(obj == _ui->_uppBoundSpin){
    	if(_ui->_lowBoundSpin->value()>_ui->_uppBoundSpin->value()){
    	    _ui->_lowBoundSpin->setValue(_ui->_uppBoundSpin->value());
    	}
    }
}

void TactileSensorDialog::setState(const rw::kinematics::State& state){
	_values.clear();
    BOOST_FOREACH(TactileArraySensor *sensor, _tsensors){
        _values.push_back(sensor->getTexelData(state));
    }
    // now detect features in the tactile images enabled
    detectFeatures();
    drawTactileInput();
    if(_ui->_saveCheckBox->isChecked()){
        if(_renderingToImage)
            return;
        _renderingToImage = true;
        Frame *world = _dwc->getWorkcell()->getWorldFrame();
        std::string gqual = world->getPropertyMap().get<std::string>("GraspQuality",std::string("NO"));
        QImage img(640,480,QImage::Format_RGB32);
        QPainter painter(&img);
        painter.setBackground( QBrush(QColor(Qt::white)));
        _ui->_gview->render(&painter);
        //double time = TimerUtil::currentTime();
        std::stringstream sstr;
        //int s = (int)(time);
        sstr << "TactileImageSave_"<< _saveCnt << ".png";
        _saveCnt++;
        img.save(sstr.str().c_str());
        _renderingToImage = false;
    }
}

void TactileSensorDialog::detectFeatures(){
	if( _ui->_centerOfMassBtn->isChecked() || _ui->_momentsBtn->isChecked()){
		detectCenterMass();
	}
	if( _ui->_momentsBtn->isChecked() ){
		findMoments();
	}
}

void TactileSensorDialog::findMoments(){

	//void calcMoments2D(matrix<float>& mat, double low_thres){

	_moments.clear();
	const int low_thres = _ui->_lowBoundSpin->value();
	BOOST_FOREACH(Eigen::MatrixXf& mat, _values){
		Moment mom = calcMoments2D(mat, low_thres);
		_moments.push_back(mom);
		//std::cout << mom.center << mom.first << mom.second << std::endl;
	}
}


void TactileSensorDialog::detectCenterMass(){
	// we run through all sensors and for each texel we calculate the center of mass
	_centers.clear();
	const int low_thres = _ui->_lowBoundSpin->value();
	const int upp_thres = _ui->_uppBoundSpin->value();
	BOOST_FOREACH(Eigen::MatrixXf& mat, _values){
		Eigen::DenseIndex w = mat.rows();
        Eigen::DenseIndex h = mat.cols();

        double nrOfVals = 0;
        double totalValue = 0;
        double x=0,y=0;
        for(Eigen::DenseIndex i=0;i<w;i++){
            for(Eigen::DenseIndex j=0;j<h;j++){
            	if(low_thres<=mat(i,j) && mat(i,j)<=upp_thres){
            		nrOfVals++;
            		totalValue += mat(i,j);
            		x += i*mat(i,j);
            		y += j*mat(i,j);
            	}
            }
        }
        if(totalValue>0){
        	x /= totalValue;
        	y /= totalValue;
        } else {
        	x=-0.5; y=-0.5;
        }
        _centers.push_back(Vector2D<>(x,y));
	}
}

void TactileSensorDialog::zoomIn() { _ui->_gview->scale(1.2, 1.2); }
void TactileSensorDialog::zoomOut() { _ui->_gview->scale(1 / 1.2, 1 / 1.2); }
void TactileSensorDialog::rotateLeft() { _ui->_gview->rotate(-10); }
void TactileSensorDialog::rotateRight() { _ui->_gview->rotate(10); }
void TactileSensorDialog::wheelEvent(QWheelEvent* event)
{
   qreal factor = 1.2;
   if (event->delta() < 0)
     factor = 1.0 / factor;
   _ui->_gview->scale(factor, factor);
}

