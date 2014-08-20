/*
 * TactileSensorDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef TACTILESENSORDIALOG_HPP_
#define TACTILESENSORDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <QDialog>
#include <QGraphicsScene>

#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/math.hpp>
#include <rwsim/util/MovingAverage.hpp>

#include <rw/proximity/CollisionDetector.hpp>

struct Moment {
    rw::math::Vector2D<> center;
    rw::math::Vector2D<> first,second;
};

namespace Ui {
    class TactileSensorDialog;
}

/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class TactileSensorDialog : public QDialog
    {
        Q_OBJECT

    public:
        TactileSensorDialog(
                          rwsim::dynamics::DynamicWorkCell *dwc,
                          QWidget *parent = 0);

        void drawTactileInput();

        void detectFeatures();

    signals:


    public slots:
        void zoomIn();
        void zoomOut();
        void rotateLeft();
        void rotateRight();
        void wheelEvent(QWheelEvent* event);

    private slots:
        void btnPressed();
        void changedEvent();
        void setState(const rw::kinematics::State& state);


    private:
        void initTactileInput();
        void detectCenterMass();
        void findMoments();
    private:
        Ui::TactileSensorDialog *_ui;

        rwsim::dynamics::DynamicWorkCell *_dwc;
        std::vector<rwsim::sensor::TactileArraySensor*> _tsensors;
        std::vector<Eigen::MatrixXf> _values;
        QGraphicsScene *_scene;
        bool _renderingToImage;

        std::vector< std::vector<QGraphicsRectItem*> > _rectItems;
        std::vector<QGraphicsEllipseItem*> _centerItems;
        std::vector<QGraphicsLineItem*> _momentItems;
        std::vector<QGraphicsLineItem*> _momentSecItems;
        std::vector<std::pair<int,int> > _dims;
        int _saveCnt;


        //typedef std::pair<rw::math::Vector2D<>,rw::math::Vector2D<> > Moment;


        std::vector< rw::math::Vector2D<> > _centers;
        std::vector< Moment > _moments;
        int _nrOfPadsH;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
