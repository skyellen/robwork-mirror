/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef SUPPORTPOSEANALYSERDIALOG_HPP_
#define SUPPORTPOSEANALYSERDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_SupportPoseAnalyserDialog.h"

#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/ThreadSimulator.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/Path.hpp>
#include <util/MovingAverage.hpp>
#include <util/SupportPose.hpp>
#include <util/CircleModel.hpp>
#include <drawable/RenderPoints.hpp>
#include <drawable/RenderPlanes.hpp>
#include <drawable/RenderCircles.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

#include "RestingPoseDialog.hpp"
#include "GLViewRW.hpp"

/**
 * @brief gaphical user interface for calculating support pose and related
 * statistics of multiple objects on some support structure.
 */
class SupportPoseAnalyserDialog : public QDialog, private Ui::SupportPoseAnalyserDialog
    {
        Q_OBJECT

    public:
    	/**
    	 * @brief
    	 */
        SupportPoseAnalyserDialog(const rw::kinematics::State& state,
                          dynamics::DynamicWorkcell *dwc,
                          rw::proximity::CollisionDetector *detector,
                          QWidget *parent = 0);


        virtual ~SupportPoseAnalyserDialog(){};

        void initializeStart();


    signals:
        void stateChanged(const rw::kinematics::State& state);

    private slots:
        void btnPressed();
        void changedEvent();
        void addRestingPose(const rw::kinematics::State& startstate,
							const rw::kinematics::State& reststate);

    private:
        void updateStatus();
        void addStatePath(rw::trajectory::TimedStatePathPtr path);
        void addStateStartPath(rw::trajectory::TimedStatePathPtr path);
        void process();
        void updateRenderView();
        void updateResultView();

        void updateHoughThres(){
        	// we set the thres hold such that [30,50] maps to [100,1000]
        	int samples = _xaxis[0].size();
        	double a = ((50.0-30.0)/(1000.0-100.0));
        	double b = 30-100.0*a;
        	double thres = a*samples+b;
        	thres = rw::math::Math::clamp(thres,20.0,250.0);
        	_thresholdSpin->setValue((int)thres);
        }

        dynamics::RigidBody* getSelectedBody(){
        	int i = _selectObjBox->currentIndex();
        	return _bodies[i];
        }



    private:
        Ui::SupportPoseAnalyserDialog _ui;
        std::string _previousOpenDirectory;

        rw::kinematics::State _defaultState;
        rw::models::WorkCell *_wc;
        dynamics::DynamicWorkcell *_dwc;
        rw::proximity::CollisionDetector *_detector;

        rw::trajectory::TimedStatePathPtr _path, _startPath;
        GLViewRW *_view;
        rw::common::Ptr<RenderPoints> _xRender,_yRender,_zRender;
        rwlibs::drawable::Drawable *_xDraw,*_yDraw,*_zDraw;

        rw::common::Ptr<RenderPoints> _selPosePntRenderX,_selPosePntRenderY,_selPosePntRenderZ;
        rwlibs::drawable::Drawable *_selPoseDrawX,*_selPoseDrawY,*_selPoseDrawZ;

        rw::common::Ptr<RenderPoints> _selxRender,_selyRender,_selzRender;
        rwlibs::drawable::Drawable *_selxDraw,*_selyDraw,*_selzDraw;

        rw::common::Ptr<RenderCircles> _xcRender,_ycRender,_zcRender;
        rwlibs::drawable::Drawable *_xcDraw,*_ycDraw,*_zcDraw;

        std::vector<dynamics::RigidBody*> _bodies;
        std::vector< std::vector<rw::math::Vector3D<> > > _xaxis,_yaxis,_zaxis;
        std::map<dynamics::RigidBody*,std::vector<rw::math::Vector3D<> > >
			_xaxisS,_yaxisS,_zaxisS;

        std::map<dynamics::RigidBody*,std::vector<SupportPose> > _supportPoses;
        std::map<dynamics::RigidBody*,std::vector<CircleModel> > _xcircBodyMap, _ycircBodyMap, _zcircBodyMap;
        std::map<SupportPose*,std::vector<CircleModel> > _xcirclesMap, _ycirclesMap, _zcirclesMap;
        std::map<std::pair<int,int>,std::vector<int> > _supportToPose;

        RestingPoseDialog *_restPoseDialog;

        //rw::common::sandbox::LogPtr _log;
};


#endif /* SupportPoseAnalyserDialog_HPP_ */
