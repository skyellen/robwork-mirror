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

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <QGraphicsPixmapItem>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/math/Math.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <rwsim/util/SupportPose.hpp>
#include <rwsim/util/CircleModel.hpp>
#include <rwsim/drawable/RenderPoints.hpp>
#include <rwsim/drawable/RenderPlanes.hpp>
#include <rwsim/drawable/RenderCircles.hpp>
#include <rw/math/Pose6D.hpp>
#include <rwlibs/opengl/RenderFrame.hpp>

#include "RestingPoseDialog.hpp"
#include "GLViewRW.hpp"

namespace Ui {
    class SupportPoseAnalyserDialog;
}

/**
 * @brief gaphical user interface for calculating support pose and related
 * statistics of multiple objects on some support structure.
 */
class SupportPoseAnalyserDialog : public QDialog
    {
        Q_OBJECT

    public:
    	/**
    	 * @brief
    	 */
        SupportPoseAnalyserDialog(const rw::kinematics::State& state,
                          rwsim::dynamics::DynamicWorkCell *dwc,
                          rw::proximity::CollisionDetector *detector,
                          rws::RobWorkStudio *_rwstudio,
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
        void addStatePath(rw::trajectory::TimedStatePath::Ptr path);
        void addStateStartPath(rw::trajectory::TimedStatePath::Ptr path);
        void process();
        void updateRenderView();
        void updateResultView();

        void showPlanarDistribution();
        void saveDistribution();

        void updateHoughThres();

        rwsim::dynamics::RigidBody::Ptr getSelectedBody();



    private:
        Ui::SupportPoseAnalyserDialog *_ui;
        std::string _previousOpenDirectory;

        rw::kinematics::State _defaultState;
        rw::models::WorkCell *_wc;
        rwsim::dynamics::DynamicWorkCell *_dwc;
        rw::proximity::CollisionDetector *_detector;

        rw::trajectory::TimedStatePath::Ptr _path, _startPath;
        GLViewRW *_view;

        rw::common::Ptr<rwlibs::opengl::RenderFrame> _frameRender;
        rwlibs::opengl::Drawable *_fDraw,*_fDraw1,*_fDraw2,*_fDraw3,*_fDraw4,*_fDraw5;

        rw::common::Ptr<rwsim::drawable::RenderPoints> _xRender,_yRender,_zRender;
        rwlibs::opengl::Drawable *_xDraw,*_yDraw,*_zDraw;

        rw::common::Ptr<rwsim::drawable::RenderPoints> _selPosePntRenderX,_selPosePntRenderY,_selPosePntRenderZ;
        rwlibs::opengl::Drawable *_selPoseDrawX,*_selPoseDrawY,*_selPoseDrawZ;

        rw::common::Ptr<rwsim::drawable::RenderPoints> _selxRender,_selyRender,_selzRender;
        rwlibs::opengl::Drawable *_selxDraw,*_selyDraw,*_selzDraw;

        rw::common::Ptr<rwsim::drawable::RenderCircles> _xcRender,_ycRender,_zcRender;
        rwlibs::opengl::Drawable *_xcDraw,*_ycDraw,*_zcDraw;

        std::vector<rwsim::dynamics::RigidBody::Ptr> _bodies;
        std::vector< std::vector<rw::math::Vector3D<> > > _xaxis,_yaxis,_zaxis;
        std::vector< std::vector<rw::math::Transform3D<> > > _endTransforms, _startTransforms;


        std::map<rwsim::dynamics::RigidBody*,std::vector<rw::math::Vector3D<> > >
			_xaxisS,_yaxisS,_zaxisS;

        std::map<rwsim::dynamics::RigidBody*,std::vector<rwsim::util::SupportPose> > _supportPoses;
        std::map<rwsim::dynamics::RigidBody*,std::vector<rwsim::util::CircleModel> > _xcircBodyMap, _ycircBodyMap, _zcircBodyMap;
        std::map<rwsim::util::SupportPose*,std::vector<rwsim::util::CircleModel> > _xcirclesMap, _ycirclesMap, _zcirclesMap;
        std::map<std::pair<int,int>,std::vector<int> > _supportToPose;

        typedef std::vector<rw::math::Transform3D<> > PoseDistribution;
        std::map<rwsim::dynamics::RigidBody*,std::vector<PoseDistribution> > _supportPoseDistributions,_supportPoseDistributionsMisses;

        QGraphicsPixmapItem *_pitem;
        RestingPoseDialog *_restPoseDialog;

        rws::RobWorkStudio *_rwstudio;

        //rw::common::sandbox::LogPtr _log;
};


#endif /* SupportPoseAnalyserDialog_HPP_ */
