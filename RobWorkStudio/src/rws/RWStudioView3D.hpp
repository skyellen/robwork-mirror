/************************************************************************
 * RobWorkStudio Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#ifndef RWStudioView3D_HPP
#define RWStudioView3D_HPP

#ifdef __WIN32
#include <windows.h>
#endif

#include <vector>

#include <QObject>
#include <QString>

#include <QMouseEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QImage>
#include <QAction>
#include <QToolBar>
#include <QMenu>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/graphics/DrawableGeometryNode.hpp>
#include <rw/graphics/SceneViewer.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rwlibs/opengl/RenderCameraFrustum.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/graphics/SceneViewer.hpp>
#include "SceneViewerWidget.hpp"

namespace rws {
class RobWorkStudio;

/**
 * @brief This class adds RobWorkStudio specific functionality to a WorkCellViewer.
 *
 * specifically:
 * * optional collision detection and highlighting of frames
 * *
 */
class RWStudioView3D: public QWidget {
    Q_OBJECT

public:

    typedef rw::common::Ptr<RWStudioView3D> Ptr;

    /**
     * @brief Constructs an OpenGL based QT Widget
     * @param rwStudio [in] robworkstudio
     * @param parent [in] Parent widget
     */
    RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent);

    /**
     * @brief destructor
     */
    virtual ~RWStudioView3D();

    /**
     * @brief
     * @param visible
     */
    void setSceneViewerWidget(SceneViewerWidget* viewer);

    /**
     * @brief Specified whether to visualize the pivot point
     * @param visible [in] True for showing pivot point, false otherwise.
     */
    void showPivotPoint(bool visible);

    /**
     * @brief Sets whether to visualize as SOLID, WIRE or BOTH.
     * @param drawType [in] The drawtype to use
     */
    void setDrawType(rw::graphics::DrawableNode::DrawType drawType);

    /**
     * @brief sets up the
     * @param mainwindow
     */
    void setupGUI(QMainWindow* mainwindow);

    /**
     * @brief picks the frame that has drawables that intersect the ray cast into the screen from
     * the screen coordinates \b x and \b y.
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @return the frame that was selected, Null if no frames where selected.
     */
    rw::kinematics::Frame* pickFrame(int x, int y);

    /**
     * @brief picks a drawable in the scene.
     * @param x
     * @param y
     * @return
     */
    rw::graphics::DrawableNode::Ptr pick(int x, int y);

    //! @brief updates the 3d view
    void update(){
        _view->updateView();
    };

    /**
     * @brief set the state of the view
     * @param state [in] new state to be rendered
     */
    void setState(const rw::kinematics::State& state);

    /**
     * @brief get propertymap
     * @return propertymap
     */
    rw::common::PropertyMap& getPropertyMap(){ return _pmap->getValue(); }



    rw::graphics::WorkCellScene::Ptr getWorkCellScene(){ return _wcscene;};

    rw::graphics::SceneViewer::Ptr getSceneViewer(){ return _view; }



    void clear();

    void setWorkCell(rw::models::WorkCell::Ptr workcell);

    rw::models::WorkCell::Ptr  getWorkCell(){ return _wc; };

    void saveBufferToFile(const QString& filename){
        _view->saveBufferToFile(filename.toStdString());
    }

    //// events inherited from QtWidget
    void keyPressEvent(QKeyEvent *e);
    void mouseDoubleClickEvent(QMouseEvent* event);

private slots:
    void setDrawTypeSlot();
    void setTransparentSlot();
    void showPivotPointSlot();
    void setCheckForCollision(bool);
    void setCheckAction();
    // Save buffer dialog.
    void saveBufferToFileDialog();

private:
    //! struct for keeping track of the sensor camera view frustrums
    struct SensorCameraView {
        SensorCameraView(double fy, double w, double h, double n, double f, rw::kinematics::Frame* fra):
            _fovy(fy),
            _width(w),
            _height(h),
            _near(n),
            _far(f),
            _frame(fra),
            _action(NULL)
        {};
        rw::graphics::SceneViewer::View::Ptr _view;
        double _fovy, _width, _height, _near, _far;
        rw::kinematics::Frame *_frame;
        QAction* _action;
    };


    void setupActions();

    virtual void setupToolBarAndMenu(QMainWindow *mwindow);
    SensorCameraView makeCameraView(const std::string& name,double fovy, double w, double h, double n, double f, rw::kinematics::Frame* frame);
protected:

    void contextMenuEvent ( QContextMenuEvent * event );

protected:
    rw::graphics::SceneViewer* _view;
    rw::graphics::WorkCellScene::Ptr _wcscene;
    rw::models::WorkCell::Ptr _wc;
    RobWorkStudio *_rws;

    rw::graphics::DrawableGeometryNode::Ptr _floorDrawable;

    rw::common::Property<rw::common::PropertyMap>::Ptr _pmap;
    std::string _viewLogo;
    QAction *_showSolidAction;
    QAction *_showWireAction;
    QAction *_showOutlineAction;
    QAction *_showTransparentAction;
    QAction *_showPivotPointAction;
    QAction *_checkForCollision;
    QAction *_saveBufferToFileAction;

    QAction *_axometricViewAction, *_frontViewAction, *_rightViewAction, *_topViewAction;
    QAction *_rearViewAction, *_leftViewAction, *_bottomViewAction;

    QAction *_addViewAction, *_clearViewAction;

    QAction *_addCameraViewAction, *_clearCameraViewsAction, *_selectMainViewAction;

    QAction *_setPerspectiveViewAction, *_setOrthographicViewAction;
    QMenu *_customViewMenu, *_cameraViewMenu;

    std::vector<std::pair<QAction*,rw::math::Transform3D<> > > _customViews;
    rw::proximity::CollisionDetector::QueryResult _qryResult;

    std::vector<std::pair<SensorCameraView, rwlibs::opengl::RenderCameraFrustumPtr> > _sensorCameraViews;
};
}

#endif //#ifndef QTGUI_RWStudioView3DRW_HPP
