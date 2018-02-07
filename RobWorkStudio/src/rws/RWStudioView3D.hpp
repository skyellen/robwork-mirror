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
#include <QWidget>

#include <rw/proximity/CollisionDetector.hpp>
#include <rw/graphics/SceneViewer.hpp>
#include <rw/graphics/WorkCellScene.hpp>

namespace rw { namespace graphics { class DrawableGeometryNode; } }
namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace opengl { class RenderCameraFrustum; } }

class QAction;
class QMainWindow;
class QMenu;
class QMouseEvent;
class QString;

namespace rws {
class RobWorkStudio;
class SceneViewerWidget;

    //! @addtogroup rws
    //! @{ 
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
	//! @brief Smart pointer type for RWStudioView3D.
    typedef rw::common::Ptr<RWStudioView3D> Ptr;

    /**
     * @brief Constructs an OpenGL based QT Widget
     * @param rwStudio [in] robworkstudio
     * @param parent [in] Parent widget
     */
    RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent);

    //! @brief Destructor.
    virtual ~RWStudioView3D();

    /**
     * @brief Set the widget used for visualization of the scene.
     * @param viewer [in] the widget for visualization of the scene.
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
     * @param x [in] x coordinate
     * @param y [in] y coordinate
     * @return the drawable that was selected, Null if no frames where selected.
     */
    rw::graphics::DrawableNode::Ptr pick(int x, int y);

    //! @brief updates the 3d view
    void update(){
        _view->updateView();
    }

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

    /**
     * @brief Get the scene.
     * @return the scene.
     */
    rw::graphics::WorkCellScene::Ptr getWorkCellScene(){ return _wcscene;}

    /**
     * @brief Get the scene viewer.
     * @return the scene viewer.
     */
    rw::graphics::SceneViewer::Ptr getSceneViewer(){ return _view; }

    /**
     * @brief Set the workcell.
     * @param workcell [in] the workcell.
     */
    void setWorkCell(rw::common::Ptr<rw::models::WorkCell> workcell);

    //! @brief Clear the view.
    void clear();

    //! @copydoc SceneOpenGLViewer::saveBufferToFile
    void saveBufferToFile(const QString& filename, const int fillR=0, const int fillG=0, const int fillB=0){
        _view->saveBufferToFile(filename.toStdString(), fillR, fillG, fillB);
    }

    //// events inherited from QtWidget

    /**
     * @brief Handle key press events.
     *
     * The following is supported:
     *
     * | Keys       | Function                                          |
     * |------------|---------------------------------------------------|
     * | CTRL+G     | Save current view to file (file dialog is opened) |
     * | CTRL+Left  | Switch to previous scene camera                   |
     * | CTRL+Right | Switch to next scene camera                       |
     * | CTRL+1...9 | Switch to scene camera number n                   |
     *
     * @param e [in] the event.
     */
    void keyPressEvent(QKeyEvent *e);

    /**
     * @brief Handle double mouse click events.
     *
     * If CTRL key is pressed while double clicking the left mouse button,
     * a frame can be selected.
     *
     * @param event [in] the event.
     */
    void mouseDoubleClickEvent(QMouseEvent* event);

    //! get current draw mask
    int getDrawMask();

private slots:
    void setDrawTypeSlot();
    void setTransparentSlot();
    void showPivotPointSlot();
    void zoomInSlot();
    void zoomOutSlot();
    void zoomAutoSlot();
    void setCheckForCollision(bool);
    void setCheckAction();
    void setMaskCheckAction();
    // Save buffer dialog.
    void saveBufferToFileDialog();

    void ShowContextMenu(const QPoint& pos);

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
        {}
        rw::graphics::SceneViewer::View::Ptr _view;
        double _fovy, _width, _height, _near, _far;
        rw::kinematics::Frame *_frame;
        QAction* _action;
    };


    void setupActions();

    virtual void setupToolBarAndMenu(QMainWindow *mwindow);

    void resetCameraViewMenu();

    SensorCameraView makeCameraView(const std::string& name,double fovy, double w, double h, double n, double f, rw::kinematics::Frame* frame);

//protected:
    //void contextMenuEvent ( QContextMenuEvent * event );

private:
    rws::SceneViewerWidget* _viewWidget;
    rw::graphics::SceneViewer* _view;
    rw::graphics::WorkCellScene::Ptr _wcscene;
    rw::common::Ptr<rw::models::WorkCell> _wc;
    RobWorkStudio *_rws;

    rw::common::Ptr<rw::graphics::DrawableGeometryNode> _floorDrawable;

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

    QAction *_physicalMaskEnabled,
            *_virtualMaskEnabled,
            *_drawableMaskEnabled,
            *_collisionMaskEnabled,
            *_user1MaskEnabled,
            *_user2MaskEnabled,
            *_user3MaskEnabled,
            *_user4MaskEnabled;

    QAction *_zoomInAction, *_zoomOutAction, *_zoomAutoAction;


    QMenu *_customViewMenu, *_cameraViewMenu, *_drawMaskMenu;

    std::vector<std::pair<QAction*,rw::math::Transform3D<> > > _customViews;
    rw::proximity::CollisionDetector::QueryResult _qryResult;

    std::vector<std::pair<SensorCameraView, rw::common::Ptr<rwlibs::opengl::RenderCameraFrustum> > > _sensorCameraViews;
};

//! @}
}

#endif //#ifndef QTGUI_RWStudioView3DRW_HPP
