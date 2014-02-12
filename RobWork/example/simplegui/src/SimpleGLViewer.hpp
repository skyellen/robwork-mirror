#ifndef SIMPLEGLVIEWER_H_
#define SIMPLEGLVIEWER_H_

#include "Menu.hpp"
#include "MenuItem.hpp"
#include "EventListener.hpp"

#include <rw/rw.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/graphics/SceneViewer.hpp>
#include <rwlibs/opengl/SceneOpenGL.hpp>

/**
 * @brief simple viewer for rendering robwork workcell
 */
class SimpleGLViewer: public rw::graphics::SceneViewer {
    public:
        virtual ~SimpleGLViewer(){};

        void init(int argc, char** argv);

        void draw();

        void addMenu(Menu *menu);

        void setKeyListener(EventListener *listener);

        void setWorkcellModel(rw::models::WorkCell::Ptr workcellModel);

        void setState(const rw::kinematics::State& state);

        const rw::kinematics::State& getState();

        rw::graphics::WorkCellScene* getWorkCellGLDrawer();

        void resize(int width, int height);

        void setPosition(int x, int y);

        bool start();

        bool stop();
    private:

        rwlibs::opengl::SceneOpenGL::Ptr _scene;
        View::Ptr _currentView;
        rw::graphics::GroupNode::Ptr _worldNode;
        rw::graphics::SceneGraph::RenderInfo _renderInfo;

};


#endif /*SIMPLEGLVIEWER_H_*/
