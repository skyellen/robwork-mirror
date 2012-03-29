#ifndef SIMPLEGLVIEWER_H_
#define SIMPLEGLVIEWER_H_

#include "Menu.hpp"
#include "MenuItem.hpp"
#include "EventListener.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/graphics/WorkCellScene.hpp>

class SimpleGLViewer {
    public:
        virtual ~SimpleGLViewer(){};

        static void init(int argc, char** argv);

        static void addMenu(Menu *menu);

        static void setKeyListener(EventListener *listener);

        static void setWorkcellModel(rw::models::WorkCell::Ptr workcellModel);

        static void setState(const rw::kinematics::State& state);

        static const rw::kinematics::State& getState();

        static rw::graphics::WorkCellScene* getWorkCellGLDrawer();

        static void resize(int width, int height);

        static void setPosition(int x, int y);

        static bool start();

        static bool stop();
};


#endif /*SIMPLEGLVIEWER_H_*/
