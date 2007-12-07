#ifndef SIMPLEGLVIEWER_H_
#define SIMPLEGLVIEWER_H_

#include "Menu.hpp"
#include "MenuItem.hpp"
#include "EventListener.hpp"

#include <core/models/WorkCell.hpp>
#include <core/drawable/WorkCellGLDrawer.hpp>

class SimpleGLViewer {
    public:
        virtual ~SimpleGLViewer(){};

        static void init(int argc, char** argv);

        static void addMenu(Menu *menu);

        static void setKeyListener(EventListener *listener);

        static void setWorkcellModel(rw::core::models::WorkCell *workcellModel);

        static rw::core::drawable::WorkCellGLDrawer* getWorkCellGLDrawer();

        static void resize(int width, int height);

        static void setPosition(int x, int y);

        static bool start();

        static bool stop();
};


#endif /*SIMPLEGLVIEWER_H_*/
