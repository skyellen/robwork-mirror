/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef QTGUI_WORKCELLVIEWERWIDGET_HPP
#define QTGUI_WORKCELLVIEWERWIDGET_HPP

#ifdef __WIN32
#include <windows.h>
#endif

#include <QWidget>
#include <rw/graphics/SceneViewer.hpp>
#include <rw/graphics/SceneGraph.hpp>

namespace rws {

    /**
     * @brief Class representing an OpenGL based QT Widget for 3D visualization of Drawables
     */
    class SceneViewerWidget: public rw::graphics::SceneViewer {
    public:

        typedef rw::common::Ptr<SceneViewerWidget> Ptr;

        virtual rw::graphics::SceneGraph::RenderInfo& getRenderInfo() = 0;

        virtual QWidget* getWidget() = 0;

    };

}

#endif //#ifndef QTGUI_VIEWGL_HPP
