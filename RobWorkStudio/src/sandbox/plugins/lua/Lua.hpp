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

#ifndef RW_STUDIO_LUA_MODULE_H
#define RW_STUDIO_LUA_MODULE_H

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/common/Message.hpp>
#include <rw/trajectory/Path.hpp>

#include <sandbox/components/luaeditor/LuaEditorWindow.hpp>

struct lua_State;

/**
 * @brief Initial Lua plugig for RobWorkStudio. This class is marked for removal - please use rws::Lua instead!
 * @deprecated This class is marked for removal.
 */
class Lua : public RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES(RobWorkStudioPlugin)

public:
    //! @brief Constructor.
    Lua();

    //! @brief Destructor.
    ~Lua();

    //! @copydoc RobWorkStudioPlugin::initialize
    void initialize();

    //! @copydoc RobWorkStudioPlugin::open
    void open(rw::models::WorkCell* workcell);

    //! @copydoc RobWorkStudioPlugin::close
    void close();

    //void setupToolBar(QToolBar* toolbar);

private:
    // This listens for changes to the state of RobWorkStudio.
    void stateChangedListener(const rw::kinematics::State& state);

    // This listens for changes to the state of the Lua interpreter.
    void luaStateChangedListener(const rw::kinematics::State& state);

    // This listens for changes to the path of the Lua interpreter.
    void luaPathChangedListener(const rw::trajectory::StatePath& path);

private slots:
    void runChunk();
    void loadFile();
    void startEditor();

private:
    QTextEdit* _input;
    QTextEdit* _output;

    lua_State* _lua;
    rw::kinematics::State _state;
    std::string _previousOpenDirectory;

    LuaEditorWindow *_editor;
};

#endif
