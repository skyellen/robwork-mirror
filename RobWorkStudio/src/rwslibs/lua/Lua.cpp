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

#include "Lua.hpp"

#include <QVBoxLayout>

#include <sstream>
#include <rw/common/StringUtil.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>

#include <rws/RobWorkStudio.hpp>
#include <rwslibs/swig/ScriptTypes.hpp>
#include <rwslibs/swig/lua/LuaState.hpp>

extern "C" {
    #include <lua.h>
    #include <lualib.h>
    #include <lauxlib.h>
}

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;
using namespace rw::trajectory;
using namespace rwlibs;
using namespace rws;

//----------------------------------------------------------------------
// Standard plugin methods

namespace
{
    void append(QTextEdit* output, const std::string& str)
    {
        output->moveCursor(QTextCursor::End);
        output->textCursor().insertText(QString(str.c_str()));
    }

    void processError(int error, lua_State* lua, QTextEdit* output)
    {
        if (error) {
            append(output, lua_tostring(lua, -1));
            append(output, "\n");
            lua_pop(lua, 1);
        }
    }

}

Lua::Lua()
    :
    RobWorkStudioPlugin("LuaConsole", QIcon(":/lua.png")),
    _editor(NULL)
{
    // Misc.
    // remove the native use of showAction
    QWidget *widget = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widget);
    widget->setLayout(lay);
    this->setWidget(widget);

    QToolBar* toolbar = new QToolBar();
    lay->addWidget(toolbar); // own toolbar
    toolbar->setIconSize(QSize(12,12));
    lay->setAlignment(toolbar, Qt::AlignTop);

    QAction* editorAction = new QAction(QIcon(":/images/collapse_all.png"), "Editor", this); // owned
    connect(editorAction , SIGNAL(triggered()), this, SLOT(startEditor()));

    //QAction* expandAllAction = new QAction(QIcon(":/images/reload.png"), "Clr", this); // owned
    //connect(expandAllAction, SIGNAL(triggered()), this, SLOT(expandAll()));

    QAction *resetAction = new QAction(QIcon(":/images/reload.png"), "Reset Lua", this); // owned
    connect(resetAction, SIGNAL(triggered()), this, SLOT(resetLua()));

    toolbar->addAction(editorAction);
    toolbar->addAction(resetAction);

    _console = new LuaConsoleWidget( this );
    //_console->setReadOnly(true);
    lay->addWidget(_console);

    //widget->setLayout( vlay );

    //this->setWidget(vwidget);  // Sets the widget on the QDockWidget

    _lua = rw::common::ownedPtr( new LuaState() );
    _lua->setRobWorkStudio( getRobWorkStudio() );
    _lua->reset();
}

Lua::~Lua()
{
    // Close the Lua state.
    //delete _lua;
}

void Lua::initialize()
{
    _lua->setRobWorkStudio( getRobWorkStudio() );
    _lua->reset();
    _console->setLuaState(_lua.get());

    getRobWorkStudio()->stateChangedEvent().add(
        boost::bind(
            &Lua::stateChangedListener,
            this,
            _1), this);

    // register the lua state in the propertymap
    getRobWorkStudio()->getPropertyMap().add<LuaState::Ptr>(
            "LuaState",
            "A lua state handle",
            _lua );
}

void Lua::stateChangedListener(const State& state)
{
    _state = state;
}

void Lua::luaStateChangedListener(const State& state)
{
    // Alert RobWorkStudio that rw.setState() was called in the Lua script:
    getRobWorkStudio()->setState(state);
}

void Lua::luaPathChangedListener(const StatePath& path)
{

}

void Lua::open(WorkCell* workcell)
{
    _lua->setRobWorkStudio( getRobWorkStudio() );
    _lua->reset();
    stateChangedListener(getRobWorkStudio()->getState());
}

void Lua::resetLua(){
    _lua->setRobWorkStudio( getRobWorkStudio() );
    _lua->reset();

    if(_editor!=NULL){
        _editor->setLuaState( _lua );
    }
}

void Lua::close()
{}


void Lua::startEditor(){

	if(_editor==NULL){
		_editor = new LuaEditorWindow(_lua, RobWorkStudioPlugin::_log , getRobWorkStudio(), this);
	}

    if (_editor->isVisible()) {
    	_editor->setVisible(false);
    } else {
    	_editor->show();
    }

}



void Lua::setupMenu(QMenu* pluginmenu){
    QMenuBar *menu = getRobWorkStudio()->menuBar();

    _openEditorAction = new QAction(tr("Lua Teachpad"), this); // owned
    connect(_openEditorAction, SIGNAL(triggered()), this, SLOT(startEditor()));




    boost::tuple<QWidget*, QMenu*, int> action = RobWorkStudioPlugin::getMenu((QWidget*)menu, std::string("&Tools") );
    if(action.get<1>()!=NULL){
        action.get<1>()->addAction( _openEditorAction );
    } else {

    }

}



//----------------------------------------------------------------------
#ifndef RW_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
Q_EXPORT_PLUGIN2(Lua, Lua)
#endif
#endif
