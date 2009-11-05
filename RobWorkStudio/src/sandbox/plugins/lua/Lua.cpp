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

#include <rws/RobWorkStudio.hpp>

#include <sstream>

#include <rw/common/StringUtil.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

extern "C" {
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}

#include <rwlibs/lua/RobWork.hpp>
#include <rwlibs/lua/Output.hpp>
using namespace rwlibs::lua;

#include <sstream>

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

    class PluginOutput : public rwlibs::lua::Output
    {
    public:
        PluginOutput(QTextEdit* output) : _output(output) {}

        void write(const std::string& str)
        {
            append(_output, str);
        }

    private:
        QTextEdit* _output;
    };

    class PluginToLogOutput : public rwlibs::lua::Output
    {
    public:
    	PluginToLogOutput(rw::common::LogPtr log) : _output(log) {}

        void write(const std::string& str)
        {
        	_output->info() << str;
        }

    private:
        rw::common::LogPtr _output;
    };

}

Lua::Lua()
    :
    RobWorkStudioPlugin("Lua", QIcon(":/lua.png")),
    _editor(NULL)
{
    // Misc.
    QWidget *widget = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widget);
    widget->setLayout(lay);
    this->setWidget(widget);

    // Input label
    {
        QLabel* label = new QLabel("Input (Press Ctrl-J to evaluate current line):");
        lay->addWidget(label); // Own label.
    }

    // Input text widget.
    {
        _input = new QTextEdit();
        lay->addWidget(_input); // Own _input.
    }

    // Ctrl-J key binding
    {
        QShortcut* act = new QShortcut(
            QKeySequence("Ctrl+J"), _input, 0, 0, Qt::WindowShortcut);
        connect(act, SIGNAL(activated()), this, SLOT(runChunk()));
    }

    // Output label
    {
        QLabel* label = new QLabel("Output:");
        lay->addWidget(label); // Own label.
    }

    // Output text widget
    {
        _output = new QTextEdit();
        lay->addWidget(_output); // Own _output.
        _output->setReadOnly(true);
    }

    // Run button for loading a script
    {
        QPushButton* button = new QPushButton("Load Lua file");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(loadFile()));
    }

    // Open button for openning script editor
    {
        QPushButton* button = new QPushButton("Editor");
        lay->addWidget(button); // Own button.
        connect(button, SIGNAL(pressed()), this, SLOT(startEditor()));
    }

    // remove the native use of showAction
    _showAction.disconnect();
    connect(&_showAction, SIGNAL(triggered()), this, SLOT(startEditor()));

    _lua = 0;

}

Lua::~Lua()
{
    // Close the Lua state.
    lua_close(_lua);
}

void Lua::initialize()
{
    _lua = lua_open();
    luaL_openlibs(_lua);

    getRobWorkStudio()->stateChangedEvent().add(
        boost::bind(
            &Lua::stateChangedListener,
            this,
            _1), this);

    // register the lua state in the propertymap
    getRobWorkStudio()->getPropertyMap().add<lua_State*>(
            "LuaState",
            "A lua state handle",
            _lua );

}

void Lua::stateChangedListener(const State& state)
{
    _state = state;
    RobWork::setState(_lua, &_state);
}

void Lua::luaStateChangedListener(const State& state)
{
    // Alert RobWorkStudio that rw.setState() was called in the Lua script:
    getRobWorkStudio()->setState(state);
}

void Lua::luaPathChangedListener(const StatePath& path)
{
    // getRobWorkStudio()->setTimedStatePath(path);
}

void Lua::open(WorkCell* workcell)
{
    if (_lua) lua_close(_lua);

    // Open the Lua state.
    _lua = lua_open();
    luaL_openlibs(_lua);
    rwlibs::lua::RobWork::open(_lua);
    rwlibs::lua::RobWork::setOutput(_lua, new PluginToLogOutput( _log ));
    rwlibs::lua::RobWork::setCollisionDetector(_lua,
        getRobWorkStudio()->getCollisionDetector());
    RobWork::setWorkCell(_lua, workcell);

    RobWork::setStateChangedListener(
        boost::bind(
            &Lua::luaStateChangedListener,
            this,
            _1));

    stateChangedListener(getRobWorkStudio()->getState());
}

void Lua::close()
{}

void Lua::loadFile()
{
    const QString dir(_previousOpenDirectory.c_str());

    QString selectedFilter;
    const QString filename = QFileDialog::getOpenFileName(
        this,
        "Run Lua script",
        dir, // Directory
        "All supported ( *.lua *.luac )"
        " \n All ( *.* )",
        &selectedFilter);

    const std::string file = filename.toStdString();
    if (!file.empty()) {
        _previousOpenDirectory = StringUtil::getDirectoryName(file);
        append(_output, "--\n");
        const int error = luaL_dofile(_lua, file.c_str());
        processError(error, _lua, _output);
    }
}

void Lua::startEditor(){
	if(_editor==NULL){
		_editor = new LuaEditorWindow(_lua, RobWorkStudioPlugin::_log ,this);
	}

    if (_editor->isVisible()) {
    	_editor->setVisible(false);
    } else {
    	_editor->show();
    }

}

void Lua::runChunk()
{
    const std::string cmd =
        _input->textCursor().block().text().toStdString();

    append(_output, "--\n");
    const int error = // The string "" is part of the error message.
        luaL_loadbuffer(_lua, cmd.data(), cmd.size(), "") ||
        lua_pcall(_lua, 0, 0, 0);

    processError(error, _lua, _output);
}
/*
void Lua::setupToolBar(QToolBar* toolbar)
{
    /toolbar->addAction(&_showAction);
}
*/

//----------------------------------------------------------------------
#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Lua, Lua)
#endif
