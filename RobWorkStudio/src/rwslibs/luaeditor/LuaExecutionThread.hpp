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

#ifndef RWS_LUAEXECUTIONTHREAD_HPP_
#define RWS_LUAEXECUTIONTHREAD_HPP_

#include <QMainWindow>
#include <QModelIndex>

#include <rw/common/Log.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rwlibs/swig/lua/LuaState.hpp>
#include "LuaHighlighter.hpp"
#include "CodeEditor.hpp"

#include <QMainWindow>
#include <QThread>

class TreeModelCompleter;
class QAbstractItemModel;
class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QCheckBox;
class QTreeView;

class QTextEdit;

namespace rws {

/**
 * @brief this class handles the execution of lua commands such that they
 * are executed in a seperate thread.
 */
class LuaExecutionThread: public QThread
{
    Q_OBJECT
public:
    /**
     * @brief constructor
     * @param cmd
     * @param lstate
     * @param output
     * @param parent
     */
    LuaExecutionThread(const std::string& cmd, rwlibs::swig::LuaState::Ptr lstate, rw::common::LogWriter::Ptr output, QWidget *parent =
                               NULL) :
            QThread(parent), _cmd(cmd), _lua(lstate), _output(output)
    {
    }

    /**
     * @brief sets the command to execute, the lua state on which to execute and
     * the log in which output should be written.
     * @param cmd [in] a valid Lua block
     * @param lstate [in] the current lua state
     * @param output [in] the log in which to print result
     */
    void set(const std::string& cmd, rwlibs::swig::LuaState::Ptr lstate, rw::common::LogWriter::Ptr output);

    //! @brief executes the command
    void run();

    //! @brief stop a running script
    void stop();

    /**
     * @brief on error this returns the error message reported by the lua interpreter
     * @return
     */
    std::string getReturnString()
    {
        return _resstring;
    }
    ;

    /**
     * @brief returns the value which lua reports upon completion of executing
     * the lua command string.
     */
    int getReturnValue()
    {
        return _resVal;
    }
    ;

    /**
     * @brief return the command that was executed.
     * @return command string that was executed.
     */
    std::string getCommand()
    {
        return _cmd;
    }
    ;

private:
    std::string _cmd, _resstring;
    int _resVal;
    rwlibs::swig::LuaState::Ptr _lua;
    rw::common::LogWriter::Ptr _output;
};

} // namespace

#endif /* LUAEDITORWINDOW_HPP_ */
