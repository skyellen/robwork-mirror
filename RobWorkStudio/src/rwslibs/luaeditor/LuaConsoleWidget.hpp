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

#ifndef RWS_LUACONSOLEWIDGET_HPP_
#define RWS_LUACONSOLEWIDGET_HPP_

#include <qstringlist.h>
#include <QTextEdit>
#include <QObject>

#include <rw/common/LogWriter.hpp>
#include <rwlibs/swig/lua/LuaState.hpp>
#include "LuaExecutionThread.hpp"

namespace rws {

    /**
     * @brief a widget that mimics console functionality for executing lua commands
     * in a RobWorkStudio framework
     */
    class LuaConsoleWidget : public QTextEdit {
        Q_OBJECT
    public:
        /**
         * @brief constructor
         * @param parent [in] the Qt widget parent
         */
        LuaConsoleWidget(QWidget *parent = 0);

        //! @brief destructor
        virtual ~LuaConsoleWidget(){}

        //void setCompleter(QCompleter *c);
        //QCompleter *completer() const;

        void reset();

        void setLuaState(rwlibs::swig::LuaState *lstate);

        bool event(QEvent *event);
    protected:
        //void resizeEvent(QResizeEvent *event);

        void keyPressEvent(QKeyEvent *e);

        virtual QString interpretCommand(QString str, int *res);

        //void focusInEvent(QFocusEvent *e);

    private slots:
        //Correctly handle the cursor when moved
        void moveCursor(QTextCursor::MoveOperation action, QTextCursor::MoveMode mode = QTextCursor::MoveAnchor);
        void runFinished();


    private:
        bool isInEditionZone();

        QString getCurrentCommand();
        void displayPrompt();
        bool isCommandComplete(QString command);
        bool execCommand(QString command, bool b);
        void setCommand(QString command);

        QColor _cmdColor, _errColor, _outColor, _completionColor;

        // cached prompt length
        int _promptLen;
        // The prompt string
        QString _promptStr;
        QStringList _commandHistory;
        int _histIdx;
        int _lastBlockNumber;

        rw::common::LogWriter::Ptr _logWriter;

        rws::LuaExecutionThread *_luaRunner;
        rwlibs::swig::LuaState *_luastate;
    };
}

#endif /* LUACONSOLE_HPP_ */
