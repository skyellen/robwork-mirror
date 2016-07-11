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

#include "LuaConsoleWidget.hpp"
#include "LuaExecutionThread.hpp"

#include <sstream>
#include <iomanip>

extern "C" {
    #include <lua.h>
    #include <lualib.h>
    #include <lauxlib.h>
}

#include <rw/common/LogWriter.hpp>
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rwlibs/swig/lua/LuaState.hpp>

#include <QApplication>
#include <QKeyEvent>
#include <QScrollBar>

using namespace rw::common;
using namespace rws;

#define MESSAGE_ADDED_EVENT 2345

namespace {

    class WriterWrapper: public LogWriter {
    public:
        WriterWrapper(QTextEdit* slog):
            _slog(slog),_tabLevel(0)
        {
        }

        virtual ~WriterWrapper(){}

        virtual void flush(){
        }

        /**
         * @brief Writes \b str to the log
         * @param str [in] message to write
         */
        virtual void write(const std::string& str) {
            std::stringstream sstr;
            sstr << std::setw(_tabLevel)<<std::setfill(' ');
            sstr << str;

            _msgQueue.push_back(std::make_pair(sstr.str(), _color));
            QApplication::postEvent( _slog, new QEvent((QEvent::Type)MESSAGE_ADDED_EVENT) );
        }

        virtual void writeln(const std::string& str){
            write(str);
        }

        virtual void setTabLevel(int tabLevel) {
            _tabLevel = tabLevel;
        }

    public:
        std::vector< std::pair<std::string, QColor> > _msgQueue;

    private:
        QTextEdit *_slog;
        QColor _color;
        int _tabLevel;
    };


}

LuaConsoleWidget::LuaConsoleWidget(QWidget *parent) :
     QTextEdit(parent),
     _cmdColor(Qt::black), _errColor(Qt::red), _outColor(Qt::blue), _completionColor(Qt::green),
     _promptLen(1), _promptStr(">"), _histIdx(0), _lastBlockNumber(0), _luastate(NULL)
 {
    _logWriter = ownedPtr( new WriterWrapper(this) );
     //connect(this, SIGNAL(blockCountChanged(int)), this, SLOT(updateLineNumberAreaWidth(int)));
     //connect(this, SIGNAL(updateRequest(const QRect &, int)), this, SLOT(updateLineNumberArea(const QRect &, int)));
     //connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(highlightCurrentLine()));
    _luaRunner =  new LuaExecutionThread("", _luastate, _logWriter, this);
    connect(_luaRunner, SIGNAL(finished()), this, SLOT(runFinished()));
     //updateLineNumberAreaWidth(0);
     //highlightCurrentLine();
    reset();
 }

void LuaConsoleWidget::reset()
{
    clear();
    //setTextFormat(Qt::PlainText);
    setCurrentFont(QFont("Courier"));
    setReadOnly(false);
    _lastBlockNumber = 0;

    //init attributes
    //historyIndex = 0;
    //history.clear();
    //recordedScript.clear();
    displayPrompt();
}


bool LuaConsoleWidget::event(QEvent *event){
    if(event->type()==MESSAGE_ADDED_EVENT){
        WriterWrapper* writer = (WriterWrapper*)_logWriter.get();
        for(unsigned int i=0;i<writer->_msgQueue.size();i++){
            //write(writer->_msgQueue[i].first, writer->_msgQueue[i].second);
            insertPlainText(writer->_msgQueue[i].first.c_str());
        }
        writer->_msgQueue.clear();
        moveCursor(QTextCursor::End);
        return true;
    } else {
        event->ignore();
    }
    return QTextEdit::event(event);
}


//void LuaConsoleWidget::keyPressEvent(QKeyEvent *e)
//{
    /*
    if (_c && _c->popup()->isVisible()) {
        // The following keys are forwarded by the completer to the widget
       switch (e->key()) {
       case Qt::Key_Enter:
       case Qt::Key_Return:
       case Qt::Key_Escape:
       case Qt::Key_Tab:
       case Qt::Key_Backtab:
            e->ignore();
            return; // let the completer do default behavior
       default:
           break;
       }
    }

    bool isShortcut = ((e->modifiers() & Qt::ControlModifier) && (e->key() == Qt::Key_Space)); // CTRL+E
    if (!_c || !isShortcut){ // dont process the shortcut when we have a completer
        QPlainTextEdit::keyPressEvent(e);
        return;
    }

    const bool ctrlOrShift = e->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier);
    if (!_c || (ctrlOrShift && e->text().isEmpty()))
        return;

    static QString eow("~!@#$%^&*()_+{}|:\"<>?,/;'[]\\-="); // end of word
    bool hasModifier = (e->modifiers() != Qt::NoModifier) && !ctrlOrShift;
    QString completionPrefix = textUnderCursor();
    //std::cout << "textUnderCursor: " << completionPrefix.toStdString() << std::endl;
    if (!isShortcut && (hasModifier || e->text().isEmpty()|| completionPrefix.length() < 1
                      || eow.contains(e->text().right(1)))) {
        //std::cout << "HIDE " << std::endl;
        _c->popup()->hide();
        return;
    }



    if (completionPrefix != _c->completionPrefix()) {
        //std::cout << "text1" << std::endl;
        _c->setCompletionPrefix(completionPrefix);
        _c->popup()->setCurrentIndex(_c->completionModel()->index(0, 0));
    }
    //std::cout << "text2" << std::endl;

    QRect cr = cursorRect();
    cr.setWidth(_c->popup()->sizeHintForColumn(0)
                + _c->popup()->verticalScrollBar()->sizeHint().width());
    _c->complete(cr); // popup it up!
    _c->popup()->show();
    */
//}

//Correctly handle the cursor when moved according to the different actions
//void LuaConsoleWidget::moveCursor(CursorAction action, bool select)
void LuaConsoleWidget::moveCursor(QTextCursor::MoveOperation action, QTextCursor::MoveMode mode)
{
    /*
    int para, index;
    //save the old cursor position
    getCursorPosition(&para, &index );
    //if home pressed, move the cursor just after the prompt (if in the first line)
    //and select the covered text if needed
    if ( (action == QTextEdit::MoveLineStart) && (promptParagraph == para) )
    {
        if (select)
            setSelection(para, index, para, promptLength);
        else
            setCursorPosition(para, promptLength);
        return;
    }
    //Process the up & down keys to navigate into the history (if not empty & if in the first line)
    else if ( ( (action == QTEXTEDIT_CLASSNAME::MoveDown) || (action == QTEXTEDIT_CLASSNAME::MoveUp) )
        && history.size() && (promptParagraph == para))
    {
        //update the historyIndex if up or down
        if ( (action == QTEXTEDIT_CLASSNAME::MoveDown) && (historyIndex + 1 < history.size()) )
            historyIndex ++;
        else if ((action == QTEXTEDIT_CLASSNAME::MoveUp) && historyIndex)
            historyIndex --;
        //replace the current command with the command found in the history
        replaceCurrentCommand(history[historyIndex]);
    }
    else
        QTEXTEDIT_CLASSNAME::moveCursor(action, select);
    //Undo the new position if it is out of the edition zone and unselect as well
    if (!isInEditionZone())
    {
        setCursorPosition(para, index);
        selectAll(false);
    }

    */

}

bool LuaConsoleWidget::isInEditionZone()
{
    int /*para = textCursor().blockNumber(), */index = textCursor().columnNumber();
    return ( index >= _promptLen );
}

//Redirect keyboard actions to perform text suppression and validation
//of the commands
//void LuaConsoleWidget::doKeyboardAction(KeyboardAction action)
void LuaConsoleWidget::keyPressEvent(QKeyEvent *e)
{
    //Get the current paragraph and the current cursor position
    int /*para = textCursor().blockNumber(), */index = textCursor().columnNumber();

    const bool ctrlMod = e->modifiers() & Qt::ControlModifier ;
    //const bool shiftMod = e->modifiers() & Qt::ShiftModifier ;


    //Get the cursor
    QTextCursor cursor = textCursor();
    if(cursor.blockNumber()>_lastBlockNumber){
        cursor.movePosition(QTextCursor::End);
        setTextCursor(cursor);
        _lastBlockNumber = cursor.blockNumber();
    } else if( cursor.blockNumber()<_lastBlockNumber ){
        cursor.movePosition(QTextCursor::End);
        setTextCursor(cursor);
    }

    //std::cout << "Cursor: block=" << cursor.blockNumber() << std::endl;

    switch (e->key())
    {
        //Don't delete the prompt if backspace is pressed
        case Qt::Key_Backspace: {
            if(ctrlMod){
                //Don't delete the prompt if ctrl+backspace is pressed
                //trick to get the new position of the cursor
                moveCursor(QTextCursor::WordLeft);
                //exit if the new position is out of the edition zone
                bool error = false;
                if ( !isInEditionZone())
                    error = true;

                //setTextCursor(cursor);
                if (error)
                    return;
            } else if ( index == _promptLen ){
                // don't delete prompt
                return;
            }
        }
        break;
        case Qt::Key_Left: {
            if ( index == _promptLen ){
                // don't delete prompt
                return;
            }
        }
        break;

        case Qt::Key_Home: {
            cursor.movePosition(QTextCursor::StartOfBlock);
            for(int i=0;i<_promptLen;i++)
                cursor.movePosition(QTextCursor::Right);

            setTextCursor(cursor);
            return;
        }
        break;
        case Qt::Key_Up: {
            // take the last command in history
            int count = _commandHistory.count();
            if(count==0)
                return;
            _histIdx--;
            if(_histIdx<0)
                _histIdx = count-1;
            setCommand(_commandHistory[_histIdx]);

            return;
        }
        break;
        case Qt::Key_Down: {
            // take the last command in history
            int count = _commandHistory.count();
            if(count==0) return;
            _histIdx++;
            if(_histIdx>=count)
                _histIdx = 0;
            setCommand(_commandHistory[_histIdx]);
            return;
        }
        break;
        case Qt::Key_Return: {
            // If return pressed, do the evaluation and append the result

            cursor.movePosition(QTextCursor::End);
            setTextCursor(cursor);

            //Get the command to validate
            QString command = getCurrentCommand();
            // execute the command and get back its text result and its return value
            if ( isCommandComplete(command) ){
                append("");
                execCommand(command, false);
            } else
            {
                append("");
                moveCursor(QTextCursor::End);
            }
            return;
        }
        break;
        default:
            break;
    }

    //std::cout << cursor.blockNumber() << "!=" << _lastBlockNumber << std::endl;
    if(cursor.blockNumber()!=_lastBlockNumber){
        moveCursor(QTextCursor::End);
        return;
    }
    //If we are here, this means that we can perform the action
    //by calling the parent implementation
    QTextEdit::keyPressEvent(e);
}

void LuaConsoleWidget::displayPrompt(){
    moveCursor(QTextCursor::End);
    if(textCursor().columnNumber()>0)
        append(_promptStr);
    else
        insertPlainText(_promptStr);
    verticalScrollBar()->setValue( verticalScrollBar()->maximum() );
    _lastBlockNumber = textCursor().blockNumber();
}

QString LuaConsoleWidget::interpretCommand(QString cmd, int *res){
    *res = 0;

    int error = luaL_loadbuffer(_luastate->get(), cmd.toStdString().c_str(), cmd.size(), "");
    if (!error)
        error = lua_pcall(_luastate->get(), 0, 0, 0);

    QString resstring("");
    if(error){
        *res = 1;
        resstring = lua_tostring(_luastate->get(), -1);
        lua_pop(_luastate->get(), 1);
    } else {
        *res = 0;
    }

    return resstring;
}

bool LuaConsoleWidget::execCommand(QString command, bool b){
    // if we want to include echo of command...
    //displayPrompt();
    //insertPlainText(command);
    rwlibs::swig::setlog( _logWriter );
    setTextColor(_outColor);
    this->setEnabled(false);

    _luaRunner->set(command.toStdString(), _luastate , _logWriter);
    _luaRunner->start();

    return true;
}

void LuaConsoleWidget::setLuaState(rwlibs::swig::LuaState *lstate){
    //_luaRunner->_lua = lstate;
    _luastate=lstate;
}


void LuaConsoleWidget::runFinished(){

    int res = _luaRunner->getReturnValue();
    QString strRes = _luaRunner->getReturnString().c_str();
    QString command = _luaRunner->getCommand().c_str();



    //According to the return value, display the result either in red or in blue
    if (res == 0){
        setTextColor(_outColor);
    } else {
        setTextColor(_errColor);
        append(strRes);
    }
    _commandHistory.append(command);
    _histIdx = _commandHistory.count();

    //append(strRes);

    setTextColor(_cmdColor);

    this->setEnabled(true);

    //Display the prompt again
    displayPrompt();

    moveCursor(QTextCursor::End);
    setFocus(Qt::OtherFocusReason);
}

bool LuaConsoleWidget::isCommandComplete(QString command){
    return true;
}

void LuaConsoleWidget::setCommand(QString command)
{
    //Get the current command: we just remove the prompt
    QTextCursor cursor( textCursor() );
    cursor.beginEditBlock();
    cursor.movePosition(QTextCursor::StartOfBlock);
    cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
    cursor.endEditBlock();

    QString cmd(_promptStr+command);
    cursor.insertText(cmd);
    //selectAll(false);
}


QString LuaConsoleWidget::getCurrentCommand()
{
    //Get the current command: we just remove the prompt
    QTextCursor cursor( textCursor() );
    cursor.beginEditBlock();
    cursor.movePosition(QTextCursor::StartOfBlock);
    cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
    cursor.endEditBlock();

    QString command = cursor.selectedText();
    command.remove(0,_promptLen);
    //std::cout << "The command is: " << command.toStdString() << std::endl;
    //selectAll(false);
    return command;
}


