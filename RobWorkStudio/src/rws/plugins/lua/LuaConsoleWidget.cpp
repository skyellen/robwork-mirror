#include "LuaConsoleWidget.hpp"

#include <QtGui>
#include <iostream>

#include <rwlibs/lua/LuaRobWork.hpp>


using namespace rw::common;

namespace {
class WriterWrapper: public LogWriter {
public:
    WriterWrapper(QTextEdit* slog):
        _slog(slog)
    {
    }

    virtual ~WriterWrapper(){}

    virtual void flush(){
    }

    /**
     * @brief Writes \b str to the log
     * @param str [in] message to write
     */
    virtual void write(const std::string& str){
        //_slog->setTextCursor(*_endCursor);
        _slog->insertPlainText(str.c_str());
    }

    virtual void writeln(const std::string& str){
        //_slog->append(str.c_str());
        _slog->insertPlainText(str.c_str());

    }
private:
    QTextEdit *_slog;
    QColor _color;
};
}

LuaConsoleWidget::LuaConsoleWidget(QWidget *parent) :
     QTextEdit(parent),_promptLen(1),_promptStr(">"),_histIdx(0),
     _cmdColor(Qt::black), _errColor(Qt::red), _outColor(Qt::blue), _completionColor(Qt::green),
     _luastate(NULL),_lastBlockNumber(0)
 {
    _logWriter = ownedPtr( new WriterWrapper(this) );
     //connect(this, SIGNAL(blockCountChanged(int)), this, SLOT(updateLineNumberAreaWidth(int)));
     //connect(this, SIGNAL(updateRequest(const QRect &, int)), this, SLOT(updateLineNumberArea(const QRect &, int)));
     //connect(this, SIGNAL(cursorPositionChanged()), this, SLOT(highlightCurrentLine()));

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
    int para, index;
    /*
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
    int para = textCursor().blockNumber(), index = textCursor().columnNumber();
    return ( index >= _promptLen );
}

//Redirect keyboard actions to perform text suppression and validation
//of the commands
//void LuaConsoleWidget::doKeyboardAction(KeyboardAction action)
void LuaConsoleWidget::keyPressEvent(QKeyEvent *e)
{
    //Get the current paragraph and the current cursor position
    int para = textCursor().blockNumber(), index = textCursor().columnNumber();

    const bool ctrlMod = e->modifiers() & Qt::ControlModifier ;
    const bool shiftMod = e->modifiers() & Qt::ShiftModifier ;


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
            if(count==0) return;
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
    //std::cout << "Executing command:" << command.toStdString() << std::endl;
    // make sure to output to console window

    rwlibs::lua::setLualog(_logWriter);

    // if we want to include echo of command...
    //displayPrompt();
    //insertPlainText(command);
    setTextColor(_outColor);

    int res;
    QString strRes = interpretCommand(command, &res);

    //According to the return value, display the result either in red or in blue
    if (res == 0){
        setTextColor(_outColor);
        _histIdx = _commandHistory.count();
    } else {
        setTextColor(_errColor);
        append(strRes);
    }

    _commandHistory.append(command);
    //append(strRes);

    setTextColor(_cmdColor);

    //Display the prompt again
    displayPrompt();


    return true;
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


