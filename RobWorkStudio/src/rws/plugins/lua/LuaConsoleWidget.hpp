/*
 * LuaConsole.hpp
 *
 *  Created on: 02/09/2010
 *      Author: jimali
 */

#ifndef LUACONSOLE_HPP_
#define LUACONSOLE_HPP_

#include <qstringlist.h>
#include <QTextEdit>
#include <QObject>

#include <rw/common/LogWriter.hpp>

#include <rws/components/luaeditor/LuaState.hpp>

class LuaConsoleWidget : public QTextEdit {
    Q_OBJECT
public:
    LuaConsoleWidget(QWidget *parent = 0);
    virtual ~LuaConsoleWidget(){}
    //void setCompleter(QCompleter *c);
    //QCompleter *completer() const;


    void reset();

    void setLuaState(LuaState *lstate){_luastate=lstate;}

protected:
    //void resizeEvent(QResizeEvent *event);

    void keyPressEvent(QKeyEvent *e);

    virtual QString interpretCommand(QString str, int *res);

    //void focusInEvent(QFocusEvent *e);
private slots:
    //Correctly handle the cursor when moved
    void moveCursor(QTextCursor::MoveOperation action, QTextCursor::MoveMode mode = QTextCursor::MoveAnchor);


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

    LuaState *_luastate;
};

#endif /* LUACONSOLE_HPP_ */
