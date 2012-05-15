/*
 * LuaEditorWindow.hpp
 *
 *  Created on: 23/10/2009
 *      Author: jimali
 */

#ifndef LUAEDITORWINDOW_HPP_
#define LUAEDITORWINDOW_HPP_

#include <QMainWindow>
#include <QModelIndex>

#include <rw/common/Log.hpp>

#include "ui_LuaEditorWindow.h"
#include "LuaHighlighter.hpp"
#include "CodeEditor.hpp"

 #include <QMainWindow>


 class TreeModelCompleter;
 class QAbstractItemModel;
 class QComboBox;
 class QLabel;
 class QLineEdit;
 class QProgressBar;
 class QCheckBox;
 class QTreeView;

struct lua_State;
class QTextEdit;

class LuaEditorWindow: public QMainWindow, private Ui::LuaEditorWindow {
	Q_OBJECT

public:

	LuaEditorWindow(lua_State* lua, rw::common::LogPtr output, QWidget *parent);
	virtual ~LuaEditorWindow();

public slots:
	void about();
	void newFile();
	void openFile(const QString &path = QString());
	void runChunk();
	void run();

private:
    QAbstractItemModel *modelFromFile(const QString& fileName);


    void setupEditor();

    //QPlainTextEdit *_editor;
    CodeEditor *_editor;
    LuaHighlighter *_highlighter;
    lua_State *_lua;
    rw::common::LogPtr _output;

    //QCompleter *_completer;
    TreeModelCompleter *_completer;

    bool _isRunning;


};


#endif /* LUAEDITORWINDOW_HPP_ */
