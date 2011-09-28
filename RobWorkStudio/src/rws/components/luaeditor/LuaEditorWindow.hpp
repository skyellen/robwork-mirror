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

#ifndef LUAEDITORWINDOW_HPP_
#define LUAEDITORWINDOW_HPP_

#include <QMainWindow>
#include <QModelIndex>

#include <rw/common/Log.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/lua/LuaState.hpp>
#include "ui_LuaEditorWindow.h"
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

class LuaRunThread : public QThread
 {
    Q_OBJECT
 public:
     LuaRunThread(const std::string& cmd,
                  LuaState *lstate,
				  rw::common::Log::Ptr output,
				  QWidget *parent=NULL):
				      QThread(parent),
                      _cmd(cmd),
                      _lua(lstate),
                      _output(output)
     {}

     void set(const std::string& cmd,
                  LuaState *lstate,
				  rw::common::Log::Ptr output)
     {
         _cmd = cmd;
         _lua = lstate;
         _output = output;
     }

     void run();

     std::string _cmd;
     LuaState *_lua;
	 rw::common::Log::Ptr _output;
 };


class LuaEditorWindow: public QMainWindow, private Ui::LuaEditorWindow {
	Q_OBJECT

public:

	LuaEditorWindow(LuaState* lua, rw::common::Log::Ptr output, rws::RobWorkStudio* rwstudio, QWidget *parent);
	virtual ~LuaEditorWindow();

	void setLuaState(LuaState* lua){_lua = lua;}

public slots:
	//void newFile();
	//void openFile(const QString &path = QString());
	//void closeFile();
	//void runChunk();
	//void run();

    void on_actionNew_triggered(bool);
    void on_actionOpen_triggered(bool);
    void on_actionSave_triggered(bool);
    void on_actionRun_triggered(bool);
    void on_actionStop_triggered(bool);
    void on_actionReload_triggered(bool);

    void textChanged();

    void runFinished();

    void ShowContextMenu(const QPoint& p);
    void setCheckAction(QAction*);


private:
    QAbstractItemModel *modelFromFile(const QString& fileName);


    void setupEditor();

    //QPlainTextEdit *_editor;
    CodeEditor *_editor;
    LuaHighlighter *_highlighter;
    LuaState *_lua;
	rw::common::Log::Ptr _output;
    rw::common::PropertyMap _pmap;
    //QCompleter *_completer;
    TreeModelCompleter *_completer;

    //bool _modified;
    bool _isRunning;

    bool save();
    bool saveAs();
    bool save(const std::string& filename);

    LuaRunThread *_luaRunner;
    rws::RobWorkStudio* _rws;
};


#endif /* LUAEDITORWINDOW_HPP_ */
