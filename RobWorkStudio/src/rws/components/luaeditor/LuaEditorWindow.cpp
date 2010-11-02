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

#include "LuaEditorWindow.hpp"

#include <QtGui>
#include <QMessageBox>

extern "C" {
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}

#include "TreeModelCompleter.hpp"

#include <rwlibs/lua/LuaRobWork.hpp>
using namespace rwlibs::lua;
using namespace rw::common;




namespace {
	void processError(int error, lua_State* lua, Log::Ptr output)
	{
		if (error) {
			output->info() << lua_tostring(lua, -1) << "\n";
			lua_pop(lua, 1);
		}
	}

	void luaLineHook(lua_State *L, lua_Debug *ar){
		lua_getinfo(L, "nS", ar);
        if (ar->name != NULL && ar->namewhat != NULL) {
		    std::cout << "Name: " << ar->name << " " << ar->namewhat << std::endl;
		    std::cout << "Line: " << ar->currentline << " " << ar->linedefined << std::endl;
        }
	}


}

LuaEditorWindow::LuaEditorWindow(LuaState* lua, rw::common::Log::Ptr output, QWidget *parent):
	QMainWindow(parent),
	_lua(lua),
	_output(output)
{
	setupUi(this);
    setupEditor();

//    _modified = false;

    //lua_sethook(_lua, luaLineHook, LUA_MASKLINE, 0);

	this->setCentralWidget(_editor);

  //  _modified = false;
}

LuaEditorWindow::~LuaEditorWindow(){

}


void LuaEditorWindow::setupEditor(){
    QFont font;
    font.setFamily("Courier");
    font.setFixedPitch(true);
    font.setPointSize(10);

    //_editor = new QTextEdit;
    _editor = new CodeEditor(this);
    _editor->setFont(font);

    //_completer = new QCompleter(this);
    _completer = new TreeModelCompleter(this);
    _completer->setSeparator(QLatin1String("."));

    _completer->setModel( modelFromFile(":/wordlist.txt") );
    _completer->setModelSorting(QCompleter::CaseInsensitivelySortedModel);
    _completer->setCaseSensitivity(Qt::CaseInsensitive);
    _completer->setWrapAround(false);

    _editor->setCompleter(_completer);

    _highlighter = new LuaHighlighter(_editor->document());

    _luaRunner =  new LuaRunThread("",_lua,_output);

    connect(_editor, SIGNAL(modificationChanged(bool)), this, SLOT(textChanged()));

    connect(_luaRunner, SIGNAL(finished()), this, SLOT(runFinished()));

}


void LuaEditorWindow::runFinished() {
    //std::cout << "FINISHED" << std::endl;
    _editor->setEnabled(true);
}
void LuaEditorWindow::textChanged() {
    //_modified = _editor->document()->isModified();
}



void LuaEditorWindow::on_actionNew_triggered(bool) {
    if (_editor->document()->isModified()) {
    //if (_modified) {
        int result = QMessageBox::warning(this, "Lua Editor", tr("Content has been modified. Do you wish to save changes?"), QMessageBox::Yes, QMessageBox::No, QMessageBox::Cancel);
        switch (result) {
            case QMessageBox::Yes:
                if (!save())
                    return;
                break;
            case QMessageBox::No:
                break;
            case QMessageBox::Cancel:
                return;
        }
    }
    _editor->clear();
    _editor->document()->setModified(false);
}

void LuaEditorWindow::on_actionOpen_triggered(bool) {
    on_actionNew_triggered(true);
    QString path = _pmap.get<std::string>("PreviousOpenDirectory", ".").c_str();

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open File"), path,
                                                    "Supported (*.lua *.txt)"
                                                    "\nLua Files (*.lua)"
                                                    "\nAll (*.*)");

    if (!fileName.isEmpty()) {
    	_pmap.set<std::string>("PreviousOpenDirectory", StringUtil::getDirectoryName(fileName.toStdString()));
        _pmap.set<std::string>("LuaFile", fileName.toStdString());
        QFile file;
        file.setFileName(fileName);
        if (file.open(QFile::ReadWrite | QFile::Text)) {
            _editor->setPlainText(file.readAll());
        }
        file.close();
        _editor->document()->setModified(false);
    }

}


void LuaEditorWindow::on_actionSave_triggered(bool) {
    save();
}

bool LuaEditorWindow::save() {
    std::string filename = _pmap.get<std::string>("LuaFile","");
    if (filename == "") {
        return saveAs();
    } else {
        return save(filename);
    }
}

bool LuaEditorWindow::saveAs() {
    std::string defaultName = _pmap.get<std::string>("LuaFile","");
    QString filename = QFileDialog::getSaveFileName(this,
                                                    "Save as",
                                                    defaultName.c_str(),
                                                    "Supported (*.lua *.txt)");

    if (filename.isEmpty())
        return false;

    return save(filename.toStdString());
}

bool LuaEditorWindow::save(const std::string& filename) {
    QFile file;
    file.setFileName(filename.c_str());
    if (file.open(QFile::ReadWrite | QFile::Text)) {
        file.write(_editor->toPlainText().toAscii());
        file.close();
        _pmap.set<std::string>("LuaFile", filename);
        _editor->document()->setModified(false);
        return true;
    } else {
        file.close();
        return false;
    }
}

 void LuaRunThread::run()
 {
     //QObject::moveToThread(this);
     try {
         //const std::string cmd = _editor->textCursor().block().text().toStdString();
         int error = luaL_loadbuffer(_lua->get(), _cmd.data(), _cmd.size(), "");
         if (!error)
             error = lua_pcall(_lua->get(), 0, 0, 0);

         if(error){
             //_editor->setLineState(number, CodeEditor::ExecutedError);
             processError(error, _lua->get(), _output);
         }
     } catch (const Exception& exp) {
         QMessageBox::critical(NULL, "Lua Editor", tr("Failed to execute script with message '%1'").arg(exp.what().c_str()));
     }
 }

void LuaEditorWindow::on_actionRun_triggered(bool) {
    _lua->reset();
    rwlibs::lua::setLualog( &Log::infoLog() );

    const std::string cmd = _editor->toPlainText().toStdString();
    _editor->setEnabled(false);
    _luaRunner->set(cmd,_lua,_output);
    _luaRunner->start();
    //_luaRunner->run(); // TODO: some lua code will not work in separate threads
}

void LuaEditorWindow::on_actionReload_triggered(bool) {
    std::string fileName = _pmap.get<std::string>("LuaFile","");
    if (fileName == "")
        return;

    QFile file;
    file.setFileName(fileName.c_str());
    if (file.open(QFile::ReadWrite | QFile::Text)) {
        _editor->setPlainText(file.readAll());
    }
    file.close();

}

void LuaEditorWindow::on_actionStop_triggered(bool) {

}


/*
void LuaEditorWindow::runChunk()
{


    const std::string cmd = _editor->textCursor().block().text().toStdString();
    int number = _editor->textCursor().blockNumber();

    _output->info() << "--\n";
    // The string "" is part of the error message.
    int error = luaL_loadbuffer(_lua, cmd.data(), cmd.size(), "");
    if (!error)
        error = lua_pcall(_lua, 0, 0, 0);

    if(error){
    	_editor->setLineState(number, CodeEditor::ExecutedError);
    	processError(error, _lua, _output);
    } else {
    	_editor->setLineState(number, CodeEditor::Executed);
    }


    _editor->moveCursor(QTextCursor::Down);
}
*/

QAbstractItemModel *LuaEditorWindow::modelFromFile(const QString& fileName)
 {
     QFile file(fileName);
     if (!file.open(QFile::ReadOnly))
         return new QStringListModel(_completer);

 #ifndef QT_NO_CURSOR
     QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
 #endif
     QStringList words;

     QStandardItemModel *model = new QStandardItemModel(_completer);
     QVector<QStandardItem *> parents(10);
     parents[0] = model->invisibleRootItem();

     while (!file.atEnd()) {
         QString line = file.readLine();
         QString trimmedLine = line.trimmed();
         if (line.isEmpty() || trimmedLine.isEmpty())
             continue;

         QRegExp re("^\\s+");
         int nonws = re.indexIn(line);
         int level = 0;
         if (nonws == -1) {
             level = 0;
         } else {
             if (line.startsWith("\t")) {
                 level = re.cap(0).length();
             } else {
                 level = re.cap(0).length()/4;
             }
         }

         if (level+1 >= parents.size())
             parents.resize(parents.size()*2);

         QStandardItem *item = new QStandardItem;
         item->setText(trimmedLine);
         parents[level]->appendRow(item);
         parents[level+1] = item;
     }

 #ifndef QT_NO_CURSOR
     QApplication::restoreOverrideCursor();
 #endif

     return model;
 }

#ifdef NOT_TREE_MODEL
QAbstractItemModel *LuaEditorWindow::modelFromFile(const QString& fileName)
 {
     QFile file(fileName);
     if (!file.open(QFile::ReadOnly)){
         std::cout << "COULD NOT OPEN RESOURCE FILE!!" << std::endl;
         return new QStringListModel(_completer);
     }

 #ifndef QT_NO_CURSOR
     QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
 #endif
     QStringList words;

     while (!file.atEnd()) {
         QByteArray line = file.readLine();
         if (!line.isEmpty())
             words << line.trimmed();
     }

 #ifndef QT_NO_CURSOR
     QApplication::restoreOverrideCursor();
 #endif
     return new QStringListModel(words, _completer);
 }
#endif

/*void LuaEditorWindow::run(){
	// start a thread and start executing lines

}
*/
