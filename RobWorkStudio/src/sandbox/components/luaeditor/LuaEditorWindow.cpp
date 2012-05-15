#include "LuaEditorWindow.hpp"

#include <QtGui>

extern "C" {
	#include <lua.h>
	#include <lualib.h>
	#include <lauxlib.h>
}

#include "TreeModelCompleter.hpp"

#include <rwlibs/lua/RobWork.hpp>
#include <rwlibs/lua/Output.hpp>
using namespace rwlibs::lua;
using namespace rw::common;

namespace {
	void processError(int error, lua_State* lua, LogPtr output)
	{
		if (error) {
			output->info() << lua_tostring(lua, -1) << "\n";
			lua_pop(lua, 1);
		}
	}

	void luaLineHook(lua_State *L, lua_Debug *ar){
		lua_getinfo(L, "nS", ar);
		std::cout << "Name: " << ar->name << " " << ar->namewhat << std::endl;
		std::cout << "Line: " << ar->currentline << " " << ar->linedefined << std::endl;

	}


}

LuaEditorWindow::LuaEditorWindow(lua_State* lua, rw::common::LogPtr output, QWidget *parent):
	QMainWindow(parent),
	_lua(lua),
	_output(output)
{
	setupUi(this);
    //setupFileMenu();
    //setupHelpMenu();
    setupEditor();

    connect(actionNew, SIGNAL(triggered(bool)), this, SLOT(openFile()));
    connect(actionOpen, SIGNAL(triggered(bool)), this, SLOT(openFile()));

    connect(actionStep, SIGNAL(triggered(bool)), this, SLOT(runChunk()));

    lua_sethook(_lua, luaLineHook, LUA_MASKLINE, 0);

	this->setCentralWidget(_editor);
}

LuaEditorWindow::~LuaEditorWindow(){

}

void LuaEditorWindow::about(){
    QMessageBox::about(this, tr("About Syntax Highlighter"),
                tr("<p>The <b>Syntax Highlighter</b> example shows how " \
                   "to perform simple syntax highlighting by subclassing " \
                   "the QSyntaxHighlighter class and describing " \
                   "highlighting rules using regular expressions.</p>"));
}

void LuaEditorWindow::newFile(){

}

void LuaEditorWindow::openFile(const QString &path){
    QString fileName = path;

    if (fileName.isNull())
        fileName = QFileDialog::getOpenFileName(this,
            tr("Open File"), "", "C++ Files (*.cpp *.h)");

    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.open(QFile::ReadOnly | QFile::Text))
            _editor->setPlainText(file.readAll());
    }
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

    QFile file("LuaEditorWindow.hpp");
    if (file.open(QFile::ReadOnly | QFile::Text))
        _editor->setPlainText(file.readAll());
}

void LuaEditorWindow::runChunk()
{


    const std::string cmd =
        _editor->textCursor().block().text().toStdString();

    _output->info() << "--\n";
    const int error = // The string "" is part of the error message.
        luaL_loadbuffer(_lua, cmd.data(), cmd.size(), "") ||
        lua_pcall(_lua, 0, 0, 0);

    processError(error, _lua, _output);
}


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

void LuaEditorWindow::run(){
	// start a thread and start executing lines

}
