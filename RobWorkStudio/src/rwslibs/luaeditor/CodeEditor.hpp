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

#ifndef CODEEDITOR_HPP_
#define CODEEDITOR_HPP_

 #include <QPlainTextEdit>
 #include <QObject>

//#include <iostream>

 class QPaintEvent;
 class QResizeEvent;
 class QSize;
 class QWidget;

 class QCompleter;


 class CodeEditor : public QPlainTextEdit
 {
     Q_OBJECT

 public:
     CodeEditor(QWidget *parent = 0);

     void lineNumberAreaPaintEvent(QPaintEvent *event);
     int lineNumberAreaWidth();

     void setCompleter(QCompleter *c);
     QCompleter *completer() const;

     typedef enum{Nothing, Executed, ExecutedError, HighLighted} LineState;
     void setLineState(size_t linenr, LineState state){
    	 //std::cout << "setLineState: " << linenr << std::endl;
    	 if(linenr<0)
    		 return;
    	 if(_executedLines.size()<=linenr)
    		 _executedLines.resize(linenr+100);
    	 _executedLines[linenr] = state;
     }
     CodeEditor::LineState getLineState(int lineNr);

 protected:
     void resizeEvent(QResizeEvent *event);

     void keyPressEvent(QKeyEvent *e);
     void focusInEvent(QFocusEvent *e);

 public slots:
     void insertCompletion(const QString &completion);

 private slots:
     void updateLineNumberAreaWidth(int newBlockCount);
     void highlightCurrentLine();
     void updateLineNumberArea(const QRect &, int);



 private:
     QString textUnderCursor() const;
     bool hasExecuted(int lineNr);

 private:
     QWidget *lineNumberArea;
     QCompleter *_c;
     std::vector<LineState> _executedLines;

 };


 class LineNumberArea : public QWidget
 {
 public:
     LineNumberArea(CodeEditor *editor) : QWidget(editor) {
         codeEditor = editor;
     }

     QSize sizeHint() const {
         return QSize(codeEditor->lineNumberAreaWidth(), 0);
     }

 protected:
     void paintEvent(QPaintEvent *event) {
         codeEditor->lineNumberAreaPaintEvent(event);
     }

 private:
     CodeEditor *codeEditor;
 };


#endif /* CODEEDITOR_HPP_ */
