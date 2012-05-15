/*
 * CodeEditor.hpp
 *
 *  Created on: 23/10/2009
 *      Author: jimali
 */

#ifndef CODEEDITOR_HPP_
#define CODEEDITOR_HPP_

 #include <QPlainTextEdit>
 #include <QObject>

 class QPaintEvent;
 class QResizeEvent;
 class QSize;
 class QWidget;

 class LineNumberArea;

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

 protected:
     void resizeEvent(QResizeEvent *event);

     void keyPressEvent(QKeyEvent *e);
     void focusInEvent(QFocusEvent *e);

 private slots:
     void updateLineNumberAreaWidth(int newBlockCount);
     void highlightCurrentLine();
     void updateLineNumberArea(const QRect &, int);

     void insertCompletion(const QString &completion);

 private:
     QString textUnderCursor() const;
     bool hasExecuted(int lineNr);

 private:
     QWidget *lineNumberArea;
     QCompleter *_c;

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
