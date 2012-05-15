/*
 * LuaHighlighter.hpp
 *
 *  Created on: 23/10/2009
 *      Author: jimali
 */

#ifndef LUAHIGHLIGHTER_HPP_
#define LUAHIGHLIGHTER_HPP_

#ifndef HIGHLIGHTER_H
 #define HIGHLIGHTER_H

 #include <QSyntaxHighlighter>

 #include <QHash>
 #include <QTextCharFormat>

 class QTextDocument;

 class LuaHighlighter : public QSyntaxHighlighter
 {
     Q_OBJECT

 public:
     LuaHighlighter(QTextDocument *parent = 0);

 protected:
     void highlightBlock(const QString &text);

 private:
     struct HighlightingRule
     {
         QRegExp pattern;
         QTextCharFormat format;
     };
     QVector<HighlightingRule> highlightingRules;

     QRegExp commentStartExpression;
     QRegExp commentEndExpression;

     QTextCharFormat keywordFormat;
     QTextCharFormat classFormat;
     QTextCharFormat singleLineCommentFormat;
     QTextCharFormat multiLineCommentFormat;
     QTextCharFormat quotationFormat;
     QTextCharFormat functionFormat;
 };

 #endif
#endif /* LUAHIGHLIGHTER_HPP_ */
