/*
 * TreeModelCompleter.hpp
 *
 *  Created on: 23/10/2009
 *      Author: jimali
 */

#ifndef TREEMODELCOMPLETER_HPP_
#define TREEMODELCOMPLETER_HPP_

 #include <QCompleter>

 class TreeModelCompleter : public QCompleter
 {
     Q_OBJECT
     Q_PROPERTY(QString separator READ separator WRITE setSeparator)

 public:
     TreeModelCompleter(QObject *parent = 0);
     TreeModelCompleter(QAbstractItemModel *model, QObject *parent = 0);

     QString separator() const;
 public slots:
     void setSeparator(const QString &separator);

 protected:
     QStringList splitPath(const QString &path) const;
     QString pathFromIndex(const QModelIndex &index) const;

 private:
     QString sep;
 };


#endif /* TREEMODELCOMPLETER_HPP_ */
