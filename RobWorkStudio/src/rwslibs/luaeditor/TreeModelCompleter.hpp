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
