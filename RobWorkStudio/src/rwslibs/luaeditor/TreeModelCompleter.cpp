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

 #include "TreeModelCompleter.hpp"
 #include <QStringList>

 TreeModelCompleter::TreeModelCompleter(QObject *parent)
     : QCompleter(parent)
 {
 }

 TreeModelCompleter::TreeModelCompleter(QAbstractItemModel *model, QObject *parent)
     : QCompleter(model, parent)
 {
 }

 void TreeModelCompleter::setSeparator(const QString &separator)
 {
     sep = separator;
 }

 QString TreeModelCompleter::separator() const
 {
     return sep;
 }

 QStringList TreeModelCompleter::splitPath(const QString &path) const
 {
     if (sep.isNull()) {
         return QCompleter::splitPath(path);
     }

     return path.split(sep);
 }

 QString TreeModelCompleter::pathFromIndex(const QModelIndex &index) const
 {
     if (sep.isNull()) {
         return QCompleter::pathFromIndex(index);
     }

     // navigate up and accumulate data
     QStringList dataList;
     for (QModelIndex i = index; i.isValid(); i = i.parent()) {
         dataList.prepend(model()->data(i, completionRole()).toString());
     }

     return dataList.join(sep);
 }
