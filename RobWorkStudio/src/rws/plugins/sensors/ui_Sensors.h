/********************************************************************************
** Form generated from reading UI file 'Sensors.ui'
**
** Created: Fri May 28 00:16:25 2010
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSORS_H
#define UI_SENSORS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SensorPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton;

    void setupUi(QDockWidget *SensorPlugin)
    {
        if (SensorPlugin->objectName().isEmpty())
            SensorPlugin->setObjectName(QString::fromUtf8("SensorPlugin"));
        SensorPlugin->resize(506, 210);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        pushButton = new QPushButton(dockWidgetContents);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout_2->addWidget(pushButton);


        verticalLayout->addLayout(verticalLayout_2);

        SensorPlugin->setWidget(dockWidgetContents);

        retranslateUi(SensorPlugin);

        QMetaObject::connectSlotsByName(SensorPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SensorPlugin)
    {
        SensorPlugin->setWindowTitle(QApplication::translate("SensorPlugin", "Log", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("SensorPlugin", "PushButton", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SensorPlugin: public Ui_SensorPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSORS_H
