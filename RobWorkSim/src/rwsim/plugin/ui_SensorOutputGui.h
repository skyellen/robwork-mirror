/********************************************************************************
** Form generated from reading UI file 'SensorOutputGui.ui'
**
** Created: Mon 28. Jun 12:34:03 2010
**      by: Qt User Interface Compiler version 4.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSOROUTPUTGUI_H
#define UI_SENSOROUTPUTGUI_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QFormLayout>
#include <QtGui/QGraphicsView>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SensorOutputGui
{
public:
    QHBoxLayout *horizontalLayout;
    QGraphicsView *_graphicsView;
    QDockWidget *dockWidget;
    QWidget *dockWidgetContents;
    QFormLayout *formLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *tab_2;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *SensorOutputGui)
    {
        if (SensorOutputGui->objectName().isEmpty())
            SensorOutputGui->setObjectName(QString::fromUtf8("SensorOutputGui"));
        SensorOutputGui->resize(548, 503);
        horizontalLayout = new QHBoxLayout(SensorOutputGui);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        _graphicsView = new QGraphicsView(SensorOutputGui);
        _graphicsView->setObjectName(QString::fromUtf8("_graphicsView"));

        horizontalLayout->addWidget(_graphicsView);

        dockWidget = new QDockWidget(SensorOutputGui);
        dockWidget->setObjectName(QString::fromUtf8("dockWidget"));
        dockWidget->setMaximumSize(QSize(171, 524287));
        dockWidget->setFloating(false);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        formLayout = new QFormLayout(dockWidgetContents);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());

        formLayout->setWidget(0, QFormLayout::LabelRole, tabWidget);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        formLayout->setItem(1, QFormLayout::LabelRole, verticalSpacer);

        dockWidget->setWidget(dockWidgetContents);

        horizontalLayout->addWidget(dockWidget);


        retranslateUi(SensorOutputGui);

        QMetaObject::connectSlotsByName(SensorOutputGui);
    } // setupUi

    void retranslateUi(QWidget *SensorOutputGui)
    {
        SensorOutputGui->setWindowTitle(QApplication::translate("SensorOutputGui", "Form", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SensorOutputGui", "Tab 1", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SensorOutputGui", "Tab 2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SensorOutputGui: public Ui_SensorOutputGui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSOROUTPUTGUI_H
