/********************************************************************************
** Form generated from reading UI file 'SensorsPlugin.ui'
**
** Created: Sat 30. Oct 17:30:51 2010
**      by: Qt User Interface Compiler version 4.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SENSORSPLUGIN_H
#define UI_SENSORSPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SensorsPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_2;
    QComboBox *cmbSensors;
    QPushButton *btnDisplay;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *spnUpdateTime;
    QSpacerItem *verticalSpacer;

    void setupUi(QDockWidget *SensorsPlugin)
    {
        if (SensorsPlugin->objectName().isEmpty())
            SensorsPlugin->setObjectName(QString::fromUtf8("SensorsPlugin"));
        SensorsPlugin->resize(506, 210);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        cmbSensors = new QComboBox(dockWidgetContents);
        cmbSensors->setObjectName(QString::fromUtf8("cmbSensors"));

        verticalLayout_2->addWidget(cmbSensors);

        btnDisplay = new QPushButton(dockWidgetContents);
        btnDisplay->setObjectName(QString::fromUtf8("btnDisplay"));

        verticalLayout_2->addWidget(btnDisplay);

        widget = new QWidget(dockWidgetContents);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        spnUpdateTime = new QSpinBox(widget);
        spnUpdateTime->setObjectName(QString::fromUtf8("spnUpdateTime"));
        spnUpdateTime->setCursor(QCursor(Qt::ArrowCursor));
        spnUpdateTime->setContextMenuPolicy(Qt::NoContextMenu);
        spnUpdateTime->setMinimum(1);
        spnUpdateTime->setMaximum(10000);
        spnUpdateTime->setSingleStep(10);
        spnUpdateTime->setValue(100);

        horizontalLayout->addWidget(spnUpdateTime);


        verticalLayout_2->addWidget(widget);


        verticalLayout->addLayout(verticalLayout_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        SensorsPlugin->setWidget(dockWidgetContents);

        retranslateUi(SensorsPlugin);

        QMetaObject::connectSlotsByName(SensorsPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SensorsPlugin)
    {
        SensorsPlugin->setWindowTitle(QApplication::translate("SensorsPlugin", "Sensors", 0, QApplication::UnicodeUTF8));
        btnDisplay->setText(QApplication::translate("SensorsPlugin", "Display Sensor", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SensorsPlugin", "Update Time (ms)", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SensorsPlugin: public Ui_SensorsPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SENSORSPLUGIN_H
