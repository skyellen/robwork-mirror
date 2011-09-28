/********************************************************************************
** Form generated from reading UI file 'RWSimPlugin.ui'
**
** Created: Mon Aug 1 12:19:15 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RWSIMPLUGIN_H
#define UI_RWSIMPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RWSimPlugin
{
public:
    QWidget *dockWidgetContents;
    QHBoxLayout *horizontalLayout_4;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_6;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *_openDwcBtn;
    QPushButton *_openLastDwcBtn;
    QPushButton *_closeDwcBtn;
    QPushButton *_editDwcBtn;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *_createSimulatorBtn;
    QPushButton *_destroySimulatorBtn;
    QPushButton *_simConfigBtn;
    QGroupBox *_controlGroupBox;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_5;
    QToolButton *_startBtn;
    QToolButton *_stepBtn;
    QToolButton *_stopBtn;
    QToolButton *_resetBtn;
    QSpacerItem *horizontalSpacer;
    QDoubleSpinBox *_timeStepSpin;
    QLabel *label;
    QDoubleSpinBox *_timeScaleSpin;
    QLabel *label_3;
    QGroupBox *_deviceGroupBox;
    QVBoxLayout *verticalLayout;
    QComboBox *_deviceControlBox;
    QPushButton *_openDeviceCtrlBtn;
    QPushButton *_tactileSensorBtn;
    QGroupBox *_loggingGroupBox;
    QVBoxLayout *verticalLayout_5;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QSpinBox *_debugLevelSpin;
    QCheckBox *_recordStatePathBox;
    QPushButton *_saveStatePathBtn;
    QCheckBox *_forceSceneUpdate;
    QHBoxLayout *horizontalLayout_2;
    QDoubleSpinBox *_updateIntervalSpin;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_5;
    QLabel *_timeLbl;

    void setupUi(QDockWidget *RWSimPlugin)
    {
        if (RWSimPlugin->objectName().isEmpty())
            RWSimPlugin->setObjectName(QString::fromUtf8("RWSimPlugin"));
        RWSimPlugin->resize(238, 698);
        RWSimPlugin->setAutoFillBackground(false);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        horizontalLayout_4 = new QHBoxLayout(dockWidgetContents);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        scrollArea = new QScrollArea(dockWidgetContents);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setFrameShape(QFrame::StyledPanel);
        scrollArea->setFrameShadow(QFrame::Sunken);
        scrollArea->setLineWidth(1);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 201, 738));
        verticalLayout_6 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMaximumSize(QSize(16777215, 150));
        groupBox->setFlat(false);
        groupBox->setCheckable(false);
        groupBox->setChecked(false);
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(9, -1, -1, -1);
        _openDwcBtn = new QPushButton(groupBox);
        _openDwcBtn->setObjectName(QString::fromUtf8("_openDwcBtn"));

        verticalLayout_2->addWidget(_openDwcBtn);

        _openLastDwcBtn = new QPushButton(groupBox);
        _openLastDwcBtn->setObjectName(QString::fromUtf8("_openLastDwcBtn"));

        verticalLayout_2->addWidget(_openLastDwcBtn);

        _closeDwcBtn = new QPushButton(groupBox);
        _closeDwcBtn->setObjectName(QString::fromUtf8("_closeDwcBtn"));

        verticalLayout_2->addWidget(_closeDwcBtn);

        _editDwcBtn = new QPushButton(groupBox);
        _editDwcBtn->setObjectName(QString::fromUtf8("_editDwcBtn"));

        verticalLayout_2->addWidget(_editDwcBtn);


        verticalLayout_6->addWidget(groupBox);

        groupBox_2 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        groupBox_2->setFlat(false);
        groupBox_2->setCheckable(false);
        verticalLayout_3 = new QVBoxLayout(groupBox_2);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        _createSimulatorBtn = new QPushButton(groupBox_2);
        _createSimulatorBtn->setObjectName(QString::fromUtf8("_createSimulatorBtn"));

        verticalLayout_3->addWidget(_createSimulatorBtn);

        _destroySimulatorBtn = new QPushButton(groupBox_2);
        _destroySimulatorBtn->setObjectName(QString::fromUtf8("_destroySimulatorBtn"));

        verticalLayout_3->addWidget(_destroySimulatorBtn);

        _simConfigBtn = new QPushButton(groupBox_2);
        _simConfigBtn->setObjectName(QString::fromUtf8("_simConfigBtn"));

        verticalLayout_3->addWidget(_simConfigBtn);

        _controlGroupBox = new QGroupBox(groupBox_2);
        _controlGroupBox->setObjectName(QString::fromUtf8("_controlGroupBox"));
        verticalLayout_7 = new QVBoxLayout(_controlGroupBox);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        _startBtn = new QToolButton(_controlGroupBox);
        _startBtn->setObjectName(QString::fromUtf8("_startBtn"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/forward.png"), QSize(), QIcon::Selected, QIcon::Off);
        _startBtn->setIcon(icon);

        horizontalLayout_5->addWidget(_startBtn);

        _stepBtn = new QToolButton(_controlGroupBox);
        _stepBtn->setObjectName(QString::fromUtf8("_stepBtn"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/end.png"), QSize(), QIcon::Selected, QIcon::Off);
        _stepBtn->setIcon(icon1);

        horizontalLayout_5->addWidget(_stepBtn);

        _stopBtn = new QToolButton(_controlGroupBox);
        _stopBtn->setObjectName(QString::fromUtf8("_stopBtn"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/pause.png"), QSize(), QIcon::Selected, QIcon::Off);
        _stopBtn->setIcon(icon2);

        horizontalLayout_5->addWidget(_stopBtn);

        _resetBtn = new QToolButton(_controlGroupBox);
        _resetBtn->setObjectName(QString::fromUtf8("_resetBtn"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/reload.png"), QSize(), QIcon::Selected, QIcon::Off);
        _resetBtn->setIcon(icon3);

        horizontalLayout_5->addWidget(_resetBtn);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_7->addLayout(horizontalLayout_5);

        _timeStepSpin = new QDoubleSpinBox(_controlGroupBox);
        _timeStepSpin->setObjectName(QString::fromUtf8("_timeStepSpin"));
        _timeStepSpin->setDecimals(3);
        _timeStepSpin->setMaximum(10);
        _timeStepSpin->setSingleStep(0.01);
        _timeStepSpin->setValue(0.01);

        verticalLayout_7->addWidget(_timeStepSpin);

        label = new QLabel(_controlGroupBox);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_7->addWidget(label);

        _timeScaleSpin = new QDoubleSpinBox(_controlGroupBox);
        _timeScaleSpin->setObjectName(QString::fromUtf8("_timeScaleSpin"));
        _timeScaleSpin->setDecimals(3);
        _timeScaleSpin->setSingleStep(0.01);

        verticalLayout_7->addWidget(_timeScaleSpin);

        label_3 = new QLabel(_controlGroupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_7->addWidget(label_3);


        verticalLayout_3->addWidget(_controlGroupBox);

        _deviceGroupBox = new QGroupBox(groupBox_2);
        _deviceGroupBox->setObjectName(QString::fromUtf8("_deviceGroupBox"));
        _deviceGroupBox->setEnabled(true);
        verticalLayout = new QVBoxLayout(_deviceGroupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _deviceControlBox = new QComboBox(_deviceGroupBox);
        _deviceControlBox->setObjectName(QString::fromUtf8("_deviceControlBox"));

        verticalLayout->addWidget(_deviceControlBox);

        _openDeviceCtrlBtn = new QPushButton(_deviceGroupBox);
        _openDeviceCtrlBtn->setObjectName(QString::fromUtf8("_openDeviceCtrlBtn"));

        verticalLayout->addWidget(_openDeviceCtrlBtn);


        verticalLayout_3->addWidget(_deviceGroupBox);

        _tactileSensorBtn = new QPushButton(groupBox_2);
        _tactileSensorBtn->setObjectName(QString::fromUtf8("_tactileSensorBtn"));

        verticalLayout_3->addWidget(_tactileSensorBtn);

        _loggingGroupBox = new QGroupBox(groupBox_2);
        _loggingGroupBox->setObjectName(QString::fromUtf8("_loggingGroupBox"));
        verticalLayout_5 = new QVBoxLayout(_loggingGroupBox);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_2 = new QLabel(_loggingGroupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        _debugLevelSpin = new QSpinBox(_loggingGroupBox);
        _debugLevelSpin->setObjectName(QString::fromUtf8("_debugLevelSpin"));

        horizontalLayout->addWidget(_debugLevelSpin);


        verticalLayout_4->addLayout(horizontalLayout);

        _recordStatePathBox = new QCheckBox(_loggingGroupBox);
        _recordStatePathBox->setObjectName(QString::fromUtf8("_recordStatePathBox"));

        verticalLayout_4->addWidget(_recordStatePathBox);

        _saveStatePathBtn = new QPushButton(_loggingGroupBox);
        _saveStatePathBtn->setObjectName(QString::fromUtf8("_saveStatePathBtn"));
        _saveStatePathBtn->setMaximumSize(QSize(101, 24));

        verticalLayout_4->addWidget(_saveStatePathBtn);

        _forceSceneUpdate = new QCheckBox(_loggingGroupBox);
        _forceSceneUpdate->setObjectName(QString::fromUtf8("_forceSceneUpdate"));
        _forceSceneUpdate->setChecked(true);
        _forceSceneUpdate->setTristate(false);

        verticalLayout_4->addWidget(_forceSceneUpdate);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _updateIntervalSpin = new QDoubleSpinBox(_loggingGroupBox);
        _updateIntervalSpin->setObjectName(QString::fromUtf8("_updateIntervalSpin"));
        _updateIntervalSpin->setSingleStep(0.01);
        _updateIntervalSpin->setValue(0.01);

        horizontalLayout_2->addWidget(_updateIntervalSpin);

        label_4 = new QLabel(_loggingGroupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_2->addWidget(label_4);


        verticalLayout_4->addLayout(horizontalLayout_2);


        verticalLayout_5->addLayout(verticalLayout_4);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_5 = new QLabel(_loggingGroupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_3->addWidget(label_5);

        _timeLbl = new QLabel(_loggingGroupBox);
        _timeLbl->setObjectName(QString::fromUtf8("_timeLbl"));

        horizontalLayout_3->addWidget(_timeLbl);


        verticalLayout_5->addLayout(horizontalLayout_3);


        verticalLayout_3->addWidget(_loggingGroupBox);


        verticalLayout_6->addWidget(groupBox_2);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout_4->addWidget(scrollArea);

        RWSimPlugin->setWidget(dockWidgetContents);
        QWidget::setTabOrder(scrollArea, _openDwcBtn);
        QWidget::setTabOrder(_openDwcBtn, _openLastDwcBtn);
        QWidget::setTabOrder(_openLastDwcBtn, _closeDwcBtn);
        QWidget::setTabOrder(_closeDwcBtn, _editDwcBtn);
        QWidget::setTabOrder(_editDwcBtn, _createSimulatorBtn);
        QWidget::setTabOrder(_createSimulatorBtn, _destroySimulatorBtn);
        QWidget::setTabOrder(_destroySimulatorBtn, _simConfigBtn);
        QWidget::setTabOrder(_simConfigBtn, _startBtn);
        QWidget::setTabOrder(_startBtn, _stepBtn);
        QWidget::setTabOrder(_stepBtn, _stopBtn);
        QWidget::setTabOrder(_stopBtn, _resetBtn);
        QWidget::setTabOrder(_resetBtn, _timeStepSpin);
        QWidget::setTabOrder(_timeStepSpin, _timeScaleSpin);
        QWidget::setTabOrder(_timeScaleSpin, _deviceControlBox);
        QWidget::setTabOrder(_deviceControlBox, _openDeviceCtrlBtn);
        QWidget::setTabOrder(_openDeviceCtrlBtn, _debugLevelSpin);
        QWidget::setTabOrder(_debugLevelSpin, _recordStatePathBox);
        QWidget::setTabOrder(_recordStatePathBox, _saveStatePathBtn);
        QWidget::setTabOrder(_saveStatePathBtn, _forceSceneUpdate);
        QWidget::setTabOrder(_forceSceneUpdate, _updateIntervalSpin);

        retranslateUi(RWSimPlugin);

        QMetaObject::connectSlotsByName(RWSimPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *RWSimPlugin)
    {
        RWSimPlugin->setWindowTitle(QApplication::translate("RWSimPlugin", "RWSim", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("RWSimPlugin", "Dynamic WorkCell", 0, QApplication::UnicodeUTF8));
        _openDwcBtn->setText(QApplication::translate("RWSimPlugin", "Open", 0, QApplication::UnicodeUTF8));
        _openLastDwcBtn->setText(QApplication::translate("RWSimPlugin", "Open last", 0, QApplication::UnicodeUTF8));
        _closeDwcBtn->setText(QApplication::translate("RWSimPlugin", "Close", 0, QApplication::UnicodeUTF8));
        _editDwcBtn->setText(QApplication::translate("RWSimPlugin", "Edit", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("RWSimPlugin", "Simulation", 0, QApplication::UnicodeUTF8));
        _createSimulatorBtn->setText(QApplication::translate("RWSimPlugin", "Create simulator", 0, QApplication::UnicodeUTF8));
        _destroySimulatorBtn->setText(QApplication::translate("RWSimPlugin", "Destroy simulator", 0, QApplication::UnicodeUTF8));
        _simConfigBtn->setText(QApplication::translate("RWSimPlugin", "Simulator configuration", 0, QApplication::UnicodeUTF8));
        _controlGroupBox->setTitle(QApplication::translate("RWSimPlugin", "Control", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _startBtn->setToolTip(QApplication::translate("RWSimPlugin", "Start/resume simulation", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _startBtn->setText(QString());
#ifndef QT_NO_TOOLTIP
        _stepBtn->setToolTip(QApplication::translate("RWSimPlugin", "Step simulation one dt", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _stepBtn->setText(QApplication::translate("RWSimPlugin", "...", 0, QApplication::UnicodeUTF8));
        _stopBtn->setText(QApplication::translate("RWSimPlugin", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _resetBtn->setToolTip(QApplication::translate("RWSimPlugin", "Reset the state of the simulator to \n"
"that of RobWorkStudio", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _resetBtn->setText(QApplication::translate("RWSimPlugin", "...", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("RWSimPlugin", "Time step:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _timeScaleSpin->setToolTip(QApplication::translate("RWSimPlugin", "\"simulation time/Real time\" parameter. Increasing this parameter will make the simulator slower than real time. Decreasing it will make it faster. A value of 1 will make the simulation speed as fast as real time. If 0 the simulation will run as fast as posible. \n"
"NOTE: A wanted simulation speed cannot be guarenteed.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_3->setText(QApplication::translate("RWSimPlugin", "Sim speed:", 0, QApplication::UnicodeUTF8));
        _deviceGroupBox->setTitle(QApplication::translate("RWSimPlugin", "Device", 0, QApplication::UnicodeUTF8));
        _openDeviceCtrlBtn->setText(QApplication::translate("RWSimPlugin", "Open control dialog", 0, QApplication::UnicodeUTF8));
        _tactileSensorBtn->setText(QApplication::translate("RWSimPlugin", "Open Tactile Sensor Dialog", 0, QApplication::UnicodeUTF8));
        _loggingGroupBox->setTitle(QApplication::translate("RWSimPlugin", "Logging", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("RWSimPlugin", "Debug level:", 0, QApplication::UnicodeUTF8));
        _recordStatePathBox->setText(QApplication::translate("RWSimPlugin", "Record state path", 0, QApplication::UnicodeUTF8));
        _saveStatePathBtn->setText(QApplication::translate("RWSimPlugin", "Save state path", 0, QApplication::UnicodeUTF8));
        _forceSceneUpdate->setText(QApplication::translate("RWSimPlugin", "Force scene update", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _updateIntervalSpin->setToolTip(QApplication::translate("RWSimPlugin", "The update interval in seconds", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_4->setText(QApplication::translate("RWSimPlugin", "Update interval", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("RWSimPlugin", "Time:", 0, QApplication::UnicodeUTF8));
        _timeLbl->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class RWSimPlugin: public Ui_RWSimPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RWSIMPLUGIN_H
