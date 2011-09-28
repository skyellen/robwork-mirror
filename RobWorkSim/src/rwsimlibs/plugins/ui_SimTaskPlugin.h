/********************************************************************************
** Form generated from reading UI file 'SimTaskPlugin.ui'
**
** Created: Mon Sep 5 13:14:52 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIMTASKPLUGIN_H
#define UI_SIMTASKPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SimTaskPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QPushButton *_loadTaskBtn;
    QPushButton *_startBtn;
    QPushButton *_stopBtn;
    QPushButton *_saveResultBtn;
    QCheckBox *_genTasksBox;
    QCheckBox *_onlySuccessBox;
    QComboBox *_typeComboBox;
    QComboBox *_objectComboBox;
    QCheckBox *_continuesBox;
    QCheckBox *_mergeResultBox;
    QComboBox *_outputFormatBox;
    QLabel *label_9;
    QGridLayout *gridLayout_2;
    QSpinBox *_nrTaskSpinBox;
    QLabel *label_2;
    QLabel *label_3;
    QSpinBox *_showTaskSpinBox;
    QSpinBox *_delaySpin;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QProgressBar *_progressBar;
    QFormLayout *formLayout;
    QLabel *label_5;
    QLabel *_simTimeLbl;
    QLabel *label_7;
    QLabel *_wallTimeLbl;
    QLabel *label_6;
    QLabel *_timePerGraspLbl;
    QLabel *label_8;
    QLabel *_timeToFinishLbl;
    QGroupBox *_configGroupBox;
    QPushButton *_updateConfigBtn;
    QSpacerItem *verticalSpacer;

    void setupUi(QDockWidget *SimTaskPlugin)
    {
        if (SimTaskPlugin->objectName().isEmpty())
            SimTaskPlugin->setObjectName(QString::fromUtf8("SimTaskPlugin"));
        SimTaskPlugin->resize(275, 577);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _loadTaskBtn = new QPushButton(dockWidgetContents);
        _loadTaskBtn->setObjectName(QString::fromUtf8("_loadTaskBtn"));

        gridLayout->addWidget(_loadTaskBtn, 5, 0, 1, 1);

        _startBtn = new QPushButton(dockWidgetContents);
        _startBtn->setObjectName(QString::fromUtf8("_startBtn"));

        gridLayout->addWidget(_startBtn, 7, 0, 1, 1);

        _stopBtn = new QPushButton(dockWidgetContents);
        _stopBtn->setObjectName(QString::fromUtf8("_stopBtn"));

        gridLayout->addWidget(_stopBtn, 7, 1, 1, 1);

        _saveResultBtn = new QPushButton(dockWidgetContents);
        _saveResultBtn->setObjectName(QString::fromUtf8("_saveResultBtn"));

        gridLayout->addWidget(_saveResultBtn, 5, 1, 1, 1);

        _genTasksBox = new QCheckBox(dockWidgetContents);
        _genTasksBox->setObjectName(QString::fromUtf8("_genTasksBox"));

        gridLayout->addWidget(_genTasksBox, 0, 0, 1, 1);

        _onlySuccessBox = new QCheckBox(dockWidgetContents);
        _onlySuccessBox->setObjectName(QString::fromUtf8("_onlySuccessBox"));

        gridLayout->addWidget(_onlySuccessBox, 0, 1, 1, 1);

        _typeComboBox = new QComboBox(dockWidgetContents);
        _typeComboBox->setObjectName(QString::fromUtf8("_typeComboBox"));

        gridLayout->addWidget(_typeComboBox, 2, 1, 1, 1);

        _objectComboBox = new QComboBox(dockWidgetContents);
        _objectComboBox->setObjectName(QString::fromUtf8("_objectComboBox"));

        gridLayout->addWidget(_objectComboBox, 3, 1, 1, 1);

        _continuesBox = new QCheckBox(dockWidgetContents);
        _continuesBox->setObjectName(QString::fromUtf8("_continuesBox"));

        gridLayout->addWidget(_continuesBox, 1, 1, 1, 1);

        _mergeResultBox = new QCheckBox(dockWidgetContents);
        _mergeResultBox->setObjectName(QString::fromUtf8("_mergeResultBox"));

        gridLayout->addWidget(_mergeResultBox, 1, 0, 1, 1);

        _outputFormatBox = new QComboBox(dockWidgetContents);
        _outputFormatBox->setObjectName(QString::fromUtf8("_outputFormatBox"));

        gridLayout->addWidget(_outputFormatBox, 4, 1, 1, 1);

        label_9 = new QLabel(dockWidgetContents);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout->addWidget(label_9, 4, 0, 1, 1);


        verticalLayout->addLayout(gridLayout);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _nrTaskSpinBox = new QSpinBox(dockWidgetContents);
        _nrTaskSpinBox->setObjectName(QString::fromUtf8("_nrTaskSpinBox"));
        _nrTaskSpinBox->setReadOnly(true);

        gridLayout_2->addWidget(_nrTaskSpinBox, 0, 1, 1, 1);

        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 0, 0, 1, 1);

        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);

        _showTaskSpinBox = new QSpinBox(dockWidgetContents);
        _showTaskSpinBox->setObjectName(QString::fromUtf8("_showTaskSpinBox"));

        gridLayout_2->addWidget(_showTaskSpinBox, 1, 1, 1, 1);

        _delaySpin = new QSpinBox(dockWidgetContents);
        _delaySpin->setObjectName(QString::fromUtf8("_delaySpin"));
        _delaySpin->setMinimum(-1);
        _delaySpin->setMaximum(1000);

        gridLayout_2->addWidget(_delaySpin, 2, 1, 1, 1);

        label_4 = new QLabel(dockWidgetContents);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 2, 0, 1, 1);


        verticalLayout->addLayout(gridLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));

        verticalLayout->addLayout(horizontalLayout);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        _progressBar = new QProgressBar(dockWidgetContents);
        _progressBar->setObjectName(QString::fromUtf8("_progressBar"));
        _progressBar->setValue(0);

        verticalLayout->addWidget(_progressBar);

        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label_5 = new QLabel(dockWidgetContents);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_5);

        _simTimeLbl = new QLabel(dockWidgetContents);
        _simTimeLbl->setObjectName(QString::fromUtf8("_simTimeLbl"));
        _simTimeLbl->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(0, QFormLayout::FieldRole, _simTimeLbl);

        label_7 = new QLabel(dockWidgetContents);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_7);

        _wallTimeLbl = new QLabel(dockWidgetContents);
        _wallTimeLbl->setObjectName(QString::fromUtf8("_wallTimeLbl"));
        _wallTimeLbl->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(1, QFormLayout::FieldRole, _wallTimeLbl);

        label_6 = new QLabel(dockWidgetContents);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_6);

        _timePerGraspLbl = new QLabel(dockWidgetContents);
        _timePerGraspLbl->setObjectName(QString::fromUtf8("_timePerGraspLbl"));
        _timePerGraspLbl->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(2, QFormLayout::FieldRole, _timePerGraspLbl);

        label_8 = new QLabel(dockWidgetContents);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_8);

        _timeToFinishLbl = new QLabel(dockWidgetContents);
        _timeToFinishLbl->setObjectName(QString::fromUtf8("_timeToFinishLbl"));
        _timeToFinishLbl->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        formLayout->setWidget(3, QFormLayout::FieldRole, _timeToFinishLbl);


        verticalLayout->addLayout(formLayout);

        _configGroupBox = new QGroupBox(dockWidgetContents);
        _configGroupBox->setObjectName(QString::fromUtf8("_configGroupBox"));
        _configGroupBox->setCheckable(false);

        verticalLayout->addWidget(_configGroupBox);

        _updateConfigBtn = new QPushButton(dockWidgetContents);
        _updateConfigBtn->setObjectName(QString::fromUtf8("_updateConfigBtn"));

        verticalLayout->addWidget(_updateConfigBtn);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        verticalLayout_2->addLayout(verticalLayout);

        SimTaskPlugin->setWidget(dockWidgetContents);

        retranslateUi(SimTaskPlugin);

        QMetaObject::connectSlotsByName(SimTaskPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SimTaskPlugin)
    {
        SimTaskPlugin->setWindowTitle(QApplication::translate("SimTaskPlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        _loadTaskBtn->setText(QApplication::translate("SimTaskPlugin", "Load tasks", 0, QApplication::UnicodeUTF8));
        _startBtn->setText(QApplication::translate("SimTaskPlugin", "Start", 0, QApplication::UnicodeUTF8));
        _stopBtn->setText(QApplication::translate("SimTaskPlugin", "Stop", 0, QApplication::UnicodeUTF8));
        _saveResultBtn->setText(QApplication::translate("SimTaskPlugin", "Save results", 0, QApplication::UnicodeUTF8));
        _genTasksBox->setText(QApplication::translate("SimTaskPlugin", "Generate Tasks", 0, QApplication::UnicodeUTF8));
        _onlySuccessBox->setText(QApplication::translate("SimTaskPlugin", "Only success", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _continuesBox->setToolTip(QApplication::translate("SimTaskPlugin", "If generate tasks is enabled and continues is enabled then the simuator will continue to run and generate new tasks.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _continuesBox->setText(QApplication::translate("SimTaskPlugin", "Continues", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _mergeResultBox->setToolTip(QApplication::translate("SimTaskPlugin", "Merge result into one file before saving. Else multiple files will be used when continues is enabled.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _mergeResultBox->setText(QApplication::translate("SimTaskPlugin", "Merge result", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("SimTaskPlugin", "Output format", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SimTaskPlugin", "Nr of tasks", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SimTaskPlugin", "Show Task", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SimTaskPlugin", "Delay in ms", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SimTaskPlugin", "Progress:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SimTaskPlugin", "Simulated time:", 0, QApplication::UnicodeUTF8));
        _simTimeLbl->setText(QApplication::translate("SimTaskPlugin", "-", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SimTaskPlugin", "Wall time:", 0, QApplication::UnicodeUTF8));
        _wallTimeLbl->setText(QApplication::translate("SimTaskPlugin", "-", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SimTaskPlugin", "Time per Grasp:", 0, QApplication::UnicodeUTF8));
        _timePerGraspLbl->setText(QApplication::translate("SimTaskPlugin", "-", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SimTaskPlugin", "Remaining time:", 0, QApplication::UnicodeUTF8));
        _timeToFinishLbl->setText(QApplication::translate("SimTaskPlugin", "-", 0, QApplication::UnicodeUTF8));
        _configGroupBox->setTitle(QApplication::translate("SimTaskPlugin", "Config", 0, QApplication::UnicodeUTF8));
        _updateConfigBtn->setText(QApplication::translate("SimTaskPlugin", "Update configuration", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SimTaskPlugin: public Ui_SimTaskPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIMTASKPLUGIN_H
