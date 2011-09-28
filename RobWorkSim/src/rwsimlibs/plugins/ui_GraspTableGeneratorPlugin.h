/********************************************************************************
** Form generated from reading UI file 'GraspTableGeneratorPlugin.ui'
**
** Created: Mon Aug 1 12:19:15 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRASPTABLEGENERATORPLUGIN_H
#define UI_GRASPTABLEGENERATORPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GraspTableGeneratorPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_5;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_9;
    QGridLayout *gridLayout_3;
    QLabel *label_62;
    QSpinBox *_tableSizeSpin;
    QLabel *label_63;
    QSpinBox *_groupSizeSpin;
    QCheckBox *_collisionFreeInitBox;
    QCheckBox *_dynamicSimulationBox;
    QCheckBox *_continuesLoggingBox;
    QLabel *label_65;
    QComboBox *_deviceBox;
    QLabel *label_66;
    QComboBox *_objectBox;
    QCheckBox *_fixedGroupBox;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_29;
    QSpacerItem *horizontalSpacer;
    QComboBox *_gPolicyBox;
    QPushButton *_propGPolicyBtn;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_30;
    QSpacerItem *horizontalSpacer_2;
    QComboBox *_gStrategyBox;
    QPushButton *_propGStrategyBtn;
    QGroupBox *groupBox_6;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_31;
    QSpacerItem *horizontalSpacer_3;
    QComboBox *_qAvailMetricsBox;
    QPushButton *_addBtn;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_3;
    QSpacerItem *horizontalSpacer_5;
    QComboBox *_qMetricsBox;
    QPushButton *_removeBtn;
    QFrame *line;
    QGroupBox *groupBox_8;
    QVBoxLayout *verticalLayout_15;
    QPlainTextEdit *_descriptionEdit;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_14;
    QLabel *label_64;
    QLineEdit *_outputFileEdit;
    QPushButton *_browseOutputBtn;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_10;
    QScrollArea *scrollArea_2;
    QWidget *scrollAreaWidgetContents_2;
    QVBoxLayout *verticalLayout_11;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_4;
    QComboBox *_simulatorBox;
    QPushButton *_simulatorConfigBtn;
    QGroupBox *groupBox_7;
    QHBoxLayout *horizontalLayout_10;
    QFormLayout *formLayout_2;
    QLabel *label_61;
    QSpinBox *_nrOfThreadsSpin;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_12;
    QFormLayout *formLayout;
    QLabel *label_11;
    QDoubleSpinBox *_linVelSpin;
    QLabel *label_12;
    QDoubleSpinBox *_angVelSpin;
    QLabel *label_13;
    QDoubleSpinBox *_linAccSpin;
    QLabel *label_14;
    QDoubleSpinBox *_angAccSpin;
    QLabel *label_15;
    QDoubleSpinBox *_minRestTimeSpin;
    QLabel *label_16;
    QDoubleSpinBox *_maxRunningTimeSpin;
    QLabel *label_17;
    QDoubleSpinBox *_minTimeValidSpin;
    QLabel *label_28;
    QSpinBox *_backupIntervalSpin;
    QSpacerItem *verticalSpacer_4;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_14;
    QCheckBox *_forceUpdateCheck_3;
    QCheckBox *_onlyShowRestBox_3;
    QCheckBox *checkBox_5;
    QSpacerItem *verticalSpacer_3;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_6;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_4;
    QSpinBox *_updateRateSpin;
    QCheckBox *_forceUpdateCheck;
    QPushButton *_simulatorBtn;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QLabel *label_22;
    QLineEdit *_savePath;
    QPushButton *_browseBtn;
    QCheckBox *_onlyShowRestBox;
    QCheckBox *_saveMultipleCheck;
    QGroupBox *groupBox_5;
    QLabel *label_5;
    QSpinBox *_lowRollSpin;
    QSpinBox *_highRollSpin;
    QSpinBox *_highPitchSpin;
    QSpinBox *_lowPitchSpin;
    QSpinBox *_lowYawSpin;
    QSpinBox *_highYawSpin;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QDoubleSpinBox *_xSpinBox;
    QDoubleSpinBox *_ySpinBox;
    QDoubleSpinBox *_zSpinBox;
    QCheckBox *_colFreeStart;
    QCheckBox *_continuesLogging;
    QSpinBox *_logIntervalSpin;
    QLabel *label_27;
    QLineEdit *_descBox;
    QGroupBox *groupBox_3;
    QComboBox *_graspPolicyBox1;
    QLabel *label_23;
    QLabel *label_24;
    QComboBox *_preshapeStratBox1;
    QComboBox *_deviceBox1;
    QLabel *label_25;
    QComboBox *_objectBox1;
    QLabel *label_26;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *groupBox;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout;
    QLabel *label_18;
    QLabel *_testsLeftLbl;
    QLabel *label_2;
    QLabel *_avgTimePerTestLbl;
    QLabel *label;
    QLabel *_estimatedTimeLeftLbl;
    QLabel *label_21;
    QLabel *_simSpeedLbl;
    QLabel *label_19;
    QLabel *_avgSimTimeLbl;
    QLabel *label_20;
    QLabel *_simsPerSecLbl;
    QProgressBar *_simProgress;
    QVBoxLayout *verticalLayout_4;
    QPushButton *_resetBtn;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer;
    QPushButton *_scapeBtn;
    QPushButton *_saveBtn1;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *_genTableBtn;
    QSpacerItem *horizontalSpacer_4;
    QPushButton *_loadGTableConfig;
    QPushButton *_saveGTableConfig;
    QGroupBox *groupBox_13;
    QVBoxLayout *verticalLayout_13;
    QLabel *_statusLbl;
    QHBoxLayout *horizontalLayout_9;
    QProgressBar *progressBar;
    QPushButton *_stopBtn;

    void setupUi(QDockWidget *GraspTableGeneratorPlugin)
    {
        if (GraspTableGeneratorPlugin->objectName().isEmpty())
            GraspTableGeneratorPlugin->setObjectName(QString::fromUtf8("GraspTableGeneratorPlugin"));
        GraspTableGeneratorPlugin->resize(544, 583);
        GraspTableGeneratorPlugin->setAutoFillBackground(false);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_5 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        tabWidget = new QTabWidget(dockWidgetContents);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_9 = new QVBoxLayout(tab);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        label_62 = new QLabel(tab);
        label_62->setObjectName(QString::fromUtf8("label_62"));

        gridLayout_3->addWidget(label_62, 0, 0, 1, 1);

        _tableSizeSpin = new QSpinBox(tab);
        _tableSizeSpin->setObjectName(QString::fromUtf8("_tableSizeSpin"));
        _tableSizeSpin->setMaximum(100000);
        _tableSizeSpin->setValue(10000);

        gridLayout_3->addWidget(_tableSizeSpin, 0, 1, 1, 1);

        label_63 = new QLabel(tab);
        label_63->setObjectName(QString::fromUtf8("label_63"));

        gridLayout_3->addWidget(label_63, 0, 2, 1, 1);

        _groupSizeSpin = new QSpinBox(tab);
        _groupSizeSpin->setObjectName(QString::fromUtf8("_groupSizeSpin"));
        _groupSizeSpin->setValue(1);

        gridLayout_3->addWidget(_groupSizeSpin, 0, 3, 1, 1);

        _collisionFreeInitBox = new QCheckBox(tab);
        _collisionFreeInitBox->setObjectName(QString::fromUtf8("_collisionFreeInitBox"));
        _collisionFreeInitBox->setChecked(true);

        gridLayout_3->addWidget(_collisionFreeInitBox, 2, 0, 1, 1);

        _dynamicSimulationBox = new QCheckBox(tab);
        _dynamicSimulationBox->setObjectName(QString::fromUtf8("_dynamicSimulationBox"));
        _dynamicSimulationBox->setChecked(true);

        gridLayout_3->addWidget(_dynamicSimulationBox, 2, 1, 1, 1);

        _continuesLoggingBox = new QCheckBox(tab);
        _continuesLoggingBox->setObjectName(QString::fromUtf8("_continuesLoggingBox"));

        gridLayout_3->addWidget(_continuesLoggingBox, 2, 2, 1, 1);

        label_65 = new QLabel(tab);
        label_65->setObjectName(QString::fromUtf8("label_65"));

        gridLayout_3->addWidget(label_65, 1, 0, 1, 1);

        _deviceBox = new QComboBox(tab);
        _deviceBox->setObjectName(QString::fromUtf8("_deviceBox"));

        gridLayout_3->addWidget(_deviceBox, 1, 1, 1, 1);

        label_66 = new QLabel(tab);
        label_66->setObjectName(QString::fromUtf8("label_66"));

        gridLayout_3->addWidget(label_66, 1, 2, 1, 1);

        _objectBox = new QComboBox(tab);
        _objectBox->setObjectName(QString::fromUtf8("_objectBox"));

        gridLayout_3->addWidget(_objectBox, 1, 3, 1, 1);

        _fixedGroupBox = new QCheckBox(tab);
        _fixedGroupBox->setObjectName(QString::fromUtf8("_fixedGroupBox"));

        gridLayout_3->addWidget(_fixedGroupBox, 2, 3, 1, 1);


        verticalLayout_9->addLayout(gridLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_29 = new QLabel(tab);
        label_29->setObjectName(QString::fromUtf8("label_29"));

        horizontalLayout_4->addWidget(label_29);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);

        _gPolicyBox = new QComboBox(tab);
        _gPolicyBox->setObjectName(QString::fromUtf8("_gPolicyBox"));
        _gPolicyBox->setMinimumSize(QSize(200, 0));

        horizontalLayout_4->addWidget(_gPolicyBox);

        _propGPolicyBtn = new QPushButton(tab);
        _propGPolicyBtn->setObjectName(QString::fromUtf8("_propGPolicyBtn"));

        horizontalLayout_4->addWidget(_propGPolicyBtn);


        verticalLayout_9->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_30 = new QLabel(tab);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        horizontalLayout_5->addWidget(label_30);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);

        _gStrategyBox = new QComboBox(tab);
        _gStrategyBox->setObjectName(QString::fromUtf8("_gStrategyBox"));
        _gStrategyBox->setMinimumSize(QSize(200, 0));

        horizontalLayout_5->addWidget(_gStrategyBox);

        _propGStrategyBtn = new QPushButton(tab);
        _propGStrategyBtn->setObjectName(QString::fromUtf8("_propGStrategyBtn"));

        horizontalLayout_5->addWidget(_propGStrategyBtn);


        verticalLayout_9->addLayout(horizontalLayout_5);

        groupBox_6 = new QGroupBox(tab);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        verticalLayout_8 = new QVBoxLayout(groupBox_6);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_31 = new QLabel(groupBox_6);
        label_31->setObjectName(QString::fromUtf8("label_31"));

        horizontalLayout_6->addWidget(label_31);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer_3);

        _qAvailMetricsBox = new QComboBox(groupBox_6);
        _qAvailMetricsBox->setObjectName(QString::fromUtf8("_qAvailMetricsBox"));
        _qAvailMetricsBox->setMinimumSize(QSize(200, 0));

        horizontalLayout_6->addWidget(_qAvailMetricsBox);

        _addBtn = new QPushButton(groupBox_6);
        _addBtn->setObjectName(QString::fromUtf8("_addBtn"));

        horizontalLayout_6->addWidget(_addBtn);


        verticalLayout_8->addLayout(horizontalLayout_6);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_3 = new QLabel(groupBox_6);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_11->addWidget(label_3);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_11->addItem(horizontalSpacer_5);

        _qMetricsBox = new QComboBox(groupBox_6);
        _qMetricsBox->setObjectName(QString::fromUtf8("_qMetricsBox"));
        _qMetricsBox->setMinimumSize(QSize(200, 0));

        horizontalLayout_11->addWidget(_qMetricsBox);

        _removeBtn = new QPushButton(groupBox_6);
        _removeBtn->setObjectName(QString::fromUtf8("_removeBtn"));

        horizontalLayout_11->addWidget(_removeBtn);


        verticalLayout_8->addLayout(horizontalLayout_11);


        verticalLayout_9->addWidget(groupBox_6);

        line = new QFrame(tab);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_9->addWidget(line);

        groupBox_8 = new QGroupBox(tab);
        groupBox_8->setObjectName(QString::fromUtf8("groupBox_8"));
        verticalLayout_15 = new QVBoxLayout(groupBox_8);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));
        _descriptionEdit = new QPlainTextEdit(groupBox_8);
        _descriptionEdit->setObjectName(QString::fromUtf8("_descriptionEdit"));

        verticalLayout_15->addWidget(_descriptionEdit);


        verticalLayout_9->addWidget(groupBox_8);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_2);

        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        label_64 = new QLabel(tab);
        label_64->setObjectName(QString::fromUtf8("label_64"));

        horizontalLayout_14->addWidget(label_64);

        _outputFileEdit = new QLineEdit(tab);
        _outputFileEdit->setObjectName(QString::fromUtf8("_outputFileEdit"));

        horizontalLayout_14->addWidget(_outputFileEdit);

        _browseOutputBtn = new QPushButton(tab);
        _browseOutputBtn->setObjectName(QString::fromUtf8("_browseOutputBtn"));

        horizontalLayout_14->addWidget(_browseOutputBtn);


        verticalLayout_9->addLayout(horizontalLayout_14);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_10 = new QVBoxLayout(tab_3);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        scrollArea_2 = new QScrollArea(tab_3);
        scrollArea_2->setObjectName(QString::fromUtf8("scrollArea_2"));
        scrollArea_2->setFrameShape(QFrame::NoFrame);
        scrollArea_2->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QString::fromUtf8("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, 0, 502, 385));
        verticalLayout_11 = new QVBoxLayout(scrollAreaWidgetContents_2);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        label_4 = new QLabel(scrollAreaWidgetContents_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_7->addWidget(label_4);

        _simulatorBox = new QComboBox(scrollAreaWidgetContents_2);
        _simulatorBox->setObjectName(QString::fromUtf8("_simulatorBox"));

        horizontalLayout_7->addWidget(_simulatorBox);

        _simulatorConfigBtn = new QPushButton(scrollAreaWidgetContents_2);
        _simulatorConfigBtn->setObjectName(QString::fromUtf8("_simulatorConfigBtn"));
        _simulatorConfigBtn->setMaximumSize(QSize(80, 16777215));

        horizontalLayout_7->addWidget(_simulatorConfigBtn);


        verticalLayout_11->addLayout(horizontalLayout_7);

        groupBox_7 = new QGroupBox(scrollAreaWidgetContents_2);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        horizontalLayout_10 = new QHBoxLayout(groupBox_7);
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        label_61 = new QLabel(groupBox_7);
        label_61->setObjectName(QString::fromUtf8("label_61"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_61);

        _nrOfThreadsSpin = new QSpinBox(groupBox_7);
        _nrOfThreadsSpin->setObjectName(QString::fromUtf8("_nrOfThreadsSpin"));
        _nrOfThreadsSpin->setMinimum(1);

        formLayout_2->setWidget(0, QFormLayout::FieldRole, _nrOfThreadsSpin);


        horizontalLayout_10->addLayout(formLayout_2);


        verticalLayout_11->addWidget(groupBox_7);

        groupBox_2 = new QGroupBox(scrollAreaWidgetContents_2);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(201, 0));
        verticalLayout_12 = new QVBoxLayout(groupBox_2);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label_11 = new QLabel(groupBox_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_11);

        _linVelSpin = new QDoubleSpinBox(groupBox_2);
        _linVelSpin->setObjectName(QString::fromUtf8("_linVelSpin"));
        _linVelSpin->setDecimals(3);
        _linVelSpin->setMinimum(0.001);
        _linVelSpin->setMaximum(1);
        _linVelSpin->setSingleStep(0.01);
        _linVelSpin->setValue(0.05);

        formLayout->setWidget(0, QFormLayout::FieldRole, _linVelSpin);

        label_12 = new QLabel(groupBox_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_12);

        _angVelSpin = new QDoubleSpinBox(groupBox_2);
        _angVelSpin->setObjectName(QString::fromUtf8("_angVelSpin"));
        _angVelSpin->setDecimals(3);
        _angVelSpin->setMinimum(0.001);
        _angVelSpin->setMaximum(1);
        _angVelSpin->setSingleStep(0.01);
        _angVelSpin->setValue(0.05);

        formLayout->setWidget(1, QFormLayout::FieldRole, _angVelSpin);

        label_13 = new QLabel(groupBox_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_13);

        _linAccSpin = new QDoubleSpinBox(groupBox_2);
        _linAccSpin->setObjectName(QString::fromUtf8("_linAccSpin"));
        _linAccSpin->setDecimals(3);
        _linAccSpin->setMinimum(0);
        _linAccSpin->setMaximum(1);
        _linAccSpin->setSingleStep(1e-06);
        _linAccSpin->setValue(0);

        formLayout->setWidget(2, QFormLayout::FieldRole, _linAccSpin);

        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_14);

        _angAccSpin = new QDoubleSpinBox(groupBox_2);
        _angAccSpin->setObjectName(QString::fromUtf8("_angAccSpin"));
        _angAccSpin->setDecimals(3);
        _angAccSpin->setMinimum(0);
        _angAccSpin->setMaximum(1);
        _angAccSpin->setSingleStep(1e-06);
        _angAccSpin->setValue(0);

        formLayout->setWidget(3, QFormLayout::FieldRole, _angAccSpin);

        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_15);

        _minRestTimeSpin = new QDoubleSpinBox(groupBox_2);
        _minRestTimeSpin->setObjectName(QString::fromUtf8("_minRestTimeSpin"));
        _minRestTimeSpin->setSingleStep(0.01);
        _minRestTimeSpin->setValue(0.4);

        formLayout->setWidget(4, QFormLayout::FieldRole, _minRestTimeSpin);

        label_16 = new QLabel(groupBox_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        formLayout->setWidget(5, QFormLayout::LabelRole, label_16);

        _maxRunningTimeSpin = new QDoubleSpinBox(groupBox_2);
        _maxRunningTimeSpin->setObjectName(QString::fromUtf8("_maxRunningTimeSpin"));
        _maxRunningTimeSpin->setMaximum(999.99);
        _maxRunningTimeSpin->setValue(50);

        formLayout->setWidget(5, QFormLayout::FieldRole, _maxRunningTimeSpin);

        label_17 = new QLabel(groupBox_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        formLayout->setWidget(6, QFormLayout::LabelRole, label_17);

        _minTimeValidSpin = new QDoubleSpinBox(groupBox_2);
        _minTimeValidSpin->setObjectName(QString::fromUtf8("_minTimeValidSpin"));
        _minTimeValidSpin->setMinimum(0.1);

        formLayout->setWidget(6, QFormLayout::FieldRole, _minTimeValidSpin);

        label_28 = new QLabel(groupBox_2);
        label_28->setObjectName(QString::fromUtf8("label_28"));

        formLayout->setWidget(7, QFormLayout::LabelRole, label_28);

        _backupIntervalSpin = new QSpinBox(groupBox_2);
        _backupIntervalSpin->setObjectName(QString::fromUtf8("_backupIntervalSpin"));
        _backupIntervalSpin->setMaximum(999999);
        _backupIntervalSpin->setSingleStep(100);
        _backupIntervalSpin->setValue(1000);

        formLayout->setWidget(7, QFormLayout::FieldRole, _backupIntervalSpin);


        verticalLayout_12->addLayout(formLayout);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_12->addItem(verticalSpacer_4);


        verticalLayout_11->addWidget(groupBox_2);

        scrollArea_2->setWidget(scrollAreaWidgetContents_2);

        verticalLayout_10->addWidget(scrollArea_2);

        tabWidget->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        verticalLayout_14 = new QVBoxLayout(tab_4);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        _forceUpdateCheck_3 = new QCheckBox(tab_4);
        _forceUpdateCheck_3->setObjectName(QString::fromUtf8("_forceUpdateCheck_3"));
        _forceUpdateCheck_3->setChecked(true);

        verticalLayout_14->addWidget(_forceUpdateCheck_3);

        _onlyShowRestBox_3 = new QCheckBox(tab_4);
        _onlyShowRestBox_3->setObjectName(QString::fromUtf8("_onlyShowRestBox_3"));

        verticalLayout_14->addWidget(_onlyShowRestBox_3);

        checkBox_5 = new QCheckBox(tab_4);
        checkBox_5->setObjectName(QString::fromUtf8("checkBox_5"));

        verticalLayout_14->addWidget(checkBox_5);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_14->addItem(verticalSpacer_3);

        tabWidget->addTab(tab_4, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_6 = new QVBoxLayout(tab_2);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        scrollArea = new QScrollArea(tab_2);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 483, 624));
        verticalLayout_7 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox_4 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setMinimumSize(QSize(280, 171));
        _updateRateSpin = new QSpinBox(groupBox_4);
        _updateRateSpin->setObjectName(QString::fromUtf8("_updateRateSpin"));
        _updateRateSpin->setGeometry(QRect(140, 50, 46, 22));
        _updateRateSpin->setMinimum(10);
        _updateRateSpin->setMaximum(10000);
        _updateRateSpin->setValue(100);
        _forceUpdateCheck = new QCheckBox(groupBox_4);
        _forceUpdateCheck->setObjectName(QString::fromUtf8("_forceUpdateCheck"));
        _forceUpdateCheck->setGeometry(QRect(10, 50, 131, 21));
        _forceUpdateCheck->setChecked(true);
        _simulatorBtn = new QPushButton(groupBox_4);
        _simulatorBtn->setObjectName(QString::fromUtf8("_simulatorBtn"));
        _simulatorBtn->setGeometry(QRect(180, 20, 75, 24));
        checkBox = new QCheckBox(groupBox_4);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(10, 20, 131, 21));
        checkBox_2 = new QCheckBox(groupBox_4);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(10, 90, 161, 21));
        label_22 = new QLabel(groupBox_4);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(10, 110, 141, 16));
        _savePath = new QLineEdit(groupBox_4);
        _savePath->setObjectName(QString::fromUtf8("_savePath"));
        _savePath->setGeometry(QRect(10, 130, 161, 21));
        _browseBtn = new QPushButton(groupBox_4);
        _browseBtn->setObjectName(QString::fromUtf8("_browseBtn"));
        _browseBtn->setGeometry(QRect(180, 130, 77, 26));
        _onlyShowRestBox = new QCheckBox(groupBox_4);
        _onlyShowRestBox->setObjectName(QString::fromUtf8("_onlyShowRestBox"));
        _onlyShowRestBox->setGeometry(QRect(10, 70, 191, 19));
        _saveMultipleCheck = new QCheckBox(groupBox_4);
        _saveMultipleCheck->setObjectName(QString::fromUtf8("_saveMultipleCheck"));
        _saveMultipleCheck->setGeometry(QRect(10, 150, 171, 19));

        verticalLayout->addWidget(groupBox_4);

        groupBox_5 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setMinimumSize(QSize(280, 170));
        label_5 = new QLabel(groupBox_5);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 30, 31, 21));
        _lowRollSpin = new QSpinBox(groupBox_5);
        _lowRollSpin->setObjectName(QString::fromUtf8("_lowRollSpin"));
        _lowRollSpin->setGeometry(QRect(50, 30, 46, 22));
        _lowRollSpin->setMinimum(-180);
        _lowRollSpin->setMaximum(180);
        _lowRollSpin->setValue(-180);
        _highRollSpin = new QSpinBox(groupBox_5);
        _highRollSpin->setObjectName(QString::fromUtf8("_highRollSpin"));
        _highRollSpin->setGeometry(QRect(100, 30, 46, 22));
        _highRollSpin->setMinimum(-180);
        _highRollSpin->setMaximum(180);
        _highRollSpin->setValue(180);
        _highPitchSpin = new QSpinBox(groupBox_5);
        _highPitchSpin->setObjectName(QString::fromUtf8("_highPitchSpin"));
        _highPitchSpin->setGeometry(QRect(100, 60, 46, 22));
        _highPitchSpin->setMinimum(-180);
        _highPitchSpin->setMaximum(180);
        _highPitchSpin->setValue(180);
        _lowPitchSpin = new QSpinBox(groupBox_5);
        _lowPitchSpin->setObjectName(QString::fromUtf8("_lowPitchSpin"));
        _lowPitchSpin->setGeometry(QRect(50, 60, 46, 22));
        _lowPitchSpin->setMinimum(-180);
        _lowPitchSpin->setMaximum(180);
        _lowPitchSpin->setValue(-180);
        _lowYawSpin = new QSpinBox(groupBox_5);
        _lowYawSpin->setObjectName(QString::fromUtf8("_lowYawSpin"));
        _lowYawSpin->setGeometry(QRect(50, 90, 46, 22));
        _lowYawSpin->setMinimum(-180);
        _lowYawSpin->setMaximum(180);
        _lowYawSpin->setValue(-180);
        _highYawSpin = new QSpinBox(groupBox_5);
        _highYawSpin->setObjectName(QString::fromUtf8("_highYawSpin"));
        _highYawSpin->setGeometry(QRect(100, 90, 46, 22));
        _highYawSpin->setMinimum(-180);
        _highYawSpin->setMaximum(180);
        _highYawSpin->setValue(180);
        label_6 = new QLabel(groupBox_5);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 60, 46, 21));
        label_7 = new QLabel(groupBox_5);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 90, 46, 21));
        label_8 = new QLabel(groupBox_5);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(170, 30, 16, 21));
        label_9 = new QLabel(groupBox_5);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(170, 60, 16, 21));
        label_10 = new QLabel(groupBox_5);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(170, 90, 16, 21));
        _xSpinBox = new QDoubleSpinBox(groupBox_5);
        _xSpinBox->setObjectName(QString::fromUtf8("_xSpinBox"));
        _xSpinBox->setGeometry(QRect(190, 30, 62, 22));
        _xSpinBox->setMinimum(0);
        _xSpinBox->setValue(0.03);
        _ySpinBox = new QDoubleSpinBox(groupBox_5);
        _ySpinBox->setObjectName(QString::fromUtf8("_ySpinBox"));
        _ySpinBox->setGeometry(QRect(190, 60, 62, 22));
        _ySpinBox->setValue(0.03);
        _zSpinBox = new QDoubleSpinBox(groupBox_5);
        _zSpinBox->setObjectName(QString::fromUtf8("_zSpinBox"));
        _zSpinBox->setGeometry(QRect(190, 90, 62, 22));
        _zSpinBox->setValue(0.03);
        _colFreeStart = new QCheckBox(groupBox_5);
        _colFreeStart->setObjectName(QString::fromUtf8("_colFreeStart"));
        _colFreeStart->setGeometry(QRect(10, 120, 131, 21));
        _colFreeStart->setChecked(true);
        _continuesLogging = new QCheckBox(groupBox_5);
        _continuesLogging->setObjectName(QString::fromUtf8("_continuesLogging"));
        _continuesLogging->setEnabled(true);
        _continuesLogging->setGeometry(QRect(10, 140, 141, 21));
        _continuesLogging->setChecked(true);
        _logIntervalSpin = new QSpinBox(groupBox_5);
        _logIntervalSpin->setObjectName(QString::fromUtf8("_logIntervalSpin"));
        _logIntervalSpin->setGeometry(QRect(220, 140, 42, 22));
        _logIntervalSpin->setMinimum(1);
        _logIntervalSpin->setMaximum(1000);
        _logIntervalSpin->setValue(10);
        label_27 = new QLabel(groupBox_5);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(140, 140, 81, 16));

        verticalLayout->addWidget(groupBox_5);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_7->addLayout(horizontalLayout);

        _descBox = new QLineEdit(scrollAreaWidgetContents);
        _descBox->setObjectName(QString::fromUtf8("_descBox"));

        verticalLayout_7->addWidget(_descBox);

        groupBox_3 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setMinimumSize(QSize(100, 100));
        _graspPolicyBox1 = new QComboBox(groupBox_3);
        _graspPolicyBox1->setObjectName(QString::fromUtf8("_graspPolicyBox1"));
        _graspPolicyBox1->setGeometry(QRect(150, 20, 101, 22));
        label_23 = new QLabel(groupBox_3);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(20, 20, 71, 16));
        label_24 = new QLabel(groupBox_3);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(20, 60, 161, 16));
        _preshapeStratBox1 = new QComboBox(groupBox_3);
        _preshapeStratBox1->setObjectName(QString::fromUtf8("_preshapeStratBox1"));
        _preshapeStratBox1->setGeometry(QRect(150, 60, 101, 22));
        _deviceBox1 = new QComboBox(groupBox_3);
        _deviceBox1->setObjectName(QString::fromUtf8("_deviceBox1"));
        _deviceBox1->setGeometry(QRect(403, 20, 111, 22));
        label_25 = new QLabel(groupBox_3);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(360, 20, 46, 20));
        _objectBox1 = new QComboBox(groupBox_3);
        _objectBox1->setObjectName(QString::fromUtf8("_objectBox1"));
        _objectBox1->setGeometry(QRect(403, 60, 111, 22));
        label_26 = new QLabel(groupBox_3);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setGeometry(QRect(360, 60, 46, 20));
        label_23->raise();
        label_24->raise();
        _preshapeStratBox1->raise();
        _deviceBox1->raise();
        label_25->raise();
        _objectBox1->raise();
        label_26->raise();
        _graspPolicyBox1->raise();

        verticalLayout_7->addWidget(groupBox_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(369, 0));
        horizontalLayout_3 = new QHBoxLayout(groupBox);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        gridLayout->addWidget(label_18, 0, 0, 1, 1);

        _testsLeftLbl = new QLabel(groupBox);
        _testsLeftLbl->setObjectName(QString::fromUtf8("_testsLeftLbl"));

        gridLayout->addWidget(_testsLeftLbl, 0, 1, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        _avgTimePerTestLbl = new QLabel(groupBox);
        _avgTimePerTestLbl->setObjectName(QString::fromUtf8("_avgTimePerTestLbl"));

        gridLayout->addWidget(_avgTimePerTestLbl, 1, 1, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        _estimatedTimeLeftLbl = new QLabel(groupBox);
        _estimatedTimeLeftLbl->setObjectName(QString::fromUtf8("_estimatedTimeLeftLbl"));

        gridLayout->addWidget(_estimatedTimeLeftLbl, 2, 1, 1, 1);

        label_21 = new QLabel(groupBox);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        gridLayout->addWidget(label_21, 0, 2, 1, 1);

        _simSpeedLbl = new QLabel(groupBox);
        _simSpeedLbl->setObjectName(QString::fromUtf8("_simSpeedLbl"));

        gridLayout->addWidget(_simSpeedLbl, 0, 3, 1, 1);

        label_19 = new QLabel(groupBox);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        gridLayout->addWidget(label_19, 1, 2, 1, 1);

        _avgSimTimeLbl = new QLabel(groupBox);
        _avgSimTimeLbl->setObjectName(QString::fromUtf8("_avgSimTimeLbl"));

        gridLayout->addWidget(_avgSimTimeLbl, 1, 3, 1, 1);

        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QString::fromUtf8("label_20"));

        gridLayout->addWidget(label_20, 2, 2, 1, 1);

        _simsPerSecLbl = new QLabel(groupBox);
        _simsPerSecLbl->setObjectName(QString::fromUtf8("_simsPerSecLbl"));

        gridLayout->addWidget(_simsPerSecLbl, 2, 3, 1, 1);


        verticalLayout_3->addLayout(gridLayout);

        _simProgress = new QProgressBar(groupBox);
        _simProgress->setObjectName(QString::fromUtf8("_simProgress"));
        _simProgress->setValue(0);
        _simProgress->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(_simProgress);


        horizontalLayout_3->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        _resetBtn = new QPushButton(groupBox);
        _resetBtn->setObjectName(QString::fromUtf8("_resetBtn"));
        _resetBtn->setMaximumSize(QSize(75, 16777215));

        verticalLayout_4->addWidget(_resetBtn);


        horizontalLayout_3->addLayout(verticalLayout_4);


        horizontalLayout_2->addWidget(groupBox);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        _scapeBtn = new QPushButton(scrollAreaWidgetContents);
        _scapeBtn->setObjectName(QString::fromUtf8("_scapeBtn"));

        verticalLayout_2->addWidget(_scapeBtn);

        _saveBtn1 = new QPushButton(scrollAreaWidgetContents);
        _saveBtn1->setObjectName(QString::fromUtf8("_saveBtn1"));
        _saveBtn1->setMaximumSize(QSize(77, 16777215));

        verticalLayout_2->addWidget(_saveBtn1);


        horizontalLayout_2->addLayout(verticalLayout_2);


        verticalLayout_7->addLayout(horizontalLayout_2);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_6->addWidget(scrollArea);

        tabWidget->addTab(tab_2, QString());

        verticalLayout_5->addWidget(tabWidget);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        _genTableBtn = new QPushButton(dockWidgetContents);
        _genTableBtn->setObjectName(QString::fromUtf8("_genTableBtn"));

        horizontalLayout_8->addWidget(_genTableBtn);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_4);

        _loadGTableConfig = new QPushButton(dockWidgetContents);
        _loadGTableConfig->setObjectName(QString::fromUtf8("_loadGTableConfig"));

        horizontalLayout_8->addWidget(_loadGTableConfig);

        _saveGTableConfig = new QPushButton(dockWidgetContents);
        _saveGTableConfig->setObjectName(QString::fromUtf8("_saveGTableConfig"));

        horizontalLayout_8->addWidget(_saveGTableConfig);


        verticalLayout_5->addLayout(horizontalLayout_8);

        groupBox_13 = new QGroupBox(dockWidgetContents);
        groupBox_13->setObjectName(QString::fromUtf8("groupBox_13"));
        groupBox_13->setFlat(false);
        groupBox_13->setCheckable(false);
        groupBox_13->setChecked(false);
        verticalLayout_13 = new QVBoxLayout(groupBox_13);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        _statusLbl = new QLabel(groupBox_13);
        _statusLbl->setObjectName(QString::fromUtf8("_statusLbl"));

        verticalLayout_13->addWidget(_statusLbl);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        progressBar = new QProgressBar(groupBox_13);
        progressBar->setObjectName(QString::fromUtf8("progressBar"));
        progressBar->setValue(0);

        horizontalLayout_9->addWidget(progressBar);

        _stopBtn = new QPushButton(groupBox_13);
        _stopBtn->setObjectName(QString::fromUtf8("_stopBtn"));
        _stopBtn->setMaximumSize(QSize(75, 16777215));

        horizontalLayout_9->addWidget(_stopBtn);


        verticalLayout_13->addLayout(horizontalLayout_9);


        verticalLayout_5->addWidget(groupBox_13);

        GraspTableGeneratorPlugin->setWidget(dockWidgetContents);

        retranslateUi(GraspTableGeneratorPlugin);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(GraspTableGeneratorPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *GraspTableGeneratorPlugin)
    {
        GraspTableGeneratorPlugin->setWindowTitle(QApplication::translate("GraspTableGeneratorPlugin", "RWSim", 0, QApplication::UnicodeUTF8));
        label_62->setText(QApplication::translate("GraspTableGeneratorPlugin", "Table size", 0, QApplication::UnicodeUTF8));
        label_63->setText(QApplication::translate("GraspTableGeneratorPlugin", "Group size", 0, QApplication::UnicodeUTF8));
        _collisionFreeInitBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Collision free", 0, QApplication::UnicodeUTF8));
        _dynamicSimulationBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Dynamic simulation", 0, QApplication::UnicodeUTF8));
        _continuesLoggingBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Continues data", 0, QApplication::UnicodeUTF8));
        label_65->setText(QApplication::translate("GraspTableGeneratorPlugin", "Device", 0, QApplication::UnicodeUTF8));
        label_66->setText(QApplication::translate("GraspTableGeneratorPlugin", "Object", 0, QApplication::UnicodeUTF8));
        _fixedGroupBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Use fixed step", 0, QApplication::UnicodeUTF8));
        label_29->setText(QApplication::translate("GraspTableGeneratorPlugin", "Grasp policy", 0, QApplication::UnicodeUTF8));
        _propGPolicyBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Customize", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("GraspTableGeneratorPlugin", "Grasp strategy", 0, QApplication::UnicodeUTF8));
        _propGStrategyBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Customize", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Quality metrics", 0, QApplication::UnicodeUTF8));
        label_31->setText(QApplication::translate("GraspTableGeneratorPlugin", "Available Quality metrics", 0, QApplication::UnicodeUTF8));
        _addBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Add", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("GraspTableGeneratorPlugin", "Selected quality metrics:", 0, QApplication::UnicodeUTF8));
        _removeBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Remove", 0, QApplication::UnicodeUTF8));
        groupBox_8->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Grasp Table Description", 0, QApplication::UnicodeUTF8));
        label_64->setText(QApplication::translate("GraspTableGeneratorPlugin", "Output:", 0, QApplication::UnicodeUTF8));
        _outputFileEdit->setText(QApplication::translate("GraspTableGeneratorPlugin", "gtable_output.gdata", 0, QApplication::UnicodeUTF8));
        _browseOutputBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Browse", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("GraspTableGeneratorPlugin", "Table setup", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("GraspTableGeneratorPlugin", "Simulator:", 0, QApplication::UnicodeUTF8));
        _simulatorConfigBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Configure", 0, QApplication::UnicodeUTF8));
        groupBox_7->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "General options", 0, QApplication::UnicodeUTF8));
        label_61->setText(QApplication::translate("GraspTableGeneratorPlugin", "Nr of threads:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _nrOfThreadsSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The nr of simulations that will</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"> be performed in parallel. </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">Should not exeed the nr of </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">processors on the current"
                        " PC.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        groupBox_2->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Stop criteria", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min linear vel:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _linVelSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "Objects must have an linear \n"
"velocity below this value, before\n"
"it is considered to be \n"
"resting.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_12->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min angular vel:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _angVelSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "Objects must have an angular \n"
"velocity below this value, before\n"
"it is considered to be \n"
"resting.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_13->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min linear acc:", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min angular acc:", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min resting time:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _minRestTimeSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"> The minimum of time that the </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">other stop criterias must be </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">true before this stop criteria </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">becomes true.</p>\n"
""
                        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"> Time is in seconds</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"> </p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_16->setText(QApplication::translate("GraspTableGeneratorPlugin", "Timeout", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _maxRunningTimeSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The maximum running time </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">allowed. If this time is exceeded </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">the one test will not be saved.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_17->setText(QApplication::translate("GraspTableGeneratorPlugin", "Min time before valid", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _minTimeValidSpin->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "The minimum time that the simulation \n"
"has too run before the other stop \n"
"criterias are evaluated.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_28->setText(QApplication::translate("GraspTableGeneratorPlugin", "Backup interval ", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("GraspTableGeneratorPlugin", "Simulation", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _forceUpdateCheck_3->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Forces RobWorkStudio to draw the simulated screen at the specified intervals.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _forceUpdateCheck_3->setText(QApplication::translate("GraspTableGeneratorPlugin", "Force OpenGl update", 0, QApplication::UnicodeUTF8));
        _onlyShowRestBox_3->setText(QApplication::translate("GraspTableGeneratorPlugin", "Only show resting configuration", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox_5->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Records not only the end state but also the intermidiate states such that it is possible to playback all simulations afterwards.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_5->setText(QApplication::translate("GraspTableGeneratorPlugin", "Record all states", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QApplication::translate("GraspTableGeneratorPlugin", "Visualization", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Options", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _forceUpdateCheck->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Forces RobWorkStudio to draw the simulated screen at the specified intervals.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _forceUpdateCheck->setText(QApplication::translate("GraspTableGeneratorPlugin", "Force OpenGl update", 0, QApplication::UnicodeUTF8));
        _simulatorBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Simulator cfg", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Records not only the end state but also the intermidiate states such that it is possible to playback all simulations afterwards.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Record complete state", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox_2->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The quality of the support pose is based on a 6D wrench space calculation of the supporting \"grasp\".</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_2->setText(QApplication::translate("GraspTableGeneratorPlugin", "Log quality of support pose", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("GraspTableGeneratorPlugin", "Save directory", 0, QApplication::UnicodeUTF8));
        _savePath->setText(QApplication::translate("GraspTableGeneratorPlugin", "c:/tmp", 0, QApplication::UnicodeUTF8));
        _browseBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Browse", 0, QApplication::UnicodeUTF8));
        _onlyShowRestBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "Only show resting configuration", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _saveMultipleCheck->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "If set end states will be saved in \n"
"seperate files with names are\n"
" appended with _XXXXXX\n"
"\n"
"ex:\n"
"\"filename\"_000001\n"
"\"filename\"_000002\n"
"\"filename\"_000003\n"
"\"filename\"_000004\n"
"", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _saveMultipleCheck->setText(QApplication::translate("GraspTableGeneratorPlugin", "Save in multiple files", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Random start configuration of object", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("GraspTableGeneratorPlugin", "Roll:", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("GraspTableGeneratorPlugin", "Pitch:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("GraspTableGeneratorPlugin", "Yaw:", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("GraspTableGeneratorPlugin", "X:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("GraspTableGeneratorPlugin", "Y:", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("GraspTableGeneratorPlugin", "Z:", 0, QApplication::UnicodeUTF8));
        _colFreeStart->setText(QApplication::translate("GraspTableGeneratorPlugin", "Collision free start cfg.", 0, QApplication::UnicodeUTF8));
        _continuesLogging->setText(QApplication::translate("GraspTableGeneratorPlugin", "Continues logging", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("GraspTableGeneratorPlugin", "Logging interval", 0, QApplication::UnicodeUTF8));
        _descBox->setText(QApplication::translate("GraspTableGeneratorPlugin", "#Descriction: ", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Grasp control", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("GraspTableGeneratorPlugin", "Grasp policy", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("GraspTableGeneratorPlugin", "Grasp preshape strategy", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("GraspTableGeneratorPlugin", "Device", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("GraspTableGeneratorPlugin", "Object", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Status", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("GraspTableGeneratorPlugin", "Test #: ", 0, QApplication::UnicodeUTF8));
        _testsLeftLbl->setText(QApplication::translate("GraspTableGeneratorPlugin", "0/0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        label_2->setToolTip(QApplication::translate("GraspTableGeneratorPlugin", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">The average time per test in miliseconds.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_2->setText(QApplication::translate("GraspTableGeneratorPlugin", "Real avg. time : ", 0, QApplication::UnicodeUTF8));
        _avgTimePerTestLbl->setText(QApplication::translate("GraspTableGeneratorPlugin", "s", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("GraspTableGeneratorPlugin", "Est real time : ", 0, QApplication::UnicodeUTF8));
        _estimatedTimeLeftLbl->setText(QApplication::translate("GraspTableGeneratorPlugin", "s", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("GraspTableGeneratorPlugin", "Speed :", 0, QApplication::UnicodeUTF8));
        _simSpeedLbl->setText(QString());
        label_19->setText(QApplication::translate("GraspTableGeneratorPlugin", "Sim avg. time:", 0, QApplication::UnicodeUTF8));
        _avgSimTimeLbl->setText(QApplication::translate("GraspTableGeneratorPlugin", "s", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("GraspTableGeneratorPlugin", "Sims per sec.:", 0, QApplication::UnicodeUTF8));
        _simsPerSecLbl->setText(QString());
        _resetBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Reset", 0, QApplication::UnicodeUTF8));
        _scapeBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Export Scape", 0, QApplication::UnicodeUTF8));
        _saveBtn1->setText(QApplication::translate("GraspTableGeneratorPlugin", "Save file", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("GraspTableGeneratorPlugin", "Tab 2", 0, QApplication::UnicodeUTF8));
        _genTableBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Generate table", 0, QApplication::UnicodeUTF8));
        _loadGTableConfig->setText(QApplication::translate("GraspTableGeneratorPlugin", "Load configuration", 0, QApplication::UnicodeUTF8));
        _saveGTableConfig->setText(QApplication::translate("GraspTableGeneratorPlugin", "Save configuration", 0, QApplication::UnicodeUTF8));
        groupBox_13->setTitle(QApplication::translate("GraspTableGeneratorPlugin", "Status", 0, QApplication::UnicodeUTF8));
        _statusLbl->setText(QApplication::translate("GraspTableGeneratorPlugin", "Status lbl", 0, QApplication::UnicodeUTF8));
        _stopBtn->setText(QApplication::translate("GraspTableGeneratorPlugin", "Stop", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GraspTableGeneratorPlugin: public Ui_GraspTableGeneratorPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRASPTABLEGENERATORPLUGIN_H
