/********************************************************************************
** Form generated from reading UI file 'GraspSelectionDialog.ui'
**
** Created: Sun 30. Jan 22:12:20 2011
**      by: Qt User Interface Compiler version 4.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GRASPSELECTIONDIALOG_H
#define UI_GRASPSELECTIONDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GraspSelectionDialog
{
public:
    QAction *actionBum;
    QVBoxLayout *verticalLayout_5;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_22;
    QLineEdit *_savePath;
    QPushButton *_browseBtn;
    QCheckBox *_saveMultipleCheck;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_3;
    QSpinBox *_nrOfTestsSpin;
    QSpacerItem *verticalSpacer_4;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox_4;
    QSpinBox *_updateRateSpin;
    QCheckBox *_forceUpdateCheck;
    QPushButton *_simulatorBtn;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *_onlyShowRestBox;
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
    QDoubleSpinBox *doubleSpinBox;
    QDoubleSpinBox *doubleSpinBox_2;
    QDoubleSpinBox *doubleSpinBox_3;
    QCheckBox *_colFreeStart;
    QGroupBox *groupBox_2;
    QSpinBox *_nrOfThreadsSpin;
    QLabel *label_4;
    QDoubleSpinBox *_linVelSpin;
    QLabel *label_11;
    QLabel *label_12;
    QDoubleSpinBox *_angVelSpin;
    QLabel *label_13;
    QLabel *label_14;
    QDoubleSpinBox *_linAccSpin;
    QDoubleSpinBox *_angAccSpin;
    QDoubleSpinBox *_minRestTimeSpin;
    QLabel *label_15;
    QDoubleSpinBox *_maxRunningTimeSpin;
    QLabel *label_16;
    QDoubleSpinBox *_minTimeValidSpin;
    QLabel *label_17;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_9;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *_graspTableLoadBtn;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_10;
    QPushButton *_buildTreeBtn;
    QSpacerItem *horizontalSpacer_2;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_8;
    QLineEdit *_searchEdit;
    QPushButton *_searchBtn;
    QHBoxLayout *horizontalLayout_9;
    QLabel *_foundFeasibleLbl;
    QSpacerItem *verticalSpacer_3;
    QGridLayout *gridLayout_2;
    QLabel *_qualityLbl;
    QLabel *_contactLbl;
    QLabel *_poseLbl;
    QLabel *label_26;
    QHBoxLayout *horizontalLayout_7;
    QSpinBox *_graspSpin;
    QSlider *_graspTableSlider;
    QSpacerItem *verticalSpacer_2;
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
    QPushButton *_startBtn;
    QPushButton *_stopBtn;
    QPushButton *_resetBtn;
    QVBoxLayout *verticalLayout_2;
    QSpacerItem *verticalSpacer;
    QPushButton *_scapeBtn;
    QPushButton *_saveBtn1;
    QPushButton *_exitBtn;

    void setupUi(QDialog *GraspSelectionDialog)
    {
        if (GraspSelectionDialog->objectName().isEmpty())
            GraspSelectionDialog->setObjectName(QString::fromUtf8("GraspSelectionDialog"));
        GraspSelectionDialog->resize(749, 552);
        GraspSelectionDialog->setSizeGripEnabled(true);
        GraspSelectionDialog->setModal(false);
        actionBum = new QAction(GraspSelectionDialog);
        actionBum->setObjectName(QString::fromUtf8("actionBum"));
        verticalLayout_5 = new QVBoxLayout(GraspSelectionDialog);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        tabWidget = new QTabWidget(GraspSelectionDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout_7 = new QVBoxLayout(tab);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_22 = new QLabel(tab);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        horizontalLayout_4->addWidget(label_22);

        _savePath = new QLineEdit(tab);
        _savePath->setObjectName(QString::fromUtf8("_savePath"));

        horizontalLayout_4->addWidget(_savePath);

        _browseBtn = new QPushButton(tab);
        _browseBtn->setObjectName(QString::fromUtf8("_browseBtn"));

        horizontalLayout_4->addWidget(_browseBtn);


        verticalLayout_7->addLayout(horizontalLayout_4);

        _saveMultipleCheck = new QCheckBox(tab);
        _saveMultipleCheck->setObjectName(QString::fromUtf8("_saveMultipleCheck"));

        verticalLayout_7->addWidget(_saveMultipleCheck);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_5->addWidget(label_3);

        _nrOfTestsSpin = new QSpinBox(tab);
        _nrOfTestsSpin->setObjectName(QString::fromUtf8("_nrOfTestsSpin"));
        _nrOfTestsSpin->setMaximumSize(QSize(100, 16777215));
        _nrOfTestsSpin->setMinimum(1);
        _nrOfTestsSpin->setMaximum(1000000);
        _nrOfTestsSpin->setValue(100);

        horizontalLayout_5->addWidget(_nrOfTestsSpin);


        verticalLayout_7->addLayout(horizontalLayout_5);

        verticalSpacer_4 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer_4);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_6 = new QVBoxLayout(tab_2);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox_4 = new QGroupBox(tab_2);
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
        _onlyShowRestBox = new QCheckBox(groupBox_4);
        _onlyShowRestBox->setObjectName(QString::fromUtf8("_onlyShowRestBox"));
        _onlyShowRestBox->setGeometry(QRect(10, 70, 191, 19));

        verticalLayout->addWidget(groupBox_4);

        groupBox_5 = new QGroupBox(tab_2);
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
        doubleSpinBox = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setGeometry(QRect(190, 30, 62, 22));
        doubleSpinBox_2 = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox_2->setObjectName(QString::fromUtf8("doubleSpinBox_2"));
        doubleSpinBox_2->setGeometry(QRect(190, 60, 62, 22));
        doubleSpinBox_3 = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox_3->setObjectName(QString::fromUtf8("doubleSpinBox_3"));
        doubleSpinBox_3->setGeometry(QRect(190, 90, 62, 22));
        _colFreeStart = new QCheckBox(groupBox_5);
        _colFreeStart->setObjectName(QString::fromUtf8("_colFreeStart"));
        _colFreeStart->setGeometry(QRect(10, 120, 131, 21));

        verticalLayout->addWidget(groupBox_5);


        horizontalLayout->addLayout(verticalLayout);

        groupBox_2 = new QGroupBox(tab_2);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setMinimumSize(QSize(201, 0));
        _nrOfThreadsSpin = new QSpinBox(groupBox_2);
        _nrOfThreadsSpin->setObjectName(QString::fromUtf8("_nrOfThreadsSpin"));
        _nrOfThreadsSpin->setGeometry(QRect(110, 50, 61, 22));
        _nrOfThreadsSpin->setMinimum(1);
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 50, 118, 21));
        _linVelSpin = new QDoubleSpinBox(groupBox_2);
        _linVelSpin->setObjectName(QString::fromUtf8("_linVelSpin"));
        _linVelSpin->setGeometry(QRect(111, 80, 71, 22));
        _linVelSpin->setDecimals(3);
        _linVelSpin->setMinimum(0.001);
        _linVelSpin->setMaximum(1);
        _linVelSpin->setSingleStep(0.01);
        _linVelSpin->setValue(0.05);
        label_11 = new QLabel(groupBox_2);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(10, 80, 71, 21));
        label_12 = new QLabel(groupBox_2);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(10, 110, 81, 21));
        _angVelSpin = new QDoubleSpinBox(groupBox_2);
        _angVelSpin->setObjectName(QString::fromUtf8("_angVelSpin"));
        _angVelSpin->setGeometry(QRect(111, 110, 71, 22));
        _angVelSpin->setDecimals(3);
        _angVelSpin->setMinimum(0.001);
        _angVelSpin->setMaximum(1);
        _angVelSpin->setSingleStep(0.01);
        _angVelSpin->setValue(0.05);
        label_13 = new QLabel(groupBox_2);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(10, 140, 81, 21));
        label_14 = new QLabel(groupBox_2);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(10, 170, 91, 21));
        _linAccSpin = new QDoubleSpinBox(groupBox_2);
        _linAccSpin->setObjectName(QString::fromUtf8("_linAccSpin"));
        _linAccSpin->setGeometry(QRect(111, 140, 71, 22));
        _linAccSpin->setDecimals(3);
        _linAccSpin->setMinimum(0);
        _linAccSpin->setMaximum(1);
        _linAccSpin->setSingleStep(1e-06);
        _linAccSpin->setValue(0);
        _angAccSpin = new QDoubleSpinBox(groupBox_2);
        _angAccSpin->setObjectName(QString::fromUtf8("_angAccSpin"));
        _angAccSpin->setGeometry(QRect(111, 170, 71, 22));
        _angAccSpin->setDecimals(3);
        _angAccSpin->setMinimum(0);
        _angAccSpin->setMaximum(1);
        _angAccSpin->setSingleStep(1e-06);
        _angAccSpin->setValue(0);
        _minRestTimeSpin = new QDoubleSpinBox(groupBox_2);
        _minRestTimeSpin->setObjectName(QString::fromUtf8("_minRestTimeSpin"));
        _minRestTimeSpin->setGeometry(QRect(120, 200, 62, 22));
        _minRestTimeSpin->setSingleStep(0.01);
        _minRestTimeSpin->setValue(0.2);
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 200, 81, 21));
        _maxRunningTimeSpin = new QDoubleSpinBox(groupBox_2);
        _maxRunningTimeSpin->setObjectName(QString::fromUtf8("_maxRunningTimeSpin"));
        _maxRunningTimeSpin->setGeometry(QRect(120, 230, 62, 22));
        _maxRunningTimeSpin->setMaximum(999.99);
        _maxRunningTimeSpin->setValue(50);
        label_16 = new QLabel(groupBox_2);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(10, 230, 91, 21));
        _minTimeValidSpin = new QDoubleSpinBox(groupBox_2);
        _minTimeValidSpin->setObjectName(QString::fromUtf8("_minTimeValidSpin"));
        _minTimeValidSpin->setGeometry(QRect(120, 260, 62, 22));
        _minTimeValidSpin->setMinimum(0.1);
        label_17 = new QLabel(groupBox_2);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 260, 101, 21));

        horizontalLayout->addWidget(groupBox_2);


        verticalLayout_6->addLayout(horizontalLayout);

        tabWidget->addTab(tab_2, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_9 = new QVBoxLayout(tab_3);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        _graspTableLoadBtn = new QPushButton(tab_3);
        _graspTableLoadBtn->setObjectName(QString::fromUtf8("_graspTableLoadBtn"));

        horizontalLayout_6->addWidget(_graspTableLoadBtn);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_6->addItem(horizontalSpacer);


        verticalLayout_9->addLayout(horizontalLayout_6);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));
        _buildTreeBtn = new QPushButton(tab_3);
        _buildTreeBtn->setObjectName(QString::fromUtf8("_buildTreeBtn"));

        horizontalLayout_10->addWidget(_buildTreeBtn);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_10->addItem(horizontalSpacer_2);


        verticalLayout_9->addLayout(horizontalLayout_10);

        groupBox_3 = new QGroupBox(tab_3);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        verticalLayout_8 = new QVBoxLayout(groupBox_3);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        _searchEdit = new QLineEdit(groupBox_3);
        _searchEdit->setObjectName(QString::fromUtf8("_searchEdit"));

        horizontalLayout_8->addWidget(_searchEdit);

        _searchBtn = new QPushButton(groupBox_3);
        _searchBtn->setObjectName(QString::fromUtf8("_searchBtn"));

        horizontalLayout_8->addWidget(_searchBtn);


        verticalLayout_8->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        _foundFeasibleLbl = new QLabel(groupBox_3);
        _foundFeasibleLbl->setObjectName(QString::fromUtf8("_foundFeasibleLbl"));

        horizontalLayout_9->addWidget(_foundFeasibleLbl);


        verticalLayout_8->addLayout(horizontalLayout_9);


        verticalLayout_9->addWidget(groupBox_3);

        verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_3);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _qualityLbl = new QLabel(tab_3);
        _qualityLbl->setObjectName(QString::fromUtf8("_qualityLbl"));

        gridLayout_2->addWidget(_qualityLbl, 0, 0, 1, 1);

        _contactLbl = new QLabel(tab_3);
        _contactLbl->setObjectName(QString::fromUtf8("_contactLbl"));

        gridLayout_2->addWidget(_contactLbl, 1, 0, 1, 1);

        _poseLbl = new QLabel(tab_3);
        _poseLbl->setObjectName(QString::fromUtf8("_poseLbl"));

        gridLayout_2->addWidget(_poseLbl, 0, 1, 1, 1);

        label_26 = new QLabel(tab_3);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        gridLayout_2->addWidget(label_26, 1, 1, 1, 1);


        verticalLayout_9->addLayout(gridLayout_2);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        _graspSpin = new QSpinBox(tab_3);
        _graspSpin->setObjectName(QString::fromUtf8("_graspSpin"));
        _graspSpin->setMaximum(100000);

        horizontalLayout_7->addWidget(_graspSpin);

        _graspTableSlider = new QSlider(tab_3);
        _graspTableSlider->setObjectName(QString::fromUtf8("_graspTableSlider"));
        _graspTableSlider->setOrientation(Qt::Horizontal);

        horizontalLayout_7->addWidget(_graspTableSlider);


        verticalLayout_9->addLayout(horizontalLayout_7);

        tabWidget->addTab(tab_3, QString());

        verticalLayout_5->addWidget(tabWidget);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        groupBox = new QGroupBox(GraspSelectionDialog);
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
        _startBtn = new QPushButton(groupBox);
        _startBtn->setObjectName(QString::fromUtf8("_startBtn"));
        _startBtn->setMaximumSize(QSize(75, 16777215));

        verticalLayout_4->addWidget(_startBtn);

        _stopBtn = new QPushButton(groupBox);
        _stopBtn->setObjectName(QString::fromUtf8("_stopBtn"));
        _stopBtn->setMaximumSize(QSize(75, 16777215));

        verticalLayout_4->addWidget(_stopBtn);

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

        _scapeBtn = new QPushButton(GraspSelectionDialog);
        _scapeBtn->setObjectName(QString::fromUtf8("_scapeBtn"));

        verticalLayout_2->addWidget(_scapeBtn);

        _saveBtn1 = new QPushButton(GraspSelectionDialog);
        _saveBtn1->setObjectName(QString::fromUtf8("_saveBtn1"));
        _saveBtn1->setMaximumSize(QSize(77, 16777215));

        verticalLayout_2->addWidget(_saveBtn1);

        _exitBtn = new QPushButton(GraspSelectionDialog);
        _exitBtn->setObjectName(QString::fromUtf8("_exitBtn"));
        _exitBtn->setMaximumSize(QSize(75, 24));

        verticalLayout_2->addWidget(_exitBtn);


        horizontalLayout_2->addLayout(verticalLayout_2);


        verticalLayout_5->addLayout(horizontalLayout_2);

        QWidget::setTabOrder(checkBox, _forceUpdateCheck);
        QWidget::setTabOrder(_forceUpdateCheck, _updateRateSpin);
        QWidget::setTabOrder(_updateRateSpin, _onlyShowRestBox);
        QWidget::setTabOrder(_onlyShowRestBox, checkBox_2);
        QWidget::setTabOrder(checkBox_2, _simulatorBtn);
        QWidget::setTabOrder(_simulatorBtn, _lowRollSpin);
        QWidget::setTabOrder(_lowRollSpin, _highRollSpin);
        QWidget::setTabOrder(_highRollSpin, _lowPitchSpin);
        QWidget::setTabOrder(_lowPitchSpin, _highPitchSpin);
        QWidget::setTabOrder(_highPitchSpin, _lowYawSpin);
        QWidget::setTabOrder(_lowYawSpin, _highYawSpin);
        QWidget::setTabOrder(_highYawSpin, doubleSpinBox);
        QWidget::setTabOrder(doubleSpinBox, doubleSpinBox_2);
        QWidget::setTabOrder(doubleSpinBox_2, doubleSpinBox_3);
        QWidget::setTabOrder(doubleSpinBox_3, _colFreeStart);
        QWidget::setTabOrder(_colFreeStart, _nrOfThreadsSpin);
        QWidget::setTabOrder(_nrOfThreadsSpin, _linVelSpin);
        QWidget::setTabOrder(_linVelSpin, _angVelSpin);
        QWidget::setTabOrder(_angVelSpin, _linAccSpin);
        QWidget::setTabOrder(_linAccSpin, _angAccSpin);
        QWidget::setTabOrder(_angAccSpin, _minRestTimeSpin);
        QWidget::setTabOrder(_minRestTimeSpin, _maxRunningTimeSpin);
        QWidget::setTabOrder(_maxRunningTimeSpin, _minTimeValidSpin);
        QWidget::setTabOrder(_minTimeValidSpin, _stopBtn);
        QWidget::setTabOrder(_stopBtn, _resetBtn);
        QWidget::setTabOrder(_resetBtn, _saveBtn1);
        QWidget::setTabOrder(_saveBtn1, _exitBtn);

        retranslateUi(GraspSelectionDialog);
        QObject::connect(_exitBtn, SIGNAL(clicked()), GraspSelectionDialog, SLOT(accept()));

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(GraspSelectionDialog);
    } // setupUi

    void retranslateUi(QDialog *GraspSelectionDialog)
    {
        GraspSelectionDialog->setWindowTitle(QApplication::translate("GraspSelectionDialog", "Resting pose simulation", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        GraspSelectionDialog->setToolTip(QApplication::translate("GraspSelectionDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionBum->setText(QApplication::translate("GraspSelectionDialog", "bum", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("GraspSelectionDialog", "Save path", 0, QApplication::UnicodeUTF8));
        _savePath->setText(QApplication::translate("GraspSelectionDialog", "restingStates.rwpath", 0, QApplication::UnicodeUTF8));
        _browseBtn->setText(QApplication::translate("GraspSelectionDialog", "Browse", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _saveMultipleCheck->setToolTip(QApplication::translate("GraspSelectionDialog", "If set end states will be saved in \n"
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
        _saveMultipleCheck->setText(QApplication::translate("GraspSelectionDialog", "Save in multiple files", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("GraspSelectionDialog", "Nr of resting poses to compute: ", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _nrOfTestsSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The number of simulations that is </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">to be performed.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("GraspSelectionDialog", "Options", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("GraspSelectionDialog", "Options", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _forceUpdateCheck->setToolTip(QApplication::translate("GraspSelectionDialog", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Forces RobWorkStudio to draw the simulated screen at the specified intervals.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _forceUpdateCheck->setText(QApplication::translate("GraspSelectionDialog", "Force OpenGl update", 0, QApplication::UnicodeUTF8));
        _simulatorBtn->setText(QApplication::translate("GraspSelectionDialog", "Simulator cfg", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox->setToolTip(QApplication::translate("GraspSelectionDialog", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Records not only the end state but also the intermidiate states such that it is possible to playback all simulations afterwards.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox->setText(QApplication::translate("GraspSelectionDialog", "Record complete state", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        checkBox_2->setToolTip(QApplication::translate("GraspSelectionDialog", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The quality of the support pose is based on a 6D wrench space calculation of the supporting \"grasp\".</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        checkBox_2->setText(QApplication::translate("GraspSelectionDialog", "Log quality of support pose", 0, QApplication::UnicodeUTF8));
        _onlyShowRestBox->setText(QApplication::translate("GraspSelectionDialog", "Only show resting configuration", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("GraspSelectionDialog", "Random start configuration of objects", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("GraspSelectionDialog", "Roll:", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("GraspSelectionDialog", "Pitch:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("GraspSelectionDialog", "Yaw:", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("GraspSelectionDialog", "X:", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("GraspSelectionDialog", "Y:", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("GraspSelectionDialog", "Z:", 0, QApplication::UnicodeUTF8));
        _colFreeStart->setText(QApplication::translate("GraspSelectionDialog", "Collision free start cfg.", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("GraspSelectionDialog", "Stop criteria", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _nrOfThreadsSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The nr of simulations that will</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"> be performed in parallel. </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">Should not exeed the nr of </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">processors on the current"
                        " PC.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_4->setText(QApplication::translate("GraspSelectionDialog", "Nr of threads:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _linVelSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "Objects must have an linear \n"
"velocity below this value, before\n"
"it is considered to be \n"
"resting.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_11->setText(QApplication::translate("GraspSelectionDialog", "Min linear vel:", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("GraspSelectionDialog", "Min angular vel:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _angVelSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "Objects must have an angular \n"
"velocity below this value, before\n"
"it is considered to be \n"
"resting.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_13->setText(QApplication::translate("GraspSelectionDialog", "Min linear acc:", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("GraspSelectionDialog", "Min angular acc:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _minRestTimeSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
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
        label_15->setText(QApplication::translate("GraspSelectionDialog", "Min resting time:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _maxRunningTimeSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">The maximum running time </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">allowed. If this time is exceeded </p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\">the one test will not be saved.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_16->setText(QApplication::translate("GraspSelectionDialog", "Timeout", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _minTimeValidSpin->setToolTip(QApplication::translate("GraspSelectionDialog", "The minimum time that the simulation \n"
"has too run before the other stop \n"
"criterias are evaluated.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_17->setText(QApplication::translate("GraspSelectionDialog", "Min time before valid", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("GraspSelectionDialog", "Advanced", 0, QApplication::UnicodeUTF8));
        _graspTableLoadBtn->setText(QApplication::translate("GraspSelectionDialog", "Open grasp table", 0, QApplication::UnicodeUTF8));
        _buildTreeBtn->setText(QApplication::translate("GraspSelectionDialog", "Build KD-tree", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("GraspSelectionDialog", "Feasible grasps", 0, QApplication::UnicodeUTF8));
        _searchEdit->setText(QApplication::translate("GraspSelectionDialog", "Bounds:", 0, QApplication::UnicodeUTF8));
        _searchBtn->setText(QApplication::translate("GraspSelectionDialog", "Search", 0, QApplication::UnicodeUTF8));
        _foundFeasibleLbl->setText(QApplication::translate("GraspSelectionDialog", "Found: 0", 0, QApplication::UnicodeUTF8));
        _qualityLbl->setText(QApplication::translate("GraspSelectionDialog", "Quality: ", 0, QApplication::UnicodeUTF8));
        _contactLbl->setText(QApplication::translate("GraspSelectionDialog", "Nr of contacts: ", 0, QApplication::UnicodeUTF8));
        _poseLbl->setText(QApplication::translate("GraspSelectionDialog", "Pose: ", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("GraspSelectionDialog", "TextLabel", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("GraspSelectionDialog", "Page", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("GraspSelectionDialog", "Status", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("GraspSelectionDialog", "Test #: ", 0, QApplication::UnicodeUTF8));
        _testsLeftLbl->setText(QApplication::translate("GraspSelectionDialog", "0/0", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        label_2->setToolTip(QApplication::translate("GraspSelectionDialog", "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">The average time per test in miliseconds.</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_2->setText(QApplication::translate("GraspSelectionDialog", "Real avg. time : ", 0, QApplication::UnicodeUTF8));
        _avgTimePerTestLbl->setText(QApplication::translate("GraspSelectionDialog", "s", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("GraspSelectionDialog", "Est real time : ", 0, QApplication::UnicodeUTF8));
        _estimatedTimeLeftLbl->setText(QApplication::translate("GraspSelectionDialog", "s", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("GraspSelectionDialog", "Speed :", 0, QApplication::UnicodeUTF8));
        _simSpeedLbl->setText(QString());
        label_19->setText(QApplication::translate("GraspSelectionDialog", "Sim avg. time:", 0, QApplication::UnicodeUTF8));
        _avgSimTimeLbl->setText(QApplication::translate("GraspSelectionDialog", "s", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("GraspSelectionDialog", "Sims per sec.:", 0, QApplication::UnicodeUTF8));
        _simsPerSecLbl->setText(QString());
        _startBtn->setText(QApplication::translate("GraspSelectionDialog", "Start", 0, QApplication::UnicodeUTF8));
        _stopBtn->setText(QApplication::translate("GraspSelectionDialog", "Stop", 0, QApplication::UnicodeUTF8));
        _resetBtn->setText(QApplication::translate("GraspSelectionDialog", "Reset", 0, QApplication::UnicodeUTF8));
        _scapeBtn->setText(QApplication::translate("GraspSelectionDialog", "Export Scape", 0, QApplication::UnicodeUTF8));
        _saveBtn1->setText(QApplication::translate("GraspSelectionDialog", "Save file", 0, QApplication::UnicodeUTF8));
        _exitBtn->setText(QApplication::translate("GraspSelectionDialog", "Exit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class GraspSelectionDialog: public Ui_GraspSelectionDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GRASPSELECTIONDIALOG_H
