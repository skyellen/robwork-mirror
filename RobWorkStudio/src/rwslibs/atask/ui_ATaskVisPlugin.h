/********************************************************************************
** Form generated from reading UI file 'ATaskVisPlugin.ui'
**
** Created: Fri Jan 10 00:15:33 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ATASKVISPLUGIN_H
#define UI_ATASKVISPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ATaskVisPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout_4;
    QGridLayout *gridLayout;
    QPushButton *_loadTasksBtn;
    QPushButton *_loadResultsBtn;
    QSpacerItem *verticalSpacer;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_5;
    QLabel *label_7;
    QSpinBox *_taskSelectSpin;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_3;
    QCheckBox *_successBox;
    QCheckBox *_failedBox;
    QPushButton *btnRecordVideo;
    QGroupBox *_taskBox;
    QGridLayout *gridLayout_2;
    QLabel *_taskDescription;
    QFormLayout *_taskBoxLayout;
    QPushButton *_highlightButton;
    QGroupBox *_resultBox;
    QGridLayout *gridLayout_6;
    QLabel *_contactLabel;
    QRadioButton *_real;
    QLabel *_timeLabel;
    QLabel *_phase;
    QRadioButton *_assumed;
    QLabel *_ftMaleLabel;
    QLabel *_ftFemaleLabel;
    QProgressBar *_progress;

    void setupUi(QDockWidget *ATaskVisPlugin)
    {
        if (ATaskVisPlugin->objectName().isEmpty())
            ATaskVisPlugin->setObjectName(QString::fromUtf8("ATaskVisPlugin"));
        ATaskVisPlugin->resize(511, 766);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _loadTasksBtn = new QPushButton(dockWidgetContents);
        _loadTasksBtn->setObjectName(QString::fromUtf8("_loadTasksBtn"));

        gridLayout->addWidget(_loadTasksBtn, 0, 0, 1, 1);

        _loadResultsBtn = new QPushButton(dockWidgetContents);
        _loadResultsBtn->setObjectName(QString::fromUtf8("_loadResultsBtn"));

        gridLayout->addWidget(_loadResultsBtn, 0, 1, 1, 1);


        gridLayout_4->addLayout(gridLayout, 0, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_4->addItem(verticalSpacer, 7, 0, 1, 1);

        groupBox_2 = new QGroupBox(dockWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_5 = new QGridLayout(groupBox_2);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_5->addWidget(label_7, 0, 0, 1, 1);

        _taskSelectSpin = new QSpinBox(groupBox_2);
        _taskSelectSpin->setObjectName(QString::fromUtf8("_taskSelectSpin"));

        gridLayout_5->addWidget(_taskSelectSpin, 0, 1, 1, 1);


        gridLayout_4->addWidget(groupBox_2, 2, 0, 1, 1);

        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_3 = new QGridLayout(groupBox);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        _successBox = new QCheckBox(groupBox);
        _successBox->setObjectName(QString::fromUtf8("_successBox"));
        _successBox->setChecked(true);

        gridLayout_3->addWidget(_successBox, 0, 0, 1, 1);

        _failedBox = new QCheckBox(groupBox);
        _failedBox->setObjectName(QString::fromUtf8("_failedBox"));
        _failedBox->setChecked(true);

        gridLayout_3->addWidget(_failedBox, 0, 1, 1, 1);


        gridLayout_4->addWidget(groupBox, 3, 0, 1, 1);

        btnRecordVideo = new QPushButton(dockWidgetContents);
        btnRecordVideo->setObjectName(QString::fromUtf8("btnRecordVideo"));

        gridLayout_4->addWidget(btnRecordVideo, 4, 0, 1, 1);

        _taskBox = new QGroupBox(dockWidgetContents);
        _taskBox->setObjectName(QString::fromUtf8("_taskBox"));
        gridLayout_2 = new QGridLayout(_taskBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _taskDescription = new QLabel(_taskBox);
        _taskDescription->setObjectName(QString::fromUtf8("_taskDescription"));

        gridLayout_2->addWidget(_taskDescription, 0, 0, 1, 1);

        _taskBoxLayout = new QFormLayout();
        _taskBoxLayout->setObjectName(QString::fromUtf8("_taskBoxLayout"));

        gridLayout_2->addLayout(_taskBoxLayout, 2, 0, 1, 1);

        _highlightButton = new QPushButton(_taskBox);
        _highlightButton->setObjectName(QString::fromUtf8("_highlightButton"));

        gridLayout_2->addWidget(_highlightButton, 1, 0, 1, 1);


        gridLayout_4->addWidget(_taskBox, 5, 0, 1, 1);

        _resultBox = new QGroupBox(dockWidgetContents);
        _resultBox->setObjectName(QString::fromUtf8("_resultBox"));
        gridLayout_6 = new QGridLayout(_resultBox);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        _contactLabel = new QLabel(_resultBox);
        _contactLabel->setObjectName(QString::fromUtf8("_contactLabel"));

        gridLayout_6->addWidget(_contactLabel, 8, 0, 1, 1);

        _real = new QRadioButton(_resultBox);
        _real->setObjectName(QString::fromUtf8("_real"));
        _real->setEnabled(false);
        _real->setCheckable(true);
        _real->setChecked(false);

        gridLayout_6->addWidget(_real, 0, 0, 1, 1);

        _timeLabel = new QLabel(_resultBox);
        _timeLabel->setObjectName(QString::fromUtf8("_timeLabel"));

        gridLayout_6->addWidget(_timeLabel, 4, 1, 1, 1);

        _phase = new QLabel(_resultBox);
        _phase->setObjectName(QString::fromUtf8("_phase"));

        gridLayout_6->addWidget(_phase, 4, 0, 1, 1);

        _assumed = new QRadioButton(_resultBox);
        _assumed->setObjectName(QString::fromUtf8("_assumed"));
        _assumed->setEnabled(false);
        _assumed->setCheckable(true);

        gridLayout_6->addWidget(_assumed, 0, 1, 1, 1);

        _ftMaleLabel = new QLabel(_resultBox);
        _ftMaleLabel->setObjectName(QString::fromUtf8("_ftMaleLabel"));

        gridLayout_6->addWidget(_ftMaleLabel, 5, 0, 1, 2);

        _ftFemaleLabel = new QLabel(_resultBox);
        _ftFemaleLabel->setObjectName(QString::fromUtf8("_ftFemaleLabel"));

        gridLayout_6->addWidget(_ftFemaleLabel, 6, 0, 1, 2);


        gridLayout_4->addWidget(_resultBox, 6, 0, 1, 1);

        _progress = new QProgressBar(dockWidgetContents);
        _progress->setObjectName(QString::fromUtf8("_progress"));
        _progress->setValue(24);

        gridLayout_4->addWidget(_progress, 1, 0, 1, 1);


        verticalLayout_2->addLayout(gridLayout_4);

        ATaskVisPlugin->setWidget(dockWidgetContents);

        retranslateUi(ATaskVisPlugin);

        QMetaObject::connectSlotsByName(ATaskVisPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *ATaskVisPlugin)
    {
        ATaskVisPlugin->setWindowTitle(QApplication::translate("ATaskVisPlugin", "Assembly Visualisation", 0, QApplication::UnicodeUTF8));
        _loadTasksBtn->setText(QApplication::translate("ATaskVisPlugin", "Load tasks", 0, QApplication::UnicodeUTF8));
        _loadResultsBtn->setText(QApplication::translate("ATaskVisPlugin", "Load results", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("ATaskVisPlugin", "Select grasp", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("ATaskVisPlugin", "Grasp", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("ATaskVisPlugin", "Show options", 0, QApplication::UnicodeUTF8));
        _successBox->setText(QApplication::translate("ATaskVisPlugin", "Success", 0, QApplication::UnicodeUTF8));
        _failedBox->setText(QApplication::translate("ATaskVisPlugin", "Failed", 0, QApplication::UnicodeUTF8));
        btnRecordVideo->setText(QApplication::translate("ATaskVisPlugin", "Record Video", 0, QApplication::UnicodeUTF8));
        _taskBox->setTitle(QApplication::translate("ATaskVisPlugin", "Task", 0, QApplication::UnicodeUTF8));
        _taskDescription->setText(QApplication::translate("ATaskVisPlugin", "TextLabel", 0, QApplication::UnicodeUTF8));
        _highlightButton->setText(QApplication::translate("ATaskVisPlugin", "Highlight", 0, QApplication::UnicodeUTF8));
        _resultBox->setTitle(QApplication::translate("ATaskVisPlugin", "Result", 0, QApplication::UnicodeUTF8));
        _contactLabel->setText(QApplication::translate("ATaskVisPlugin", "Contact", 0, QApplication::UnicodeUTF8));
        _real->setText(QApplication::translate("ATaskVisPlugin", "Real", 0, QApplication::UnicodeUTF8));
        _timeLabel->setText(QApplication::translate("ATaskVisPlugin", "Time", 0, QApplication::UnicodeUTF8));
        _phase->setText(QApplication::translate("ATaskVisPlugin", "Phase", 0, QApplication::UnicodeUTF8));
        _assumed->setText(QApplication::translate("ATaskVisPlugin", "Assumed", 0, QApplication::UnicodeUTF8));
        _ftMaleLabel->setText(QApplication::translate("ATaskVisPlugin", "Force/Torque (male)", 0, QApplication::UnicodeUTF8));
        _ftFemaleLabel->setText(QApplication::translate("ATaskVisPlugin", "Force/Torque (female)", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ATaskVisPlugin: public Ui_ATaskVisPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ATASKVISPLUGIN_H
