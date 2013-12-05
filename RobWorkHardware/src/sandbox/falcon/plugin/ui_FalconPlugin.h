/********************************************************************************
** Form generated from reading UI file 'FalconPlugin.ui'
**
** Created: Thu Dec 5 20:14:46 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_FALCONPLUGIN_H
#define UI_FALCONPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_FalconPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QComboBox *_gripperCombo;
    QLabel *label_2;
    QComboBox *_robotCombo;
    QLabel *label;
    QComboBox *_tcpCombo;
    QLabel *label_3;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout;
    QPushButton *_stopButton;
    QPushButton *_startButton;
    QPushButton *_recordingButton;
    QLabel *_modeLabel;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *_wsadLabel;

    void setupUi(QDockWidget *FalconPlugin)
    {
        if (FalconPlugin->objectName().isEmpty())
            FalconPlugin->setObjectName(QString::fromUtf8("FalconPlugin"));
        FalconPlugin->resize(275, 460);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setEnabled(false);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        _gripperCombo = new QComboBox(groupBox);
        _gripperCombo->setObjectName(QString::fromUtf8("_gripperCombo"));

        gridLayout_2->addWidget(_gripperCombo, 2, 1, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 2, 0, 1, 1);

        _robotCombo = new QComboBox(groupBox);
        _robotCombo->setObjectName(QString::fromUtf8("_robotCombo"));

        gridLayout_2->addWidget(_robotCombo, 0, 1, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        _tcpCombo = new QComboBox(groupBox);
        _tcpCombo->setObjectName(QString::fromUtf8("_tcpCombo"));

        gridLayout_2->addWidget(_tcpCombo, 1, 1, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 1, 0, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_2 = new QGroupBox(dockWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout = new QGridLayout(groupBox_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _stopButton = new QPushButton(groupBox_2);
        _stopButton->setObjectName(QString::fromUtf8("_stopButton"));

        gridLayout->addWidget(_stopButton, 2, 3, 1, 1);

        _startButton = new QPushButton(groupBox_2);
        _startButton->setObjectName(QString::fromUtf8("_startButton"));

        gridLayout->addWidget(_startButton, 2, 2, 1, 1);

        _recordingButton = new QPushButton(groupBox_2);
        _recordingButton->setObjectName(QString::fromUtf8("_recordingButton"));

        gridLayout->addWidget(_recordingButton, 3, 2, 1, 2);

        _modeLabel = new QLabel(groupBox_2);
        _modeLabel->setObjectName(QString::fromUtf8("_modeLabel"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        font.setKerning(true);
        _modeLabel->setFont(font);

        gridLayout->addWidget(_modeLabel, 0, 3, 1, 1);

        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 0, 2, 1, 1);

        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 1, 2, 1, 1);

        _wsadLabel = new QLabel(groupBox_2);
        _wsadLabel->setObjectName(QString::fromUtf8("_wsadLabel"));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        _wsadLabel->setFont(font1);

        gridLayout->addWidget(_wsadLabel, 1, 3, 1, 1);


        verticalLayout->addWidget(groupBox_2);


        verticalLayout_2->addLayout(verticalLayout);

        FalconPlugin->setWidget(dockWidgetContents);

        retranslateUi(FalconPlugin);

        QMetaObject::connectSlotsByName(FalconPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *FalconPlugin)
    {
        FalconPlugin->setWindowTitle(QApplication::translate("FalconPlugin", "FalconControl", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("FalconPlugin", "Setup", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("FalconPlugin", "Gripper ctrlr", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("FalconPlugin", "Robot ctrlr", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("FalconPlugin", "TCP frame", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("FalconPlugin", "Simulation", 0, QApplication::UnicodeUTF8));
        _stopButton->setText(QApplication::translate("FalconPlugin", "Stop", 0, QApplication::UnicodeUTF8));
        _startButton->setText(QApplication::translate("FalconPlugin", "Start", 0, QApplication::UnicodeUTF8));
        _recordingButton->setText(QApplication::translate("FalconPlugin", "Start recording", 0, QApplication::UnicodeUTF8));
        _modeLabel->setText(QApplication::translate("FalconPlugin", "GLOBAL", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("FalconPlugin", "Mode:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("FalconPlugin", "WSAD:", 0, QApplication::UnicodeUTF8));
        _wsadLabel->setText(QApplication::translate("FalconPlugin", "ROTATION", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class FalconPlugin: public Ui_FalconPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_FALCONPLUGIN_H
