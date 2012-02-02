/********************************************************************************
** Form generated from reading UI file 'SDHPlugin.ui'
**
** Created: Tue Jan 31 19:14:57 2012
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SDHPLUGIN_H
#define UI_SDHPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QToolButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SDHPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *_configBtn;
    QPushButton *_connectBtn;
    QPushButton *_disconnectBtn;
    QComboBox *_devListBox;
    QCheckBox *_forceSceneUpdate;
    QCheckBox *_blockMoveBox;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_3;
    QPushButton *_moveToBtn;
    QLabel *label_7;
    QSlider *_velSlider;
    QLabel *label;
    QSlider *_accSlider;
    QLabel *label_6;
    QSlider *_forceSlider;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_5;
    QToolButton *_startBtn;
    QToolButton *_stepBtn;
    QToolButton *_stopBtn;
    QToolButton *_resetBtn;
    QSpacerItem *horizontalSpacer;
    QPushButton *_addTargetBtn;
    QPushButton *_moveTargetBtn;
    QListWidget *_targetListWidget;

    void setupUi(QDockWidget *SDHPlugin)
    {
        if (SDHPlugin->objectName().isEmpty())
            SDHPlugin->setObjectName(QString::fromUtf8("SDHPlugin"));
        SDHPlugin->resize(226, 697);
        SDHPlugin->setAutoFillBackground(false);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        scrollArea = new QScrollArea(dockWidgetContents);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setFrameShape(QFrame::StyledPanel);
        scrollArea->setLineWidth(1);
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 204, 655));
        verticalLayout_4 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMaximumSize(QSize(16777215, 200));
        groupBox->setFlat(false);
        groupBox->setCheckable(false);
        groupBox->setChecked(false);
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(9, -1, -1, -1);
        _configBtn = new QPushButton(groupBox);
        _configBtn->setObjectName(QString::fromUtf8("_configBtn"));

        verticalLayout_2->addWidget(_configBtn);

        _connectBtn = new QPushButton(groupBox);
        _connectBtn->setObjectName(QString::fromUtf8("_connectBtn"));

        verticalLayout_2->addWidget(_connectBtn);

        _disconnectBtn = new QPushButton(groupBox);
        _disconnectBtn->setObjectName(QString::fromUtf8("_disconnectBtn"));

        verticalLayout_2->addWidget(_disconnectBtn);

        _devListBox = new QComboBox(groupBox);
        _devListBox->setObjectName(QString::fromUtf8("_devListBox"));

        verticalLayout_2->addWidget(_devListBox);

        _forceSceneUpdate = new QCheckBox(groupBox);
        _forceSceneUpdate->setObjectName(QString::fromUtf8("_forceSceneUpdate"));
        _forceSceneUpdate->setChecked(true);
        _forceSceneUpdate->setTristate(false);

        verticalLayout_2->addWidget(_forceSceneUpdate);

        _blockMoveBox = new QCheckBox(groupBox);
        _blockMoveBox->setObjectName(QString::fromUtf8("_blockMoveBox"));
        _blockMoveBox->setChecked(true);

        verticalLayout_2->addWidget(_blockMoveBox);


        verticalLayout_4->addWidget(groupBox);

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
        _moveToBtn = new QPushButton(groupBox_2);
        _moveToBtn->setObjectName(QString::fromUtf8("_moveToBtn"));

        verticalLayout_3->addWidget(_moveToBtn);

        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_3->addWidget(label_7);

        _velSlider = new QSlider(groupBox_2);
        _velSlider->setObjectName(QString::fromUtf8("_velSlider"));
        _velSlider->setMaximum(99);
        _velSlider->setSingleStep(0);
        _velSlider->setValue(40);
        _velSlider->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(_velSlider);

        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_3->addWidget(label);

        _accSlider = new QSlider(groupBox_2);
        _accSlider->setObjectName(QString::fromUtf8("_accSlider"));
        _accSlider->setValue(20);
        _accSlider->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(_accSlider);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_3->addWidget(label_6);

        _forceSlider = new QSlider(groupBox_2);
        _forceSlider->setObjectName(QString::fromUtf8("_forceSlider"));
        _forceSlider->setValue(20);
        _forceSlider->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(_forceSlider);


        verticalLayout_4->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(scrollAreaWidgetContents);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        verticalLayout_7 = new QVBoxLayout(groupBox_3);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        _startBtn = new QToolButton(groupBox_3);
        _startBtn->setObjectName(QString::fromUtf8("_startBtn"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/forward.png"), QSize(), QIcon::Selected, QIcon::Off);
        _startBtn->setIcon(icon);

        horizontalLayout_5->addWidget(_startBtn);

        _stepBtn = new QToolButton(groupBox_3);
        _stepBtn->setObjectName(QString::fromUtf8("_stepBtn"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/end.png"), QSize(), QIcon::Selected, QIcon::Off);
        _stepBtn->setIcon(icon1);

        horizontalLayout_5->addWidget(_stepBtn);

        _stopBtn = new QToolButton(groupBox_3);
        _stopBtn->setObjectName(QString::fromUtf8("_stopBtn"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/pause.png"), QSize(), QIcon::Selected, QIcon::Off);
        _stopBtn->setIcon(icon2);

        horizontalLayout_5->addWidget(_stopBtn);

        _resetBtn = new QToolButton(groupBox_3);
        _resetBtn->setObjectName(QString::fromUtf8("_resetBtn"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/reload.png"), QSize(), QIcon::Selected, QIcon::Off);
        _resetBtn->setIcon(icon3);

        horizontalLayout_5->addWidget(_resetBtn);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_7->addLayout(horizontalLayout_5);

        _addTargetBtn = new QPushButton(groupBox_3);
        _addTargetBtn->setObjectName(QString::fromUtf8("_addTargetBtn"));

        verticalLayout_7->addWidget(_addTargetBtn);

        _moveTargetBtn = new QPushButton(groupBox_3);
        _moveTargetBtn->setObjectName(QString::fromUtf8("_moveTargetBtn"));

        verticalLayout_7->addWidget(_moveTargetBtn);

        _targetListWidget = new QListWidget(groupBox_3);
        _targetListWidget->setObjectName(QString::fromUtf8("_targetListWidget"));
        _targetListWidget->setDragEnabled(true);

        verticalLayout_7->addWidget(_targetListWidget);


        verticalLayout_4->addWidget(groupBox_3);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        SDHPlugin->setWidget(dockWidgetContents);
        QWidget::setTabOrder(_configBtn, _moveToBtn);

        retranslateUi(SDHPlugin);

        QMetaObject::connectSlotsByName(SDHPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SDHPlugin)
    {
        SDHPlugin->setWindowTitle(QApplication::translate("SDHPlugin", "RW SDH", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SDHPlugin", "Control", 0, QApplication::UnicodeUTF8));
        _configBtn->setText(QApplication::translate("SDHPlugin", "Config", 0, QApplication::UnicodeUTF8));
        _connectBtn->setText(QApplication::translate("SDHPlugin", "Connect", 0, QApplication::UnicodeUTF8));
        _disconnectBtn->setText(QApplication::translate("SDHPlugin", "Disconnect", 0, QApplication::UnicodeUTF8));
        _forceSceneUpdate->setText(QApplication::translate("SDHPlugin", "Force scene update", 0, QApplication::UnicodeUTF8));
        _blockMoveBox->setText(QApplication::translate("SDHPlugin", "Blocking move", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SDHPlugin", "Config", 0, QApplication::UnicodeUTF8));
        _moveToBtn->setText(QApplication::translate("SDHPlugin", "Move to State", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SDHPlugin", "Velocity", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SDHPlugin", "Acceleration", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SDHPlugin", "Force", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SDHPlugin", "Teach control", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _startBtn->setToolTip(QApplication::translate("SDHPlugin", "Start/resume simulation", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _startBtn->setText(QString());
#ifndef QT_NO_TOOLTIP
        _stepBtn->setToolTip(QApplication::translate("SDHPlugin", "Step simulation one dt", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _stepBtn->setText(QApplication::translate("SDHPlugin", "...", 0, QApplication::UnicodeUTF8));
        _stopBtn->setText(QApplication::translate("SDHPlugin", "...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _resetBtn->setToolTip(QApplication::translate("SDHPlugin", "Reset the state of the simulator to \n"
"that of RobWorkStudio", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        _resetBtn->setText(QApplication::translate("SDHPlugin", "...", 0, QApplication::UnicodeUTF8));
        _addTargetBtn->setText(QApplication::translate("SDHPlugin", "Add target", 0, QApplication::UnicodeUTF8));
        _moveTargetBtn->setText(QApplication::translate("SDHPlugin", "Move to target", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SDHPlugin: public Ui_SDHPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SDHPLUGIN_H
