/********************************************************************************
** Form generated from reading UI file 'SimTaskVisPlugin.ui'
**
** Created: Wed Aug 24 20:31:07 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIMTASKVISPLUGIN_H
#define UI_SIMTASKVISPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDockWidget>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SimTaskVisPlugin
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QPushButton *_loadTaskBtn;
    QPushButton *_updateBtn;
    QSpinBox *_nrOfTargetSpin;
    QComboBox *_frameSelectBox;
    QLabel *label;
    QGridLayout *gridLayout_2;
    QCheckBox *_showEndGraspTargetBox;
    QCheckBox *_showEndLiftTargetBox;
    QCheckBox *_showTargetBox;
    QHBoxLayout *horizontalLayout_2;
    QCheckBox *_showQuality;
    QSpinBox *_qualitySpin;
    QHBoxLayout *horizontalLayout;
    QLabel *label_2;
    QDoubleSpinBox *_zoffsetSpin;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QDoubleSpinBox *_fromThresSpin;
    QDoubleSpinBox *_toThresSpin;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_3;
    QCheckBox *_slippedBox;
    QCheckBox *_successBox;
    QCheckBox *_droppedBox;
    QCheckBox *_untestedBox;
    QCheckBox *_missedBox;
    QCheckBox *_otherBox;
    QCheckBox *_collisionsBox;
    QSpacerItem *verticalSpacer;

    void setupUi(QDockWidget *SimTaskVisPlugin)
    {
        if (SimTaskVisPlugin->objectName().isEmpty())
            SimTaskVisPlugin->setObjectName(QString::fromUtf8("SimTaskVisPlugin"));
        SimTaskVisPlugin->resize(275, 488);
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

        gridLayout->addWidget(_loadTaskBtn, 0, 0, 1, 1);

        _updateBtn = new QPushButton(dockWidgetContents);
        _updateBtn->setObjectName(QString::fromUtf8("_updateBtn"));

        gridLayout->addWidget(_updateBtn, 2, 0, 1, 1);

        _nrOfTargetSpin = new QSpinBox(dockWidgetContents);
        _nrOfTargetSpin->setObjectName(QString::fromUtf8("_nrOfTargetSpin"));
        _nrOfTargetSpin->setMinimum(1);
        _nrOfTargetSpin->setMaximum(1000000);
        _nrOfTargetSpin->setSingleStep(10);
        _nrOfTargetSpin->setValue(1000);

        gridLayout->addWidget(_nrOfTargetSpin, 2, 1, 1, 1);

        _frameSelectBox = new QComboBox(dockWidgetContents);
        _frameSelectBox->setObjectName(QString::fromUtf8("_frameSelectBox"));

        gridLayout->addWidget(_frameSelectBox, 3, 1, 1, 1);

        label = new QLabel(dockWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 3, 0, 1, 1);


        verticalLayout->addLayout(gridLayout);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(-1, -1, -1, 6);
        _showEndGraspTargetBox = new QCheckBox(dockWidgetContents);
        _showEndGraspTargetBox->setObjectName(QString::fromUtf8("_showEndGraspTargetBox"));

        gridLayout_2->addWidget(_showEndGraspTargetBox, 2, 0, 1, 1);

        _showEndLiftTargetBox = new QCheckBox(dockWidgetContents);
        _showEndLiftTargetBox->setObjectName(QString::fromUtf8("_showEndLiftTargetBox"));

        gridLayout_2->addWidget(_showEndLiftTargetBox, 1, 0, 1, 1);

        _showTargetBox = new QCheckBox(dockWidgetContents);
        _showTargetBox->setObjectName(QString::fromUtf8("_showTargetBox"));
        _showTargetBox->setChecked(true);

        gridLayout_2->addWidget(_showTargetBox, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _showQuality = new QCheckBox(dockWidgetContents);
        _showQuality->setObjectName(QString::fromUtf8("_showQuality"));

        horizontalLayout_2->addWidget(_showQuality);

        _qualitySpin = new QSpinBox(dockWidgetContents);
        _qualitySpin->setObjectName(QString::fromUtf8("_qualitySpin"));

        horizontalLayout_2->addWidget(_qualitySpin);


        gridLayout_2->addLayout(horizontalLayout_2, 4, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(-1, 0, -1, -1);
        label_2 = new QLabel(dockWidgetContents);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout->addWidget(label_2);

        _zoffsetSpin = new QDoubleSpinBox(dockWidgetContents);
        _zoffsetSpin->setObjectName(QString::fromUtf8("_zoffsetSpin"));
        _zoffsetSpin->setDecimals(3);
        _zoffsetSpin->setMinimum(-1);
        _zoffsetSpin->setMaximum(1);
        _zoffsetSpin->setSingleStep(0.002);

        horizontalLayout->addWidget(_zoffsetSpin);


        gridLayout_2->addLayout(horizontalLayout, 6, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(-1, 10, -1, -1);
        label_3 = new QLabel(dockWidgetContents);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_3->addWidget(label_3);

        _fromThresSpin = new QDoubleSpinBox(dockWidgetContents);
        _fromThresSpin->setObjectName(QString::fromUtf8("_fromThresSpin"));
        _fromThresSpin->setDecimals(3);
        _fromThresSpin->setMinimum(-10);
        _fromThresSpin->setMaximum(10);
        _fromThresSpin->setSingleStep(0.001);

        horizontalLayout_3->addWidget(_fromThresSpin);

        _toThresSpin = new QDoubleSpinBox(dockWidgetContents);
        _toThresSpin->setObjectName(QString::fromUtf8("_toThresSpin"));
        _toThresSpin->setDecimals(3);
        _toThresSpin->setSingleStep(0.001);
        _toThresSpin->setValue(1);

        horizontalLayout_3->addWidget(_toThresSpin);


        gridLayout_2->addLayout(horizontalLayout_3, 5, 0, 1, 1);


        verticalLayout->addLayout(gridLayout_2);

        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_3 = new QGridLayout(groupBox);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        _slippedBox = new QCheckBox(groupBox);
        _slippedBox->setObjectName(QString::fromUtf8("_slippedBox"));
        _slippedBox->setChecked(true);

        gridLayout_3->addWidget(_slippedBox, 1, 1, 1, 1);

        _successBox = new QCheckBox(groupBox);
        _successBox->setObjectName(QString::fromUtf8("_successBox"));
        _successBox->setChecked(true);

        gridLayout_3->addWidget(_successBox, 0, 1, 1, 1);

        _droppedBox = new QCheckBox(groupBox);
        _droppedBox->setObjectName(QString::fromUtf8("_droppedBox"));
        _droppedBox->setChecked(true);

        gridLayout_3->addWidget(_droppedBox, 0, 0, 1, 1);

        _untestedBox = new QCheckBox(groupBox);
        _untestedBox->setObjectName(QString::fromUtf8("_untestedBox"));
        QPalette palette;
        QBrush brush(QColor(170, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        QBrush brush1(QColor(118, 118, 117, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        _untestedBox->setPalette(palette);
        _untestedBox->setChecked(true);

        gridLayout_3->addWidget(_untestedBox, 3, 1, 1, 1);

        _missedBox = new QCheckBox(groupBox);
        _missedBox->setObjectName(QString::fromUtf8("_missedBox"));
        _missedBox->setChecked(true);

        gridLayout_3->addWidget(_missedBox, 1, 0, 1, 1);

        _otherBox = new QCheckBox(groupBox);
        _otherBox->setObjectName(QString::fromUtf8("_otherBox"));
        _otherBox->setChecked(true);

        gridLayout_3->addWidget(_otherBox, 4, 0, 1, 1);

        _collisionsBox = new QCheckBox(groupBox);
        _collisionsBox->setObjectName(QString::fromUtf8("_collisionsBox"));
        _collisionsBox->setChecked(true);

        gridLayout_3->addWidget(_collisionsBox, 3, 0, 1, 1);


        verticalLayout->addWidget(groupBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        verticalLayout_2->addLayout(verticalLayout);

        SimTaskVisPlugin->setWidget(dockWidgetContents);

        retranslateUi(SimTaskVisPlugin);

        QMetaObject::connectSlotsByName(SimTaskVisPlugin);
    } // setupUi

    void retranslateUi(QDockWidget *SimTaskVisPlugin)
    {
        SimTaskVisPlugin->setWindowTitle(QApplication::translate("SimTaskVisPlugin", "DockWidget", 0, QApplication::UnicodeUTF8));
        _loadTaskBtn->setText(QApplication::translate("SimTaskVisPlugin", "Load tasks", 0, QApplication::UnicodeUTF8));
        _updateBtn->setText(QApplication::translate("SimTaskVisPlugin", "Update", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SimTaskVisPlugin", "Object frame", 0, QApplication::UnicodeUTF8));
        _showEndGraspTargetBox->setText(QApplication::translate("SimTaskVisPlugin", "Show End Grasp", 0, QApplication::UnicodeUTF8));
        _showEndLiftTargetBox->setText(QApplication::translate("SimTaskVisPlugin", "Show End Lift", 0, QApplication::UnicodeUTF8));
        _showTargetBox->setText(QApplication::translate("SimTaskVisPlugin", "Show target", 0, QApplication::UnicodeUTF8));
        _showQuality->setText(QApplication::translate("SimTaskVisPlugin", "Show quality", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SimTaskVisPlugin", "Z Offset", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SimTaskVisPlugin", "Quality threshold", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SimTaskVisPlugin", "Show options", 0, QApplication::UnicodeUTF8));
        _slippedBox->setText(QApplication::translate("SimTaskVisPlugin", "Slipped", 0, QApplication::UnicodeUTF8));
        _successBox->setText(QApplication::translate("SimTaskVisPlugin", "Success", 0, QApplication::UnicodeUTF8));
        _droppedBox->setText(QApplication::translate("SimTaskVisPlugin", "Dropped", 0, QApplication::UnicodeUTF8));
        _untestedBox->setText(QApplication::translate("SimTaskVisPlugin", "Untested", 0, QApplication::UnicodeUTF8));
        _missedBox->setText(QApplication::translate("SimTaskVisPlugin", "Missed", 0, QApplication::UnicodeUTF8));
        _otherBox->setText(QApplication::translate("SimTaskVisPlugin", "Other", 0, QApplication::UnicodeUTF8));
        _collisionsBox->setText(QApplication::translate("SimTaskVisPlugin", "Collisions", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SimTaskVisPlugin: public Ui_SimTaskVisPlugin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIMTASKVISPLUGIN_H
