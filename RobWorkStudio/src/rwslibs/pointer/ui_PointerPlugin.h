/********************************************************************************
** Form generated from reading UI file 'PointerPlugin.ui'
**
** Created: Mon Mar 24 09:30:44 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTERPLUGIN_H
#define UI_POINTERPLUGIN_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PointerPluginWidget
{
public:
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QRadioButton *toolButton;
    QRadioButton *worldButton;
    QRadioButton *viewButton;
    QRadioButton *rotationButton;
    QPushButton *saveButton;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QLineEdit *xEdit;
    QLabel *label;
    QLabel *label_4;
    QLineEdit *yEdit;
    QLabel *label_2;
    QLineEdit *zEdit;
    QLabel *label_3;
    QLineEdit *rollEdit;
    QLabel *label_5;
    QLabel *label_6;
    QLineEdit *pitchEdit;
    QLineEdit *yawEdit;
    QPushButton *action1Button;
    QCheckBox *showBox;
    QCheckBox *keyboardBox;
    QPushButton *action2Button;
    QSpacerItem *verticalSpacer;
    QPushButton *originButton;
    QPushButton *restoreButton;
    QCheckBox *lockBox;
    QSlider *speedSlider;

    void setupUi(QDockWidget *PointerPluginWidget)
    {
        if (PointerPluginWidget->objectName().isEmpty())
            PointerPluginWidget->setObjectName(QString::fromUtf8("PointerPluginWidget"));
        PointerPluginWidget->resize(467, 592);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        groupBox_2 = new QGroupBox(dockWidgetContents);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        toolButton = new QRadioButton(groupBox_2);
        toolButton->setObjectName(QString::fromUtf8("toolButton"));

        gridLayout_3->addWidget(toolButton, 0, 1, 1, 1);

        worldButton = new QRadioButton(groupBox_2);
        worldButton->setObjectName(QString::fromUtf8("worldButton"));
        worldButton->setChecked(true);

        gridLayout_3->addWidget(worldButton, 0, 0, 1, 1);

        viewButton = new QRadioButton(groupBox_2);
        viewButton->setObjectName(QString::fromUtf8("viewButton"));

        gridLayout_3->addWidget(viewButton, 1, 0, 1, 1);

        rotationButton = new QRadioButton(groupBox_2);
        rotationButton->setObjectName(QString::fromUtf8("rotationButton"));

        gridLayout_3->addWidget(rotationButton, 1, 1, 1, 1);


        gridLayout->addWidget(groupBox_2, 6, 0, 1, 2);

        saveButton = new QPushButton(dockWidgetContents);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));

        gridLayout->addWidget(saveButton, 3, 0, 1, 1);

        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        xEdit = new QLineEdit(groupBox);
        xEdit->setObjectName(QString::fromUtf8("xEdit"));
        xEdit->setReadOnly(true);

        gridLayout_2->addWidget(xEdit, 0, 1, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 0, 0, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_2->addWidget(label_4, 0, 2, 1, 1);

        yEdit = new QLineEdit(groupBox);
        yEdit->setObjectName(QString::fromUtf8("yEdit"));
        yEdit->setReadOnly(true);

        gridLayout_2->addWidget(yEdit, 1, 1, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_2->addWidget(label_2, 1, 0, 1, 1);

        zEdit = new QLineEdit(groupBox);
        zEdit->setObjectName(QString::fromUtf8("zEdit"));
        zEdit->setReadOnly(true);

        gridLayout_2->addWidget(zEdit, 2, 1, 1, 1);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_2->addWidget(label_3, 2, 0, 1, 1);

        rollEdit = new QLineEdit(groupBox);
        rollEdit->setObjectName(QString::fromUtf8("rollEdit"));
        rollEdit->setReadOnly(true);

        gridLayout_2->addWidget(rollEdit, 0, 3, 1, 1);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_2->addWidget(label_5, 1, 2, 1, 1);

        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 2, 2, 1, 1);

        pitchEdit = new QLineEdit(groupBox);
        pitchEdit->setObjectName(QString::fromUtf8("pitchEdit"));
        pitchEdit->setReadOnly(true);

        gridLayout_2->addWidget(pitchEdit, 1, 3, 1, 1);

        yawEdit = new QLineEdit(groupBox);
        yawEdit->setObjectName(QString::fromUtf8("yawEdit"));
        yawEdit->setReadOnly(true);

        gridLayout_2->addWidget(yawEdit, 2, 3, 1, 1);


        gridLayout->addWidget(groupBox, 0, 0, 1, 2);

        action1Button = new QPushButton(dockWidgetContents);
        action1Button->setObjectName(QString::fromUtf8("action1Button"));

        gridLayout->addWidget(action1Button, 8, 0, 1, 1);

        showBox = new QCheckBox(dockWidgetContents);
        showBox->setObjectName(QString::fromUtf8("showBox"));
        showBox->setEnabled(true);
        showBox->setChecked(false);

        gridLayout->addWidget(showBox, 4, 0, 1, 1);

        keyboardBox = new QCheckBox(dockWidgetContents);
        keyboardBox->setObjectName(QString::fromUtf8("keyboardBox"));
        keyboardBox->setEnabled(true);
        keyboardBox->setChecked(false);

        gridLayout->addWidget(keyboardBox, 4, 1, 1, 1);

        action2Button = new QPushButton(dockWidgetContents);
        action2Button->setObjectName(QString::fromUtf8("action2Button"));

        gridLayout->addWidget(action2Button, 8, 1, 1, 1);

        verticalSpacer = new QSpacerItem(20, 7, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 9, 0, 1, 1);

        originButton = new QPushButton(dockWidgetContents);
        originButton->setObjectName(QString::fromUtf8("originButton"));

        gridLayout->addWidget(originButton, 2, 0, 1, 2);

        restoreButton = new QPushButton(dockWidgetContents);
        restoreButton->setObjectName(QString::fromUtf8("restoreButton"));

        gridLayout->addWidget(restoreButton, 3, 1, 1, 1);

        lockBox = new QCheckBox(dockWidgetContents);
        lockBox->setObjectName(QString::fromUtf8("lockBox"));
        lockBox->setChecked(false);

        gridLayout->addWidget(lockBox, 5, 0, 1, 1);

        speedSlider = new QSlider(dockWidgetContents);
        speedSlider->setObjectName(QString::fromUtf8("speedSlider"));
        speedSlider->setMinimum(-10);
        speedSlider->setMaximum(10);
        speedSlider->setPageStep(1);
        speedSlider->setOrientation(Qt::Horizontal);
        speedSlider->setTickPosition(QSlider::TicksBelow);
        speedSlider->setTickInterval(1);

        gridLayout->addWidget(speedSlider, 1, 0, 1, 2);

        PointerPluginWidget->setWidget(dockWidgetContents);

        retranslateUi(PointerPluginWidget);

        QMetaObject::connectSlotsByName(PointerPluginWidget);
    } // setupUi

    void retranslateUi(QDockWidget *PointerPluginWidget)
    {
#ifndef QT_NO_TOOLTIP
        PointerPluginWidget->setToolTip(QApplication::translate("PointerPluginWidget", "By Dagothar", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        PointerPluginWidget->setWindowTitle(QApplication::translate("PointerPluginWidget", "Pointer", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("PointerPluginWidget", "Mode", 0, QApplication::UnicodeUTF8));
        toolButton->setText(QApplication::translate("PointerPluginWidget", "TOOL (C)", 0, QApplication::UnicodeUTF8));
        worldButton->setText(QApplication::translate("PointerPluginWidget", "WORLD (Z)", 0, QApplication::UnicodeUTF8));
        viewButton->setText(QApplication::translate("PointerPluginWidget", "VIEW (X)", 0, QApplication::UnicodeUTF8));
        rotationButton->setText(QApplication::translate("PointerPluginWidget", "ROTATION (V)", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("PointerPluginWidget", "Save position (R)", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("PointerPluginWidget", "Pointer pose", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("PointerPluginWidget", "x", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("PointerPluginWidget", "R", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("PointerPluginWidget", "y", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("PointerPluginWidget", "z", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("PointerPluginWidget", "P", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("PointerPluginWidget", "Y", 0, QApplication::UnicodeUTF8));
        action1Button->setText(QApplication::translate("PointerPluginWidget", "Action1 (F)", 0, QApplication::UnicodeUTF8));
        showBox->setText(QApplication::translate("PointerPluginWidget", "Show pointer frame", 0, QApplication::UnicodeUTF8));
        keyboardBox->setText(QApplication::translate("PointerPluginWidget", "Enable keyboard", 0, QApplication::UnicodeUTF8));
        action2Button->setText(QApplication::translate("PointerPluginWidget", "Action2 (G)", 0, QApplication::UnicodeUTF8));
        originButton->setText(QApplication::translate("PointerPluginWidget", "Return to origin (O)", 0, QApplication::UnicodeUTF8));
        restoreButton->setText(QApplication::translate("PointerPluginWidget", "Restore position (T)", 0, QApplication::UnicodeUTF8));
        lockBox->setText(QApplication::translate("PointerPluginWidget", "Lock view", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        speedSlider->setToolTip(QApplication::translate("PointerPluginWidget", "pointer speed", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
    } // retranslateUi

};

namespace Ui {
    class PointerPluginWidget: public Ui_PointerPluginWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTERPLUGIN_H
