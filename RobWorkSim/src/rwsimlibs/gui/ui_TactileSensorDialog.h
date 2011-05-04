/********************************************************************************
** Form generated from reading UI file 'TactileSensorDialog.ui'
**
** Created: Tue 26. Apr 01:15:26 2011
**      by: Qt User Interface Compiler version 4.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TACTILESENSORDIALOG_H
#define UI_TACTILESENSORDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGraphicsView>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QScrollArea>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TactileSensorDialog
{
public:
    QHBoxLayout *horizontalLayout;
    QGraphicsView *_gview;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QFrame *frame;
    QFrame *frame_3;
    QFrame *frame_5;
    QFrame *frame_4;
    QFrame *frame_2;
    QLabel *label;
    QDoubleSpinBox *_maxPressureSpin;
    QPushButton *_updateBtn;
    QCheckBox *_autoScaleBox;
    QPushButton *_saveViewBtn;
    QCheckBox *_saveCheckBox;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_6;
    QCheckBox *_centerOfMassBtn;
    QCheckBox *_momentsBtn;
    QCheckBox *_segmentationBtn;
    QHBoxLayout *horizontalLayout_2;
    QSpinBox *_lowBoundSpin;
    QSpinBox *_uppBoundSpin;
    QSpacerItem *verticalSpacer;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_4;
    QSpacerItem *verticalSpacer_2;
    QPushButton *_saveDataBtn;
    QPushButton *_loadDataBtn;

    void setupUi(QDialog *TactileSensorDialog)
    {
        if (TactileSensorDialog->objectName().isEmpty())
            TactileSensorDialog->setObjectName(QString::fromUtf8("TactileSensorDialog"));
        TactileSensorDialog->resize(763, 508);
        TactileSensorDialog->setMaximumSize(QSize(789, 568));
        horizontalLayout = new QHBoxLayout(TactileSensorDialog);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        _gview = new QGraphicsView(TactileSensorDialog);
        _gview->setObjectName(QString::fromUtf8("_gview"));

        horizontalLayout->addWidget(_gview);

        tabWidget = new QTabWidget(TactileSensorDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tabWidget->setMinimumSize(QSize(200, 0));
        tabWidget->setMovable(false);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout = new QVBoxLayout(tab);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        scrollArea = new QScrollArea(tab);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 174, 444));
        verticalLayout_3 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        frame = new QFrame(scrollAreaWidgetContents);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(30, 29));
        frame->setMaximumSize(QSize(16777215, 29));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame);

        frame_3 = new QFrame(scrollAreaWidgetContents);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setMinimumSize(QSize(0, 28));
        frame_3->setMaximumSize(QSize(16777215, 28));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_3);

        frame_5 = new QFrame(scrollAreaWidgetContents);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        frame_5->setMinimumSize(QSize(0, 29));
        frame_5->setMaximumSize(QSize(16777215, 29));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_5);

        frame_4 = new QFrame(scrollAreaWidgetContents);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        frame_4->setMinimumSize(QSize(0, 28));
        frame_4->setMaximumSize(QSize(16777215, 28));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_4);

        frame_2 = new QFrame(scrollAreaWidgetContents);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMinimumSize(QSize(0, 29));
        frame_2->setMaximumSize(QSize(16777215, 29));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_2);


        verticalLayout_3->addLayout(verticalLayout_2);

        label = new QLabel(scrollAreaWidgetContents);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_3->addWidget(label);

        _maxPressureSpin = new QDoubleSpinBox(scrollAreaWidgetContents);
        _maxPressureSpin->setObjectName(QString::fromUtf8("_maxPressureSpin"));

        verticalLayout_3->addWidget(_maxPressureSpin);

        _updateBtn = new QPushButton(scrollAreaWidgetContents);
        _updateBtn->setObjectName(QString::fromUtf8("_updateBtn"));

        verticalLayout_3->addWidget(_updateBtn);

        _autoScaleBox = new QCheckBox(scrollAreaWidgetContents);
        _autoScaleBox->setObjectName(QString::fromUtf8("_autoScaleBox"));
        _autoScaleBox->setChecked(true);

        verticalLayout_3->addWidget(_autoScaleBox);

        _saveViewBtn = new QPushButton(scrollAreaWidgetContents);
        _saveViewBtn->setObjectName(QString::fromUtf8("_saveViewBtn"));

        verticalLayout_3->addWidget(_saveViewBtn);

        _saveCheckBox = new QCheckBox(scrollAreaWidgetContents);
        _saveCheckBox->setObjectName(QString::fromUtf8("_saveCheckBox"));

        verticalLayout_3->addWidget(_saveCheckBox);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        tabWidget->addTab(tab, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        verticalLayout_5 = new QVBoxLayout(tab_3);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        groupBox = new QGroupBox(tab_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        verticalLayout_6 = new QVBoxLayout(groupBox);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        _centerOfMassBtn = new QCheckBox(groupBox);
        _centerOfMassBtn->setObjectName(QString::fromUtf8("_centerOfMassBtn"));

        verticalLayout_6->addWidget(_centerOfMassBtn);

        _momentsBtn = new QCheckBox(groupBox);
        _momentsBtn->setObjectName(QString::fromUtf8("_momentsBtn"));

        verticalLayout_6->addWidget(_momentsBtn);

        _segmentationBtn = new QCheckBox(groupBox);
        _segmentationBtn->setObjectName(QString::fromUtf8("_segmentationBtn"));

        verticalLayout_6->addWidget(_segmentationBtn);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _lowBoundSpin = new QSpinBox(groupBox);
        _lowBoundSpin->setObjectName(QString::fromUtf8("_lowBoundSpin"));
        _lowBoundSpin->setMaximum(300000);
        _lowBoundSpin->setSingleStep(10);

        horizontalLayout_2->addWidget(_lowBoundSpin);

        _uppBoundSpin = new QSpinBox(groupBox);
        _uppBoundSpin->setObjectName(QString::fromUtf8("_uppBoundSpin"));
        _uppBoundSpin->setMaximum(300000);
        _uppBoundSpin->setSingleStep(10);
        _uppBoundSpin->setValue(300000);

        horizontalLayout_2->addWidget(_uppBoundSpin);


        verticalLayout_6->addLayout(horizontalLayout_2);


        verticalLayout_5->addWidget(groupBox);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_5->addItem(verticalSpacer);

        tabWidget->addTab(tab_3, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_4 = new QVBoxLayout(tab_2);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_2);

        _saveDataBtn = new QPushButton(tab_2);
        _saveDataBtn->setObjectName(QString::fromUtf8("_saveDataBtn"));

        verticalLayout_4->addWidget(_saveDataBtn);

        _loadDataBtn = new QPushButton(tab_2);
        _loadDataBtn->setObjectName(QString::fromUtf8("_loadDataBtn"));

        verticalLayout_4->addWidget(_loadDataBtn);

        tabWidget->addTab(tab_2, QString());

        horizontalLayout->addWidget(tabWidget);


        retranslateUi(TactileSensorDialog);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(TactileSensorDialog);
    } // setupUi

    void retranslateUi(QDialog *TactileSensorDialog)
    {
        TactileSensorDialog->setWindowTitle(QApplication::translate("TactileSensorDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("TactileSensorDialog", "Max scale pressure in Pa", 0, QApplication::UnicodeUTF8));
        _updateBtn->setText(QApplication::translate("TactileSensorDialog", "Update auto", 0, QApplication::UnicodeUTF8));
        _autoScaleBox->setText(QApplication::translate("TactileSensorDialog", "Auto scale", 0, QApplication::UnicodeUTF8));
        _saveViewBtn->setText(QApplication::translate("TactileSensorDialog", "Save image", 0, QApplication::UnicodeUTF8));
        _saveCheckBox->setText(QApplication::translate("TactileSensorDialog", "Save on update event", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("TactileSensorDialog", "Palette", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("TactileSensorDialog", "Static features", 0, QApplication::UnicodeUTF8));
        _centerOfMassBtn->setText(QApplication::translate("TactileSensorDialog", "Center of mass", 0, QApplication::UnicodeUTF8));
        _momentsBtn->setText(QApplication::translate("TactileSensorDialog", "Moments", 0, QApplication::UnicodeUTF8));
        _segmentationBtn->setText(QApplication::translate("TactileSensorDialog", "Segmentation", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QApplication::translate("TactileSensorDialog", "Features", 0, QApplication::UnicodeUTF8));
        _saveDataBtn->setText(QApplication::translate("TactileSensorDialog", "Save data", 0, QApplication::UnicodeUTF8));
        _loadDataBtn->setText(QApplication::translate("TactileSensorDialog", "Load data", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("TactileSensorDialog", "Data", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TactileSensorDialog: public Ui_TactileSensorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TACTILESENSORDIALOG_H
