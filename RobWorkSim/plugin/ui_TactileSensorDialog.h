/********************************************************************************
** Form generated from reading UI file 'TactileSensorDialog.ui'
**
** Created: Thu 28. Jan 14:04:54 2010
**      by: Qt User Interface Compiler version 4.6.0
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
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
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
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_2;
    QFrame *frame;
    QFrame *frame_3;
    QFrame *frame_5;
    QFrame *frame_4;
    QFrame *frame_2;
    QSpacerItem *verticalSpacer_2;
    QLabel *label;
    QDoubleSpinBox *_maxPressureSpin;
    QPushButton *_updateBtn;
    QCheckBox *_autoScaleBox;
    QPushButton *_saveViewBtn;
    QCheckBox *_saveCheckBox;
    QWidget *tab_2;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *_loadDataBtn;
    QPushButton *_saveDataBtn;

    void setupUi(QDialog *TactileSensorDialog)
    {
        if (TactileSensorDialog->objectName().isEmpty())
            TactileSensorDialog->setObjectName(QString::fromUtf8("TactileSensorDialog"));
        TactileSensorDialog->resize(789, 423);
        TactileSensorDialog->setMaximumSize(QSize(789, 568));
        horizontalLayout = new QHBoxLayout(TactileSensorDialog);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        _gview = new QGraphicsView(TactileSensorDialog);
        _gview->setObjectName(QString::fromUtf8("_gview"));

        horizontalLayout->addWidget(_gview);

        tabWidget = new QTabWidget(TactileSensorDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout = new QVBoxLayout(tab);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        frame = new QFrame(tab);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setMinimumSize(QSize(30, 29));
        frame->setMaximumSize(QSize(16777215, 29));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame);

        frame_3 = new QFrame(tab);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        frame_3->setMinimumSize(QSize(0, 28));
        frame_3->setMaximumSize(QSize(16777215, 28));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_3);

        frame_5 = new QFrame(tab);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        frame_5->setMinimumSize(QSize(0, 29));
        frame_5->setMaximumSize(QSize(16777215, 29));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_5);

        frame_4 = new QFrame(tab);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        frame_4->setMinimumSize(QSize(0, 28));
        frame_4->setMaximumSize(QSize(16777215, 28));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_4);

        frame_2 = new QFrame(tab);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setMinimumSize(QSize(0, 29));
        frame_2->setMaximumSize(QSize(16777215, 29));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);

        verticalLayout_2->addWidget(frame_2);


        verticalLayout->addLayout(verticalLayout_2);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);

        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout->addWidget(label);

        _maxPressureSpin = new QDoubleSpinBox(tab);
        _maxPressureSpin->setObjectName(QString::fromUtf8("_maxPressureSpin"));

        verticalLayout->addWidget(_maxPressureSpin);

        _updateBtn = new QPushButton(tab);
        _updateBtn->setObjectName(QString::fromUtf8("_updateBtn"));

        verticalLayout->addWidget(_updateBtn);

        _autoScaleBox = new QCheckBox(tab);
        _autoScaleBox->setObjectName(QString::fromUtf8("_autoScaleBox"));
        _autoScaleBox->setChecked(true);

        verticalLayout->addWidget(_autoScaleBox);

        _saveViewBtn = new QPushButton(tab);
        _saveViewBtn->setObjectName(QString::fromUtf8("_saveViewBtn"));

        verticalLayout->addWidget(_saveViewBtn);

        _saveCheckBox = new QCheckBox(tab);
        _saveCheckBox->setObjectName(QString::fromUtf8("_saveCheckBox"));

        verticalLayout->addWidget(_saveCheckBox);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        horizontalLayout_2 = new QHBoxLayout(tab_2);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _loadDataBtn = new QPushButton(tab_2);
        _loadDataBtn->setObjectName(QString::fromUtf8("_loadDataBtn"));

        horizontalLayout_2->addWidget(_loadDataBtn);

        _saveDataBtn = new QPushButton(tab_2);
        _saveDataBtn->setObjectName(QString::fromUtf8("_saveDataBtn"));

        horizontalLayout_2->addWidget(_saveDataBtn);

        tabWidget->addTab(tab_2, QString());

        horizontalLayout->addWidget(tabWidget);


        retranslateUi(TactileSensorDialog);

        tabWidget->setCurrentIndex(1);


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
        _loadDataBtn->setText(QApplication::translate("TactileSensorDialog", "Load data", 0, QApplication::UnicodeUTF8));
        _saveDataBtn->setText(QApplication::translate("TactileSensorDialog", "Save data", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("TactileSensorDialog", "Tab 2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class TactileSensorDialog: public Ui_TactileSensorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TACTILESENSORDIALOG_H
