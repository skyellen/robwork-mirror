/********************************************************************************
** Form generated from reading UI file 'SimCfgDialog.ui'
**
** Created: Mon Aug 1 12:19:16 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SIMCFGDIALOG_H
#define UI_SIMCFGDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SimCfgDialog
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *_tabPane;
    QWidget *General;
    QLabel *label;
    QDoubleSpinBox *_xGravitySpin;
    QDoubleSpinBox *_zGravitySpin;
    QDoubleSpinBox *_yGravitySpin;
    QWidget *Material;
    QWidget *RWPhysics;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *_applyBtn;
    QPushButton *_cancelBtn;

    void setupUi(QDialog *SimCfgDialog)
    {
        if (SimCfgDialog->objectName().isEmpty())
            SimCfgDialog->setObjectName(QString::fromUtf8("SimCfgDialog"));
        SimCfgDialog->resize(645, 429);
        verticalLayout = new QVBoxLayout(SimCfgDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        _tabPane = new QTabWidget(SimCfgDialog);
        _tabPane->setObjectName(QString::fromUtf8("_tabPane"));
        General = new QWidget();
        General->setObjectName(QString::fromUtf8("General"));
        label = new QLabel(General);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 20, 81, 16));
        _xGravitySpin = new QDoubleSpinBox(General);
        _xGravitySpin->setObjectName(QString::fromUtf8("_xGravitySpin"));
        _xGravitySpin->setGeometry(QRect(100, 20, 62, 22));
        _zGravitySpin = new QDoubleSpinBox(General);
        _zGravitySpin->setObjectName(QString::fromUtf8("_zGravitySpin"));
        _zGravitySpin->setGeometry(QRect(240, 20, 62, 22));
        _zGravitySpin->setMinimum(-99.99);
        _zGravitySpin->setValue(-9.82);
        _yGravitySpin = new QDoubleSpinBox(General);
        _yGravitySpin->setObjectName(QString::fromUtf8("_yGravitySpin"));
        _yGravitySpin->setGeometry(QRect(170, 20, 62, 22));
        _tabPane->addTab(General, QString());
        Material = new QWidget();
        Material->setObjectName(QString::fromUtf8("Material"));
        _tabPane->addTab(Material, QString());
        RWPhysics = new QWidget();
        RWPhysics->setObjectName(QString::fromUtf8("RWPhysics"));
        _tabPane->addTab(RWPhysics, QString());

        verticalLayout->addWidget(_tabPane);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        _applyBtn = new QPushButton(SimCfgDialog);
        _applyBtn->setObjectName(QString::fromUtf8("_applyBtn"));

        horizontalLayout->addWidget(_applyBtn);

        _cancelBtn = new QPushButton(SimCfgDialog);
        _cancelBtn->setObjectName(QString::fromUtf8("_cancelBtn"));

        horizontalLayout->addWidget(_cancelBtn);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(SimCfgDialog);

        _tabPane->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(SimCfgDialog);
    } // setupUi

    void retranslateUi(QDialog *SimCfgDialog)
    {
        SimCfgDialog->setWindowTitle(QApplication::translate("SimCfgDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SimCfgDialog", "Gravity (x,y,z): ", 0, QApplication::UnicodeUTF8));
        _tabPane->setTabText(_tabPane->indexOf(General), QApplication::translate("SimCfgDialog", "General", 0, QApplication::UnicodeUTF8));
        _tabPane->setTabText(_tabPane->indexOf(Material), QApplication::translate("SimCfgDialog", "Material", 0, QApplication::UnicodeUTF8));
        _tabPane->setTabText(_tabPane->indexOf(RWPhysics), QApplication::translate("SimCfgDialog", "RWPhysics", 0, QApplication::UnicodeUTF8));
        _applyBtn->setText(QApplication::translate("SimCfgDialog", "Apply", 0, QApplication::UnicodeUTF8));
        _cancelBtn->setText(QApplication::translate("SimCfgDialog", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SimCfgDialog: public Ui_SimCfgDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SIMCFGDIALOG_H
