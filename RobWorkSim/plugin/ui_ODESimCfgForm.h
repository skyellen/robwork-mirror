/********************************************************************************
** Form generated from reading UI file 'ODESimCfgForm.ui'
**
** Created: Thu 28. Jan 14:04:54 2010
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ODESIMCFGFORM_H
#define UI_ODESIMCFGFORM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ODESimCfgForm
{
public:
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *General;
    QLabel *label;
    QComboBox *_stepMethodBox;
    QDoubleSpinBox *_cfmSpin;
    QDoubleSpinBox *_marginSpin;
    QComboBox *_spaceMethodBox;
    QComboBox *_clusterAlgBox;
    QSpinBox *_maxIterSpin;
    QLabel *label_3;
    QLabel *label_6;
    QLabel *label_25;
    QLabel *label_2;
    QLabel *label_4;
    QDoubleSpinBox *_erpSpin;
    QLabel *label_5;
    QWidget *Objects;
    QHBoxLayout *horizontalLayout_2;
    QListWidget *_objListWidget;
    QGroupBox *groupBox;
    QLabel *label_18;
    QLabel *label_17;
    QLabel *label_20;
    QLabel *_selectedName;
    QDoubleSpinBox *_crThresSpin;
    QCheckBox *_enabledBox;
    QLabel *label_22;
    QLabel *label_13;
    QLabel *label_7;
    QLabel *label_16;
    QComboBox *_materialBox;
    QLabel *label_12;
    QLabel *label_14;
    QLabel *label_21;
    QLabel *label_15;
    QLabel *_objectTypeName;
    QLabel *label_19;
    QWidget *Devices;
    QLabel *label_24;
    QListView *listView_2;
    QLabel *_objectTypeName_2;
    QLabel *label_23;
    QLabel *_selectedName_2;
    QWidget *Contact;
    QListView *_objectListB;
    QLabel *label_10;
    QLabel *label_11;
    QDoubleSpinBox *_erpSpin_2;
    QListView *_objectListA;
    QLabel *label_8;
    QDoubleSpinBox *_cfmSpin_2;
    QLabel *label_9;
    QWidget *ExcludeList;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer;
    QPushButton *_applyBtn;
    QPushButton *_cancelBtn;

    void setupUi(QWidget *ODESimCfgForm)
    {
        if (ODESimCfgForm->objectName().isEmpty())
            ODESimCfgForm->setObjectName(QString::fromUtf8("ODESimCfgForm"));
        ODESimCfgForm->resize(605, 425);
        verticalLayout = new QVBoxLayout(ODESimCfgForm);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        tabWidget = new QTabWidget(ODESimCfgForm);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        General = new QWidget();
        General->setObjectName(QString::fromUtf8("General"));
        label = new QLabel(General);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 20, 71, 21));
        _stepMethodBox = new QComboBox(General);
        _stepMethodBox->setObjectName(QString::fromUtf8("_stepMethodBox"));
        _stepMethodBox->setGeometry(QRect(100, 20, 141, 22));
        _cfmSpin = new QDoubleSpinBox(General);
        _cfmSpin->setObjectName(QString::fromUtf8("_cfmSpin"));
        _cfmSpin->setGeometry(QRect(120, 110, 62, 22));
        _cfmSpin->setDecimals(2);
        _cfmSpin->setMaximum(1);
        _cfmSpin->setSingleStep(0.1);
        _cfmSpin->setValue(0.6);
        _marginSpin = new QDoubleSpinBox(General);
        _marginSpin->setObjectName(QString::fromUtf8("_marginSpin"));
        _marginSpin->setGeometry(QRect(120, 170, 71, 22));
        _marginSpin->setDecimals(5);
        _marginSpin->setMaximum(1);
        _marginSpin->setSingleStep(0.01);
        _marginSpin->setValue(0.001);
        _spaceMethodBox = new QComboBox(General);
        _spaceMethodBox->setObjectName(QString::fromUtf8("_spaceMethodBox"));
        _spaceMethodBox->setGeometry(QRect(240, 70, 141, 21));
        _clusterAlgBox = new QComboBox(General);
        _clusterAlgBox->setObjectName(QString::fromUtf8("_clusterAlgBox"));
        _clusterAlgBox->setGeometry(QRect(140, 210, 141, 21));
        _maxIterSpin = new QSpinBox(General);
        _maxIterSpin->setObjectName(QString::fromUtf8("_maxIterSpin"));
        _maxIterSpin->setGeometry(QRect(360, 20, 46, 22));
        _maxIterSpin->setMaximum(1000);
        _maxIterSpin->setValue(20);
        label_3 = new QLabel(General);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 110, 71, 21));
        label_6 = new QLabel(General);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(260, 20, 81, 21));
        label_25 = new QLabel(General);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(10, 170, 81, 21));
        label_2 = new QLabel(General);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(10, 70, 201, 21));
        label_4 = new QLabel(General);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 140, 71, 21));
        _erpSpin = new QDoubleSpinBox(General);
        _erpSpin->setObjectName(QString::fromUtf8("_erpSpin"));
        _erpSpin->setGeometry(QRect(120, 140, 62, 22));
        _erpSpin->setMaximum(1);
        _erpSpin->setSingleStep(0.1);
        _erpSpin->setValue(0.3);
        label_5 = new QLabel(General);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 210, 111, 21));
        tabWidget->addTab(General, QString());
        Objects = new QWidget();
        Objects->setObjectName(QString::fromUtf8("Objects"));
        horizontalLayout_2 = new QHBoxLayout(Objects);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _objListWidget = new QListWidget(Objects);
        _objListWidget->setObjectName(QString::fromUtf8("_objListWidget"));

        horizontalLayout_2->addWidget(_objListWidget);

        groupBox = new QGroupBox(Objects);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setMinimumSize(QSize(281, 331));
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 260, 91, 21));
        label_17 = new QLabel(groupBox);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 240, 81, 21));
        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(10, 300, 101, 21));
        _selectedName = new QLabel(groupBox);
        _selectedName->setObjectName(QString::fromUtf8("_selectedName"));
        _selectedName->setGeometry(QRect(60, 10, 171, 21));
        _crThresSpin = new QDoubleSpinBox(groupBox);
        _crThresSpin->setObjectName(QString::fromUtf8("_crThresSpin"));
        _crThresSpin->setGeometry(QRect(150, 120, 62, 22));
        _crThresSpin->setDecimals(3);
        _crThresSpin->setSingleStep(0.001);
        _enabledBox = new QCheckBox(groupBox);
        _enabledBox->setObjectName(QString::fromUtf8("_enabledBox"));
        _enabledBox->setGeometry(QRect(10, 50, 71, 19));
        label_22 = new QLabel(groupBox);
        label_22->setObjectName(QString::fromUtf8("label_22"));
        label_22->setGeometry(QRect(10, 170, 61, 21));
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(10, 120, 141, 21));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 10, 46, 21));
        label_16 = new QLabel(groupBox);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(10, 220, 61, 21));
        _materialBox = new QComboBox(groupBox);
        _materialBox->setObjectName(QString::fromUtf8("_materialBox"));
        _materialBox->setGeometry(QRect(80, 80, 74, 22));
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(10, 80, 51, 21));
        label_14 = new QLabel(groupBox);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(10, 30, 71, 21));
        label_21 = new QLabel(groupBox);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(10, 150, 61, 21));
        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 200, 61, 21));
        _objectTypeName = new QLabel(groupBox);
        _objectTypeName->setObjectName(QString::fromUtf8("_objectTypeName"));
        _objectTypeName->setGeometry(QRect(80, 30, 71, 21));
        label_19 = new QLabel(groupBox);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(10, 280, 101, 21));

        horizontalLayout_2->addWidget(groupBox);

        tabWidget->addTab(Objects, QString());
        Devices = new QWidget();
        Devices->setObjectName(QString::fromUtf8("Devices"));
        label_24 = new QLabel(Devices);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setGeometry(QRect(190, 30, 71, 21));
        listView_2 = new QListView(Devices);
        listView_2->setObjectName(QString::fromUtf8("listView_2"));
        listView_2->setGeometry(QRect(20, 10, 151, 321));
        _objectTypeName_2 = new QLabel(Devices);
        _objectTypeName_2->setObjectName(QString::fromUtf8("_objectTypeName_2"));
        _objectTypeName_2->setGeometry(QRect(260, 30, 71, 21));
        label_23 = new QLabel(Devices);
        label_23->setObjectName(QString::fromUtf8("label_23"));
        label_23->setGeometry(QRect(190, 10, 46, 21));
        _selectedName_2 = new QLabel(Devices);
        _selectedName_2->setObjectName(QString::fromUtf8("_selectedName_2"));
        _selectedName_2->setGeometry(QRect(240, 10, 171, 21));
        tabWidget->addTab(Devices, QString());
        Contact = new QWidget();
        Contact->setObjectName(QString::fromUtf8("Contact"));
        _objectListB = new QListView(Contact);
        _objectListB->setObjectName(QString::fromUtf8("_objectListB"));
        _objectListB->setGeometry(QRect(120, 40, 111, 281));
        label_10 = new QLabel(Contact);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(10, 10, 71, 21));
        label_11 = new QLabel(Contact);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(130, 10, 71, 21));
        _erpSpin_2 = new QDoubleSpinBox(Contact);
        _erpSpin_2->setObjectName(QString::fromUtf8("_erpSpin_2"));
        _erpSpin_2->setGeometry(QRect(470, 80, 62, 22));
        _erpSpin_2->setMaximum(1);
        _erpSpin_2->setSingleStep(0.1);
        _erpSpin_2->setValue(0.3);
        _objectListA = new QListView(Contact);
        _objectListA->setObjectName(QString::fromUtf8("_objectListA"));
        _objectListA->setGeometry(QRect(10, 40, 101, 281));
        label_8 = new QLabel(Contact);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(360, 50, 71, 21));
        _cfmSpin_2 = new QDoubleSpinBox(Contact);
        _cfmSpin_2->setObjectName(QString::fromUtf8("_cfmSpin_2"));
        _cfmSpin_2->setGeometry(QRect(470, 50, 62, 22));
        _cfmSpin_2->setDecimals(2);
        _cfmSpin_2->setMaximum(1);
        _cfmSpin_2->setSingleStep(0.1);
        _cfmSpin_2->setValue(0.6);
        label_9 = new QLabel(Contact);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(360, 70, 71, 21));
        tabWidget->addTab(Contact, QString());
        ExcludeList = new QWidget();
        ExcludeList->setObjectName(QString::fromUtf8("ExcludeList"));
        tabWidget->addTab(ExcludeList, QString());

        verticalLayout->addWidget(tabWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        _applyBtn = new QPushButton(ODESimCfgForm);
        _applyBtn->setObjectName(QString::fromUtf8("_applyBtn"));

        horizontalLayout->addWidget(_applyBtn);

        _cancelBtn = new QPushButton(ODESimCfgForm);
        _cancelBtn->setObjectName(QString::fromUtf8("_cancelBtn"));

        horizontalLayout->addWidget(_cancelBtn);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(ODESimCfgForm);

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(ODESimCfgForm);
    } // setupUi

    void retranslateUi(QWidget *ODESimCfgForm)
    {
        ODESimCfgForm->setWindowTitle(QApplication::translate("ODESimCfgForm", "Form", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("ODESimCfgForm", "Step method: ", 0, QApplication::UnicodeUTF8));
        _stepMethodBox->clear();
        _stepMethodBox->insertItems(0, QStringList()
         << QApplication::translate("ODESimCfgForm", "WorldStep", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ODESimCfgForm", "WorldQuickStep", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ODESimCfgForm", "WorldFast1", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_WHATSTHIS
        _stepMethodBox->setWhatsThis(QApplication::translate("ODESimCfgForm", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        _spaceMethodBox->clear();
        _spaceMethodBox->insertItems(0, QStringList()
         << QApplication::translate("ODESimCfgForm", "QuadTree", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ODESimCfgForm", "Simple", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ODESimCfgForm", "Hash", 0, QApplication::UnicodeUTF8)
        );
#ifndef QT_NO_WHATSTHIS
        _spaceMethodBox->setWhatsThis(QApplication::translate("ODESimCfgForm", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">There are several kinds of spaces. Each kind uses different internaldata structures to store the geoms, and different algorithms to performthe collision culling: </p>\n"
"<ul style=\"-qt-list-indent: 1;\"><li style=\" font-size:8pt;\" style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"> Simple space. This does not do any collision culling - itsimply checks every possible pair of geoms for intersection, andreports the pairs whose AABBs overlap. The time required to dointersection testing "
                        "for <span style=\" font-style:italic;\">n</span> objects is <span style=\" font-style:italic;\">O</span>(<span style=\" font-style:italic;\">n</span><span style=\" vertical-align:super;\">2</span>).This should not be used for large numbers of objects, but it can be thepreferred algorithm for a small number of objects. This is also usefulfor debugging potential problems with the collision system.</li></ul>\n"
"<ul style=\"-qt-list-indent: 1;\"><li style=\" font-size:8pt;\" style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"> Multi-resolution hash table space. This uses an internal datastructure that records how each geom overlaps cells in one of severalthree dimensional grids. Each grid has cubical cells of side lengths 2<span style=\" font-style:italic; vertical-align:super;\">i</span>, where <span style=\" font-style:italic;\">i</span> is an integer that ranges from a minimum to a maximum value. The time required to do intersection testing fo"
                        "r <span style=\" font-style:italic;\">n</span> objects is <span style=\" font-style:italic;\">O</span>(<span style=\" font-style:italic;\">n</span>)(as long as those objects are not clustered together too closely), aseach object can be quickly paired with the objects around it.</li></ul>\n"
"<ul style=\"-qt-list-indent: 1;\"><li style=\" font-size:8pt;\" style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"> Quadtree space. This uses a pre-allocated hierarchicalgrid-based AABB tree to quickly cull collision checks. It'sexceptionally quick for large amounts of objects in landscape-shapedworlds. The amount of memory used is 4^depth * 32 bytes. CurrentlydSpaceGetGeom is not implemented for the quadtree space. </li></ul></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_WHATSTHIS
        _clusterAlgBox->clear();
        _clusterAlgBox->insertItems(0, QStringList()
         << QApplication::translate("ODESimCfgForm", "Box", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("ODESimCfgForm", "None", 0, QApplication::UnicodeUTF8)
        );
        label_3->setText(QApplication::translate("ODESimCfgForm", "Global CFM", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("ODESimCfgForm", "Max iterations: ", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("ODESimCfgForm", "Collision margin: ", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("ODESimCfgForm", "Broad phase collision detection method: ", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("ODESimCfgForm", "Global ERP", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("ODESimCfgForm", "Clustering algorithm: ", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(General), QApplication::translate("ODESimCfgForm", "General", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("ODESimCfgForm", "GroupBox", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("ODESimCfgForm", "Angular velocity:", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("ODESimCfgForm", "Linear velocity:", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("ODESimCfgForm", "Torque: ", 0, QApplication::UnicodeUTF8));
        _selectedName->setText(QString());
        _enabledBox->setText(QApplication::translate("ODESimCfgForm", "Enabled", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("ODESimCfgForm", "Inertia:", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("ODESimCfgForm", "Contact reduction threshold:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("ODESimCfgForm", "Name:", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("ODESimCfgForm", "Orientation:", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("ODESimCfgForm", "Material:", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("ODESimCfgForm", "Object type:", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("ODESimCfgForm", "Mass:", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("ODESimCfgForm", "Position:", 0, QApplication::UnicodeUTF8));
        _objectTypeName->setText(QString());
        label_19->setText(QApplication::translate("ODESimCfgForm", "Force: ", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Objects), QApplication::translate("ODESimCfgForm", "Objects", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("ODESimCfgForm", "Device type:", 0, QApplication::UnicodeUTF8));
        _objectTypeName_2->setText(QString());
        label_23->setText(QApplication::translate("ODESimCfgForm", "Name:", 0, QApplication::UnicodeUTF8));
        _selectedName_2->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(Devices), QApplication::translate("ODESimCfgForm", "Devices", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("ODESimCfgForm", "Object A", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("ODESimCfgForm", "Object B", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("ODESimCfgForm", "Global CFM", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("ODESimCfgForm", "Global ERP", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Contact), QApplication::translate("ODESimCfgForm", "Contact", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(ExcludeList), QApplication::translate("ODESimCfgForm", "Exclude list", 0, QApplication::UnicodeUTF8));
        _applyBtn->setText(QApplication::translate("ODESimCfgForm", "Apply", 0, QApplication::UnicodeUTF8));
        _cancelBtn->setText(QApplication::translate("ODESimCfgForm", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ODESimCfgForm: public Ui_ODESimCfgForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ODESIMCFGFORM_H
