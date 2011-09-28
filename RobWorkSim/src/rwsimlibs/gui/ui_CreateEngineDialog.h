/********************************************************************************
** Form generated from reading UI file 'CreateEngineDialog.ui'
**
** Created: Mon Aug 1 12:19:16 2011
**      by: Qt User Interface Compiler version 4.7.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CREATEENGINEDIALOG_H
#define UI_CREATEENGINEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_CreateEngineDialog
{
public:
    QComboBox *_spaceMethodBox;
    QLabel *label;
    QPushButton *_cancelBtn;
    QPushButton *_createBtn;

    void setupUi(QDialog *CreateEngineDialog)
    {
        if (CreateEngineDialog->objectName().isEmpty())
            CreateEngineDialog->setObjectName(QString::fromUtf8("CreateEngineDialog"));
        CreateEngineDialog->setWindowModality(Qt::ApplicationModal);
        CreateEngineDialog->resize(264, 115);
        _spaceMethodBox = new QComboBox(CreateEngineDialog);
        _spaceMethodBox->setObjectName(QString::fromUtf8("_spaceMethodBox"));
        _spaceMethodBox->setGeometry(QRect(100, 40, 141, 21));
        label = new QLabel(CreateEngineDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(10, 40, 81, 21));
        _cancelBtn = new QPushButton(CreateEngineDialog);
        _cancelBtn->setObjectName(QString::fromUtf8("_cancelBtn"));
        _cancelBtn->setGeometry(QRect(180, 90, 75, 24));
        _createBtn = new QPushButton(CreateEngineDialog);
        _createBtn->setObjectName(QString::fromUtf8("_createBtn"));
        _createBtn->setGeometry(QRect(100, 90, 75, 24));

        retranslateUi(CreateEngineDialog);

        QMetaObject::connectSlotsByName(CreateEngineDialog);
    } // setupUi

    void retranslateUi(QDialog *CreateEngineDialog)
    {
        CreateEngineDialog->setWindowTitle(QApplication::translate("CreateEngineDialog", "Create physics engine", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_WHATSTHIS
        _spaceMethodBox->setWhatsThis(QApplication::translate("CreateEngineDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
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
        label->setText(QApplication::translate("CreateEngineDialog", "Physics engine:", 0, QApplication::UnicodeUTF8));
        _cancelBtn->setText(QApplication::translate("CreateEngineDialog", "Cancel", 0, QApplication::UnicodeUTF8));
        _createBtn->setText(QApplication::translate("CreateEngineDialog", "Create", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CreateEngineDialog: public Ui_CreateEngineDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CREATEENGINEDIALOG_H
