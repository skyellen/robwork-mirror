/********************************************************************************
** Form generated from reading UI file 'SupportPoseAnalyserDialog.ui'
**
** Created: Mon 28. Jun 12:34:02 2010
**      by: Qt User Interface Compiler version 4.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SUPPORTPOSEANALYSERDIALOG_H
#define UI_SUPPORTPOSEANALYSERDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SupportPoseAnalyserDialog
{
public:
    QVBoxLayout *verticalLayout_6;
    QTabWidget *tabWidget_2;
    QWidget *tab_3;
    QPushButton *pushButton;
    QDoubleSpinBox *doubleSpinBox;
    QLabel *label_2;
    QComboBox *_planarObjectBox;
    QWidget *tab_4;
    QHBoxLayout *horizontalLayout_6;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *groupBox;
    QVBoxLayout *vboxLayout;
    QPushButton *_loadStartPosesBtn;
    QPushButton *_loadFromFileBtn;
    QPushButton *_listenForDataBtn;
    QLabel *_dataLoadedLbl;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QSpinBox *_minPointsCircleSpin;
    QLabel *label;
    QDoubleSpinBox *_thresSpin;
    QLabel *label_3;
    QDoubleSpinBox *_epsilonSpin;
    QLabel *label_5;
    QSpinBox *_thresholdSpin;
    QLabel *label_6;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *_resetBtn;
    QPushButton *_processBtn;
    QSpacerItem *verticalSpacer;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_5;
    QGroupBox *groupBox_4;
    QHBoxLayout *horizontalLayout_7;
    QVBoxLayout *verticalLayout_5;
    QComboBox *_selectObjBox;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *_drawXBox;
    QCheckBox *_drawYBox;
    QCheckBox *_drawZBox;
    QGridLayout *gridLayout;
    QCheckBox *_drawPointsBox;
    QCheckBox *_drawCirclesBox;
    QCheckBox *_drawStartPosesBox;
    QSpacerItem *horizontalSpacer;
    QFrame *_glframe;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_8;
    QListWidget *_resultView;
    QGroupBox *groupBox_5;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *_saveBtn;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *_exitBtn;

    void setupUi(QDialog *SupportPoseAnalyserDialog)
    {
        if (SupportPoseAnalyserDialog->objectName().isEmpty())
            SupportPoseAnalyserDialog->setObjectName(QString::fromUtf8("SupportPoseAnalyserDialog"));
        SupportPoseAnalyserDialog->setWindowModality(Qt::NonModal);
        SupportPoseAnalyserDialog->setEnabled(true);
        SupportPoseAnalyserDialog->resize(659, 438);
        SupportPoseAnalyserDialog->setMaximumSize(QSize(16777215, 16777215));
        SupportPoseAnalyserDialog->setSizeGripEnabled(true);
        SupportPoseAnalyserDialog->setModal(false);
        verticalLayout_6 = new QVBoxLayout(SupportPoseAnalyserDialog);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        tabWidget_2 = new QTabWidget(SupportPoseAnalyserDialog);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        tab_3 = new QWidget();
        tab_3->setObjectName(QString::fromUtf8("tab_3"));
        pushButton = new QPushButton(tab_3);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(30, 90, 75, 23));
        doubleSpinBox = new QDoubleSpinBox(tab_3);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setGeometry(QRect(30, 140, 62, 22));
        label_2 = new QLabel(tab_3);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(100, 140, 111, 20));
        _planarObjectBox = new QComboBox(tab_3);
        _planarObjectBox->setObjectName(QString::fromUtf8("_planarObjectBox"));
        _planarObjectBox->setGeometry(QRect(30, 50, 151, 22));
        tabWidget_2->addTab(tab_3, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName(QString::fromUtf8("tab_4"));
        horizontalLayout_6 = new QHBoxLayout(tab_4);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        groupBox_3 = new QGroupBox(tab_4);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        verticalLayout_7 = new QVBoxLayout(groupBox_3);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        groupBox = new QGroupBox(groupBox_3);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        vboxLayout = new QVBoxLayout(groupBox);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        _loadStartPosesBtn = new QPushButton(groupBox);
        _loadStartPosesBtn->setObjectName(QString::fromUtf8("_loadStartPosesBtn"));

        vboxLayout->addWidget(_loadStartPosesBtn);

        _loadFromFileBtn = new QPushButton(groupBox);
        _loadFromFileBtn->setObjectName(QString::fromUtf8("_loadFromFileBtn"));

        vboxLayout->addWidget(_loadFromFileBtn);

        _listenForDataBtn = new QPushButton(groupBox);
        _listenForDataBtn->setObjectName(QString::fromUtf8("_listenForDataBtn"));

        vboxLayout->addWidget(_listenForDataBtn);

        _dataLoadedLbl = new QLabel(groupBox);
        _dataLoadedLbl->setObjectName(QString::fromUtf8("_dataLoadedLbl"));

        vboxLayout->addWidget(_dataLoadedLbl);


        verticalLayout_7->addWidget(groupBox);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        _minPointsCircleSpin = new QSpinBox(groupBox_3);
        _minPointsCircleSpin->setObjectName(QString::fromUtf8("_minPointsCircleSpin"));
        _minPointsCircleSpin->setMinimum(1);
        _minPointsCircleSpin->setMaximum(10000);

        formLayout->setWidget(0, QFormLayout::LabelRole, _minPointsCircleSpin);

        label = new QLabel(groupBox_3);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::FieldRole, label);

        _thresSpin = new QDoubleSpinBox(groupBox_3);
        _thresSpin->setObjectName(QString::fromUtf8("_thresSpin"));
        _thresSpin->setDecimals(6);
        _thresSpin->setMaximum(1);
        _thresSpin->setSingleStep(0.0001);
        _thresSpin->setValue(0.0001);

        formLayout->setWidget(1, QFormLayout::LabelRole, _thresSpin);

        label_3 = new QLabel(groupBox_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::FieldRole, label_3);

        _epsilonSpin = new QDoubleSpinBox(groupBox_3);
        _epsilonSpin->setObjectName(QString::fromUtf8("_epsilonSpin"));
        _epsilonSpin->setDecimals(5);
        _epsilonSpin->setMaximum(1);
        _epsilonSpin->setSingleStep(0.001);
        _epsilonSpin->setValue(0.02);

        formLayout->setWidget(2, QFormLayout::LabelRole, _epsilonSpin);

        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(2, QFormLayout::FieldRole, label_5);

        _thresholdSpin = new QSpinBox(groupBox_3);
        _thresholdSpin->setObjectName(QString::fromUtf8("_thresholdSpin"));
        _thresholdSpin->setMinimum(1);
        _thresholdSpin->setMaximum(254);
        _thresholdSpin->setValue(50);

        formLayout->setWidget(3, QFormLayout::LabelRole, _thresholdSpin);

        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(3, QFormLayout::FieldRole, label_6);


        verticalLayout->addLayout(formLayout);

        groupBox_2 = new QGroupBox(groupBox_3);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        horizontalLayout = new QHBoxLayout(groupBox_2);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        _resetBtn = new QPushButton(groupBox_2);
        _resetBtn->setObjectName(QString::fromUtf8("_resetBtn"));

        horizontalLayout->addWidget(_resetBtn);

        _processBtn = new QPushButton(groupBox_2);
        _processBtn->setObjectName(QString::fromUtf8("_processBtn"));

        horizontalLayout->addWidget(_processBtn);

        _processBtn->raise();
        _resetBtn->raise();

        verticalLayout->addWidget(groupBox_2);


        verticalLayout_7->addLayout(verticalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_7->addItem(verticalSpacer);


        horizontalLayout_6->addWidget(groupBox_3);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setSizeConstraint(QLayout::SetDefaultConstraint);
        groupBox_4 = new QGroupBox(tab_4);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setMaximumSize(QSize(500, 300));
        horizontalLayout_7 = new QHBoxLayout(groupBox_4);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        _selectObjBox = new QComboBox(groupBox_4);
        _selectObjBox->setObjectName(QString::fromUtf8("_selectObjBox"));

        verticalLayout_5->addWidget(_selectObjBox);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        _drawXBox = new QCheckBox(groupBox_4);
        _drawXBox->setObjectName(QString::fromUtf8("_drawXBox"));
        _drawXBox->setChecked(true);

        horizontalLayout_4->addWidget(_drawXBox);

        _drawYBox = new QCheckBox(groupBox_4);
        _drawYBox->setObjectName(QString::fromUtf8("_drawYBox"));
        _drawYBox->setChecked(true);

        horizontalLayout_4->addWidget(_drawYBox);

        _drawZBox = new QCheckBox(groupBox_4);
        _drawZBox->setObjectName(QString::fromUtf8("_drawZBox"));
        _drawZBox->setChecked(true);

        horizontalLayout_4->addWidget(_drawZBox);


        verticalLayout_5->addLayout(horizontalLayout_4);


        horizontalLayout_7->addLayout(verticalLayout_5);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _drawPointsBox = new QCheckBox(groupBox_4);
        _drawPointsBox->setObjectName(QString::fromUtf8("_drawPointsBox"));
        _drawPointsBox->setChecked(true);

        gridLayout->addWidget(_drawPointsBox, 0, 0, 1, 1);

        _drawCirclesBox = new QCheckBox(groupBox_4);
        _drawCirclesBox->setObjectName(QString::fromUtf8("_drawCirclesBox"));

        gridLayout->addWidget(_drawCirclesBox, 1, 0, 1, 1);

        _drawStartPosesBox = new QCheckBox(groupBox_4);
        _drawStartPosesBox->setObjectName(QString::fromUtf8("_drawStartPosesBox"));

        gridLayout->addWidget(_drawStartPosesBox, 0, 1, 1, 1);


        horizontalLayout_7->addLayout(gridLayout);


        horizontalLayout_5->addWidget(groupBox_4);

        horizontalSpacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_5);

        _glframe = new QFrame(tab_4);
        _glframe->setObjectName(QString::fromUtf8("_glframe"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(_glframe->sizePolicy().hasHeightForWidth());
        _glframe->setSizePolicy(sizePolicy);
        _glframe->setMinimumSize(QSize(200, 100));
        _glframe->setFrameShape(QFrame::StyledPanel);
        _glframe->setFrameShadow(QFrame::Raised);

        verticalLayout_3->addWidget(_glframe);


        horizontalLayout_6->addLayout(verticalLayout_3);

        tabWidget_2->addTab(tab_4, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName(QString::fromUtf8("tab_5"));
        verticalLayout_2 = new QVBoxLayout(tab_5);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        _resultView = new QListWidget(tab_5);
        _resultView->setObjectName(QString::fromUtf8("_resultView"));

        horizontalLayout_8->addWidget(_resultView);

        groupBox_5 = new QGroupBox(tab_5);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setMinimumSize(QSize(400, 150));
        label_7 = new QLabel(groupBox_5);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 60, 71, 16));
        label_8 = new QLabel(groupBox_5);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 40, 71, 16));
        label_9 = new QLabel(groupBox_5);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(10, 100, 91, 16));
        label_10 = new QLabel(groupBox_5);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(10, 20, 71, 16));
        label_11 = new QLabel(groupBox_5);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(10, 80, 71, 16));

        horizontalLayout_8->addWidget(groupBox_5);


        verticalLayout_2->addLayout(horizontalLayout_8);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);

        _saveBtn = new QPushButton(tab_5);
        _saveBtn->setObjectName(QString::fromUtf8("_saveBtn"));

        horizontalLayout_3->addWidget(_saveBtn);


        verticalLayout_2->addLayout(horizontalLayout_3);

        tabWidget_2->addTab(tab_5, QString());

        verticalLayout_6->addWidget(tabWidget_2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);

        _exitBtn = new QPushButton(SupportPoseAnalyserDialog);
        _exitBtn->setObjectName(QString::fromUtf8("_exitBtn"));

        horizontalLayout_2->addWidget(_exitBtn);


        verticalLayout_6->addLayout(horizontalLayout_2);

        QWidget::setTabOrder(_listenForDataBtn, _minPointsCircleSpin);
        QWidget::setTabOrder(_minPointsCircleSpin, _thresSpin);
        QWidget::setTabOrder(_thresSpin, _resetBtn);
        QWidget::setTabOrder(_resetBtn, _processBtn);
        QWidget::setTabOrder(_processBtn, _drawXBox);
        QWidget::setTabOrder(_drawXBox, _drawYBox);
        QWidget::setTabOrder(_drawYBox, _drawZBox);
        QWidget::setTabOrder(_drawZBox, _drawPointsBox);
        QWidget::setTabOrder(_drawPointsBox, _drawCirclesBox);
        QWidget::setTabOrder(_drawCirclesBox, _drawStartPosesBox);

        retranslateUi(SupportPoseAnalyserDialog);
        QObject::connect(_exitBtn, SIGNAL(clicked()), SupportPoseAnalyserDialog, SLOT(accept()));

        tabWidget_2->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(SupportPoseAnalyserDialog);
    } // setupUi

    void retranslateUi(QDialog *SupportPoseAnalyserDialog)
    {
        SupportPoseAnalyserDialog->setWindowTitle(QApplication::translate("SupportPoseAnalyserDialog", "SupportPoseAnalyser", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("SupportPoseAnalyserDialog", "Analyse", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SupportPoseAnalyserDialog", "Stable Threshold in m", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_3), QApplication::translate("SupportPoseAnalyserDialog", "Planar", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Control", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Data options", 0, QApplication::UnicodeUTF8));
        _loadStartPosesBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Load start poses", 0, QApplication::UnicodeUTF8));
        _loadFromFileBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Load end poses", 0, QApplication::UnicodeUTF8));
        _listenForDataBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Run from simulation", 0, QApplication::UnicodeUTF8));
        _dataLoadedLbl->setText(QApplication::translate("SupportPoseAnalyserDialog", "No data loaded!", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SupportPoseAnalyserDialog", "d - Points per circle", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SupportPoseAnalyserDialog", "t - dist threshold", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _epsilonSpin->setToolTip(QApplication::translate("SupportPoseAnalyserDialog", "The points within an euclidean distance \n"
"\"epsilon\" of a circle are considered to \n"
"belong to that circle.\n"
"If too many support poses is found then \n"
"decrease this. If too few is found increase it.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_5->setText(QApplication::translate("SupportPoseAnalyserDialog", "Epsilon", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        _thresholdSpin->setToolTip(QApplication::translate("SupportPoseAnalyserDialog", "The threshold used for the hough \n"
"line extraction. If too many support\n"
"poses is found then increase this. If\n"
"too few is found decrease it.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        label_6->setText(QApplication::translate("SupportPoseAnalyserDialog", "Hough Threshold", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Processing", 0, QApplication::UnicodeUTF8));
        _resetBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Reset", 0, QApplication::UnicodeUTF8));
        _processBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Process", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Visual options", 0, QApplication::UnicodeUTF8));
        _drawXBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw X", 0, QApplication::UnicodeUTF8));
        _drawYBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw Y", 0, QApplication::UnicodeUTF8));
        _drawZBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw Z", 0, QApplication::UnicodeUTF8));
        _drawPointsBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw points", 0, QApplication::UnicodeUTF8));
        _drawCirclesBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw circles", 0, QApplication::UnicodeUTF8));
        _drawStartPosesBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw start poses", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_4), QApplication::translate("SupportPoseAnalyserDialog", "General", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Support pose info", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SupportPoseAnalyserDialog", "Quality: ", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SupportPoseAnalyserDialog", "Probability: ", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("SupportPoseAnalyserDialog", "Rotation vectors:", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("SupportPoseAnalyserDialog", "Degree: ", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("SupportPoseAnalyserDialog", "Segments: ", 0, QApplication::UnicodeUTF8));
        _saveBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Save poses", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QApplication::translate("SupportPoseAnalyserDialog", "Result", 0, QApplication::UnicodeUTF8));
        _exitBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Exit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SupportPoseAnalyserDialog: public Ui_SupportPoseAnalyserDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SUPPORTPOSEANALYSERDIALOG_H
