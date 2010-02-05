/********************************************************************************
** Form generated from reading UI file 'SupportPoseAnalyserDialog.ui'
**
** Created: Thu 28. Jan 14:04:54 2010
**      by: Qt User Interface Compiler version 4.6.0
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
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_4;
    QTabWidget *tabWidget;
    QWidget *tab;
    QFormLayout *formLayout_2;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *vboxLayout;
    QPushButton *_loadStartPosesBtn;
    QPushButton *_loadFromFileBtn;
    QPushButton *_listenForDataBtn;
    QLabel *_dataLoadedLbl;
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
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_5;
    QListWidget *_resultView;
    QGroupBox *groupBox_5;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *_saveBtn;
    QPushButton *_exitBtn;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_5;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_2;
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
        horizontalLayout_3 = new QHBoxLayout(SupportPoseAnalyserDialog);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        tabWidget = new QTabWidget(SupportPoseAnalyserDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setMaximumSize(QSize(212, 16777215));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        formLayout_2 = new QFormLayout(tab);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        groupBox = new QGroupBox(tab);
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


        verticalLayout->addWidget(groupBox);

        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        _minPointsCircleSpin = new QSpinBox(tab);
        _minPointsCircleSpin->setObjectName(QString::fromUtf8("_minPointsCircleSpin"));
        _minPointsCircleSpin->setMinimum(1);
        _minPointsCircleSpin->setMaximum(10000);

        formLayout->setWidget(0, QFormLayout::LabelRole, _minPointsCircleSpin);

        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::FieldRole, label);

        _thresSpin = new QDoubleSpinBox(tab);
        _thresSpin->setObjectName(QString::fromUtf8("_thresSpin"));
        _thresSpin->setDecimals(6);
        _thresSpin->setMaximum(1);
        _thresSpin->setSingleStep(0.0001);
        _thresSpin->setValue(0.0001);

        formLayout->setWidget(1, QFormLayout::LabelRole, _thresSpin);

        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::FieldRole, label_3);

        _epsilonSpin = new QDoubleSpinBox(tab);
        _epsilonSpin->setObjectName(QString::fromUtf8("_epsilonSpin"));
        _epsilonSpin->setDecimals(5);
        _epsilonSpin->setMaximum(1);
        _epsilonSpin->setSingleStep(0.001);
        _epsilonSpin->setValue(0.02);

        formLayout->setWidget(2, QFormLayout::LabelRole, _epsilonSpin);

        label_5 = new QLabel(tab);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout->setWidget(2, QFormLayout::FieldRole, label_5);

        _thresholdSpin = new QSpinBox(tab);
        _thresholdSpin->setObjectName(QString::fromUtf8("_thresholdSpin"));
        _thresholdSpin->setMinimum(1);
        _thresholdSpin->setMaximum(254);
        _thresholdSpin->setValue(50);

        formLayout->setWidget(3, QFormLayout::LabelRole, _thresholdSpin);

        label_6 = new QLabel(tab);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(3, QFormLayout::FieldRole, label_6);


        verticalLayout->addLayout(formLayout);

        groupBox_2 = new QGroupBox(tab);
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


        formLayout_2->setLayout(0, QFormLayout::LabelRole, verticalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        formLayout_2->setItem(2, QFormLayout::LabelRole, verticalSpacer);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        verticalLayout_5 = new QVBoxLayout(tab_2);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        _resultView = new QListWidget(tab_2);
        _resultView->setObjectName(QString::fromUtf8("_resultView"));

        verticalLayout_5->addWidget(_resultView);

        groupBox_5 = new QGroupBox(tab_2);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setMinimumSize(QSize(0, 150));
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

        verticalLayout_5->addWidget(groupBox_5);

        tabWidget->addTab(tab_2, QString());

        verticalLayout_4->addWidget(tabWidget);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        _saveBtn = new QPushButton(SupportPoseAnalyserDialog);
        _saveBtn->setObjectName(QString::fromUtf8("_saveBtn"));

        horizontalLayout_2->addWidget(_saveBtn);

        _exitBtn = new QPushButton(SupportPoseAnalyserDialog);
        _exitBtn->setObjectName(QString::fromUtf8("_exitBtn"));

        horizontalLayout_2->addWidget(_exitBtn);


        verticalLayout_4->addLayout(horizontalLayout_2);


        horizontalLayout_3->addLayout(verticalLayout_4);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout_5->setSizeConstraint(QLayout::SetDefaultConstraint);
        groupBox_4 = new QGroupBox(SupportPoseAnalyserDialog);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setMaximumSize(QSize(271, 100));
        verticalLayout_2 = new QVBoxLayout(groupBox_4);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        _selectObjBox = new QComboBox(groupBox_4);
        _selectObjBox->setObjectName(QString::fromUtf8("_selectObjBox"));

        verticalLayout_2->addWidget(_selectObjBox);

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


        verticalLayout_2->addLayout(horizontalLayout_4);


        horizontalLayout_5->addWidget(groupBox_4);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        _drawPointsBox = new QCheckBox(SupportPoseAnalyserDialog);
        _drawPointsBox->setObjectName(QString::fromUtf8("_drawPointsBox"));
        _drawPointsBox->setChecked(true);

        gridLayout->addWidget(_drawPointsBox, 0, 0, 1, 1);

        _drawCirclesBox = new QCheckBox(SupportPoseAnalyserDialog);
        _drawCirclesBox->setObjectName(QString::fromUtf8("_drawCirclesBox"));

        gridLayout->addWidget(_drawCirclesBox, 1, 0, 1, 1);

        _drawStartPosesBox = new QCheckBox(SupportPoseAnalyserDialog);
        _drawStartPosesBox->setObjectName(QString::fromUtf8("_drawStartPosesBox"));

        gridLayout->addWidget(_drawStartPosesBox, 0, 1, 1, 1);


        horizontalLayout_5->addLayout(gridLayout);

        horizontalSpacer = new QSpacerItem(0, 0, QSizePolicy::Preferred, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);


        verticalLayout_3->addLayout(horizontalLayout_5);

        _glframe = new QFrame(SupportPoseAnalyserDialog);
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


        horizontalLayout_3->addLayout(verticalLayout_3);

        QWidget::setTabOrder(_listenForDataBtn, _minPointsCircleSpin);
        QWidget::setTabOrder(_minPointsCircleSpin, _thresSpin);
        QWidget::setTabOrder(_thresSpin, _resetBtn);
        QWidget::setTabOrder(_resetBtn, _processBtn);
        QWidget::setTabOrder(_processBtn, _selectObjBox);
        QWidget::setTabOrder(_selectObjBox, _drawXBox);
        QWidget::setTabOrder(_drawXBox, _drawYBox);
        QWidget::setTabOrder(_drawYBox, _drawZBox);
        QWidget::setTabOrder(_drawZBox, _drawPointsBox);
        QWidget::setTabOrder(_drawPointsBox, _drawCirclesBox);
        QWidget::setTabOrder(_drawCirclesBox, _drawStartPosesBox);

        retranslateUi(SupportPoseAnalyserDialog);
        QObject::connect(_exitBtn, SIGNAL(clicked()), SupportPoseAnalyserDialog, SLOT(accept()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(SupportPoseAnalyserDialog);
    } // setupUi

    void retranslateUi(QDialog *SupportPoseAnalyserDialog)
    {
        SupportPoseAnalyserDialog->setWindowTitle(QApplication::translate("SupportPoseAnalyserDialog", "SupportPoseAnalyser", 0, QApplication::UnicodeUTF8));
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
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("SupportPoseAnalyserDialog", "Control", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "Support pose info", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SupportPoseAnalyserDialog", "Quality: ", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SupportPoseAnalyserDialog", "Probability: ", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("SupportPoseAnalyserDialog", "Rotation vectors:", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("SupportPoseAnalyserDialog", "Degree: ", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("SupportPoseAnalyserDialog", "Segments: ", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("SupportPoseAnalyserDialog", "Result", 0, QApplication::UnicodeUTF8));
        _saveBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Save poses", 0, QApplication::UnicodeUTF8));
        _exitBtn->setText(QApplication::translate("SupportPoseAnalyserDialog", "Exit", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("SupportPoseAnalyserDialog", "GroupBox", 0, QApplication::UnicodeUTF8));
        _drawXBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw X", 0, QApplication::UnicodeUTF8));
        _drawYBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw Y", 0, QApplication::UnicodeUTF8));
        _drawZBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw Z", 0, QApplication::UnicodeUTF8));
        _drawPointsBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw points", 0, QApplication::UnicodeUTF8));
        _drawCirclesBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw circles", 0, QApplication::UnicodeUTF8));
        _drawStartPosesBox->setText(QApplication::translate("SupportPoseAnalyserDialog", "Draw start poses", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SupportPoseAnalyserDialog: public Ui_SupportPoseAnalyserDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SUPPORTPOSEANALYSERDIALOG_H
