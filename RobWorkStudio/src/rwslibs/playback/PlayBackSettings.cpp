/*
 * PlayBackSettings.cpp
 *
 *  Created on: Oct 28, 2008
 *      Author: lpe
 */

#include "PlayBackSettings.hpp"

#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>

namespace {
    const double DEFAULT_UPDATERATE = 0.040;
    const QString DEFAULT_RECORDFILENAME = "Image";
    const QString DEFAULT_RECORDTYPE = "png";
}

PlayBackSettings::PlayBackSettings():
    QDialog(NULL),_theScale(1.0)
{
    setWindowModality(Qt::WindowModal);

    QVBoxLayout* pLayout = new QVBoxLayout(this);
    this->setLayout(pLayout);

    {
        QGroupBox* grpGeneral = new QGroupBox(tr("&General"));
        pLayout->addWidget(grpGeneral);
        QHBoxLayout *hbox = new QHBoxLayout();

        hbox->addWidget(new QLabel("UpdateRate (seconds)"));

        _spnUpdateRate = new QDoubleSpinBox();
        _spnUpdateRate->setRange(0.001, 10);
        _spnUpdateRate->setValue(0.04);
        _spnUpdateRate->setSingleStep(0.01);
        hbox->addWidget(_spnUpdateRate);
        grpGeneral->setLayout(hbox);
    }

    {
        QGroupBox* grpRecording = new QGroupBox(tr("&Recording"));
        pLayout->addWidget(grpRecording);
        QGridLayout* layout = new QGridLayout();
        int n = 0;
        layout->addWidget(new QLabel("Filename"), n, 0);
        _edtFilename = new QLineEdit("Image");
        layout->addWidget(_edtFilename, n, 1);

        QPushButton* btnBrowse = new QPushButton(QIcon(":/fileopen"), "Browse");
        connect(btnBrowse, SIGNAL(clicked()), this, SLOT(browse()));
        layout->addWidget(btnBrowse, n++, 2);

        layout->addWidget(new QLabel("Type"), n, 0);
        _cmbType = new QComboBox();
        _cmbType->addItem("png");
        _cmbType->addItem("jpg");
        _cmbType->addItem("bmp");
        layout->addWidget(_cmbType, n++, 1);

        grpRecording->setLayout(layout);

    }

    {
        QGroupBox* grpScale = new QGroupBox(tr("&Scale"));
        pLayout->addWidget(grpScale);
        QHBoxLayout *hbox = new QHBoxLayout();

        hbox->addWidget(new QLabel("To:"));

        _spnScale = new QDoubleSpinBox();
        _spnScale->setRange(0.001, 1000000);
        _spnScale->setValue(1);
        _spnScale->setSingleStep(1);
        hbox->addWidget(_spnScale);
        grpScale->setLayout(hbox);
        QPushButton* btnScale = new QPushButton(tr("Scale"));
        hbox->addWidget(btnScale);
        connect(btnScale, SIGNAL(clicked()), this, SLOT(scale()));


    }

    {
        QWidget* base = new QWidget();
        QHBoxLayout* layout = new QHBoxLayout(base);
        base->setLayout(layout);

        QPushButton* btnOk = new QPushButton(tr("&Ok"));
        btnOk->isDefault();
        layout->addWidget(btnOk);
        connect(btnOk, SIGNAL(clicked()), this, SLOT(ok()));

        QPushButton* btnCancel = new QPushButton(tr("&Cancel"));
        layout->addWidget(btnCancel);
        connect(btnCancel, SIGNAL(clicked()), this, SLOT(cancel()));

        pLayout->addWidget(base);

    }


    _updateRate = DEFAULT_UPDATERATE;
    _recordFilename = DEFAULT_RECORDFILENAME;
    _recordType = DEFAULT_RECORDTYPE;



}

void PlayBackSettings::browse() {
    QString result = QFileDialog::getSaveFileName(this, "Save Images As");
    if (result.isEmpty())
        return;
    _edtFilename->setText(result);
}

void PlayBackSettings::scale() {
    _theScale = _spnScale->value();
    done(0);
}


void PlayBackSettings::ok() {
    _updateRate = _spnUpdateRate->value();

    _recordFilename = _edtFilename->text();

    _recordType = _cmbType->currentText();

    done(0);
}

void PlayBackSettings::cancel() {
    reject();
}

void PlayBackSettings::showEvent(QShowEvent* event) {
    _spnUpdateRate->setValue(_updateRate);

    _edtFilename->setText(_recordFilename);

    int index = _cmbType->findText(_recordType);
    _cmbType->setCurrentIndex(index);



}


PlayBackSettings::~PlayBackSettings()
{
    // TODO Auto-generated destructor stub
}


double PlayBackSettings::getUpdateRate() {
    return _updateRate;
}

QString PlayBackSettings::getRecordFilename() {
   return _recordFilename;
}

QString PlayBackSettings::getRecordFileType() {
    return _recordType;
}
