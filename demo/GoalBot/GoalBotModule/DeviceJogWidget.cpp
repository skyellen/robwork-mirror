#include "DeviceJogWidget.hpp"

#include <QGridLayout>
#include <QMessageBox>

using namespace rw::vworkcell;
using namespace rw::math;

DeviceJogWidget::DeviceJogWidget() {

}


DeviceJogWidget::~DeviceJogWidget() {

}

void DeviceJogWidget::setup(const std::pair<Device::Q, Device::Q>& bounds, const Device::Q& q) {
    _n = q.size();
    _ppJoints = new QDoubleSpinBox*[_n];
    _ppJointReal = new QLineEdit*[_n];

    QGridLayout* pLayout = new QGridLayout();

    for (size_t i = 0; i<q.size(); i++) {
	_ppJoints[i] = new QDoubleSpinBox();
	_ppJoints[i]->setRange(bounds.first(i), bounds.second(i));
	_ppJoints[i]->setSingleStep(0.05);
	_ppJoints[i]->setValue(q(i)); 
	_ppJoints[i]->adjustSize(); 
	pLayout->addWidget(_ppJoints[i],i,0);
	_ppJointReal[i] = new QLineEdit();
	_ppJointReal[i]->setReadOnly(true);
	_ppJointReal[i]->setText(QString::number(q(i)));
	pLayout->addWidget(_ppJointReal[i], i, 1, Qt::AlignTop);
	connect(_ppJoints[i], SIGNAL(valueChanged(double)), this, SLOT(valueChanged()));	
    }
    pLayout->setRowStretch(q.size()-1, 1);
    setLayout(pLayout);
}

void DeviceJogWidget::setup(const std::pair<Device::Q, Device::Q>& bounds, const Pose6D<>& p) {
    Device::Q q(6);
    for (size_t i = 0; i<6; i++)
	q(i) = p.get(i);
    setup(bounds, q);
}

void DeviceJogWidget::updatePosition(const Device::Q& qreal) {
    if (qreal.size() != _n) {
	QMessageBox::information(this, "Error", "Length of Real configuration does not match", QMessageBox::Ok);
	return;
    }
    for (size_t i = 0; i<_n; i++) {
	_ppJointReal[i]->setText(QString::number(qreal(i)));
    }    
}

void DeviceJogWidget::updatePosition(const Pose6D<>& pose) {
    for (size_t i = 0; i<_n; i++) {
	_ppJointReal[i]->setText(QString::number(pose.get(i)));
    }   
}

void DeviceJogWidget::valueChanged() {
    Device::Q qtarget(_n);
    for (size_t i = 0; i<_n; i++) {
	qtarget(i) = _ppJoints[i]->value();
    }
    std::cout<<"target = "<<qtarget<<std::endl;
    emit newTarget(qtarget);
}
