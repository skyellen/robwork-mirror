/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <iomanip>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QInputDialog>
#include <QLabel>
#include <QSlider>

#include "SliderTab.hpp"

#include <rw/math/RPY.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/Device.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;
using namespace rw::proximity;

namespace
{
    QLabel* makeNumericQLabel(double val)
    {
        std::stringstream s; s << std::setprecision(4) << val;
        return new QLabel(s.str().c_str());
    }

    QSlider* makeHSlider(int end)
    {
        QSlider* slider = new QSlider(Qt::Horizontal);

        slider->setRange(0, end);

        // The normalized step size for Page Up / Page Down keys.
        const double pageScale = 0.10;
        const int pageStep = (int)
            (pageScale *
             (slider->maximum() -
              slider->minimum()));
        slider->setPageStep(pageStep);

        return slider;
    }

    QDoubleSpinBox* makeDoubleSpinBox(double low, double high)
    {
        QDoubleSpinBox* box = new QDoubleSpinBox();
        box->setDecimals(3);
        box->setRange(low, high);

        const double step = (high - low) / 400;
        box->setSingleStep(step);

        return box;
    }

    const int sliderEnd = 4000;
}

//----------------------------------------------------------------------
// JointLine

/*
  A lot of things are missing here:

  Currently we have:

  - lower limit
  - upper limit

  (except this doesn't work for dependent joint limits).

  To make the interface nicer we also need access to:

  - name of joint
  - type of joint
  - joint index (maybe)

  In other words, we need access to the joint frame itself.
*/
Slider::Slider(const std::string& title,
               double low,
               double high,
               QGridLayout* layout,
               int row,
               QWidget* parent):
    QWidget(parent),
    _boxChanged(false),
    _sliderChanged(false),
    _toUnit(1.0)
{
    _title = new QLabel(tr(title.data()));
    _low = low;
    _high = high;

    _lowLabel = makeNumericQLabel(_low);
    _slider = makeHSlider(sliderEnd);
    _highLabel = makeNumericQLabel(_high);
    _box = makeDoubleSpinBox(_low, _high);

    unsigned int col = 0;
    if(!title.empty())
        layout->addWidget(_title, row, col++, Qt::AlignLeft);
    layout->addWidget(_lowLabel, row, col++, Qt::AlignRight);
    layout->addWidget(_lowLabel, row, col++); // own lowLabel
    layout->addWidget(_slider, row, col++); // own _slider
    layout->addWidget(_highLabel, row, col++); // own highLabel
    layout->addWidget(_box, row, col); // own _box
    /*
    layout->addWidget(_lowLabel, row, 0, Qt::AlignRight); // own lowLabel
    layout->addWidget(_lowLabel, row, 0); // own lowLabel
    layout->addWidget(_slider, row, 1); // own _slider
    layout->addWidget(_highLabel, row, 2); // own highLabel
    layout->addWidget(_box, row, 3); // own _box
    */

    std::stringstream sstr;
    sstr << "Limits: [" << _low  << ";" << _high << "]" << _desc;
    this->setToolTip(sstr.str().c_str());

    connect(_box,
            SIGNAL(valueChanged(double)),
            this,
            SLOT(boxValueChanged(double)));

    connect(_slider,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(sliderValueChanged(int)));

    setValue((_low + _high) / 2);
}

void Slider::unitUpdated() {
    const double lowUnit = _toUnit * _low;
    const double highUnit = _toUnit * _high;

    _lowLabel->setText(QString::number(lowUnit, 'g', 4));
    _highLabel->setText(QString::number(highUnit, 'g', 4));

    disconnect(_box, 0, 0, 0);
    disconnect(_slider, 0, 0, 0);

    const int val = _slider->value();
    _box->setRange(lowUnit, highUnit);
    const double step = (highUnit - lowUnit) / 400;
    _box->setSingleStep(step);
    setBoxValueFromSlider(val);

    connect(_box,
            SIGNAL(valueChanged(double)),
            this,
            SLOT(boxValueChanged(double)));

    connect(_slider,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(sliderValueChanged(int)));

    std::stringstream sstr;
    sstr << "Limits: [" << lowUnit  << ";" << highUnit << "]" << _desc;
    this->setToolTip(sstr.str().c_str());
}

void Slider::boxValueChanged(double val)
{
    _boxChanged = true;

    // Change the value of the slider.
    if (!_sliderChanged) {
        setSliderValueFromBox(val);
    }

    _sliderChanged = false;
    emit valueChanged();
}

void Slider::sliderValueChanged(int val)
{
    _sliderChanged = true;

    // Change the value of the box.
    double boxVal = _box->value();
    int boxVali = ((boxVal/_toUnit - _low) / (_high - _low) * sliderEnd );
    bool isBoxSame = boxVali==val;
    if (!isBoxSame) {
        setBoxValueFromSlider(val);
    }

    _boxChanged = false;

//    emit valueChanged();
}

void Slider::setSliderValueFromBox(double val)
{
//    _slider->setValue((int)((val - _low) / (_high - _low) * sliderEnd));
    _slider->setValue((int)((val/_toUnit - _low) / (_high - _low) * sliderEnd));
}

void Slider::setBoxValueFromSlider(int val)
{
//    _box->setValue(((double)val / sliderEnd) * (_high - _low) + _low);
    _box->setValue( ( ((double)val / sliderEnd) * (_high - _low) + _low ) * _toUnit);
}

double Slider::value() const
{
    return _box->value() / _toUnit;
}

void Slider::setValue(double val)
{
    _sliderChanged = true;
    _boxChanged = true;

    if (_low-0.00001 <= val && val <= _high+0.00001) {
        _box->setValue(val*_toUnit);


        _slider->setValue(
            (int)((val - _low) / (_high - _low) * sliderEnd));

    } else {
        RW_WARN(
            "Jog joint value "
            << val
            << " out of range ["
            << _low
            << ", "
            << _high
            << "].");
    }

    _sliderChanged = false;
    _boxChanged = false;
}


namespace {

    Q transform2q(const Transform3D<>& transform) {
        Q q(6);
        RPY<> rpy(transform.R());
        for (size_t i = 0; i<3; i++) {
            q(i) = transform.P()(i);
            q(i+3) = rpy(i);
        }
        return q;
    }

    Transform3D<> q2transform(const Q& q) {
        return Transform3D<>(Vector3D<>(q(0), q(1), q(2)), RPY<>(q(3), q(4), q(5)));
    }

}


MovableFrameTab::MovableFrameTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
                           MovableFrame* frame,
                           rw::models::WorkCell* workcell,
                           const rw::kinematics::State& state):
    _state(state),
    _frame(frame),
	_updating(false)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned
    QWidget* toppanel = new QWidget();
    QHBoxLayout* toplayout = new QHBoxLayout(toppanel);

    QWidget* sliderpanel = new QWidget();
    _layout = new QGridLayout(sliderpanel);



    _frames = Kinematics::findAllFrames(workcell->getWorldFrame(), _state);
    _cmbFrames = new QComboBox();
    int i = 0;
    for (std::vector<Frame*>::iterator it = _frames.begin(); it != _frames.end(); ++it, ++i) {
        _cmbFrames->addItem((*it)->getName().c_str(), QVariant(i));
    }
    connect(_cmbFrames, SIGNAL(currentIndexChanged(int)), this, SLOT(refFrameChanged(int)));
    _refframe = workcell->getWorldFrame(); //Setup the reference frame

    toplayout->addWidget(new QLabel("Ref. Frame: "));
    toplayout->addWidget(_cmbFrames);
    
    tablayout->addWidget(toppanel, 0, 0);
    _transformSliderWidget = new TransformSliderWidget(bounds, Kinematics::frameTframe(_refframe, _frame, _state));
    
    QPushButton* btnPasteQ = new QPushButton("Paste", _transformSliderWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(new QLabel(""));
    btnlayout->addWidget(btnPasteQ);
    tablayout->addLayout(btnlayout, 1, 0);
    connect(btnPasteQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(paste()));
    
    connect(_transformSliderWidget,
            SIGNAL(valueChanged(const rw::math::Transform3D<>&)),
            this,
            SLOT(transformChanged(const rw::math::Transform3D<>&)));

    tablayout->addWidget(_transformSliderWidget, 2, 0);




}

void MovableFrameTab::setUnits(const std::vector<double> &converters, const std::vector<std::string> &descriptions) {
    _transformSliderWidget->setUnits(converters, descriptions);
}

void MovableFrameTab::refFrameChanged(int index) {
    _refframe = _frames[index];
    doUpdateValues();
}



void MovableFrameTab::updateValues(const State& state) {
    _state = state;
    doUpdateValues();
}

void MovableFrameTab::doUpdateValues() {
	if (_updating)
		return;
    Transform3D<> transform = Kinematics::frameTframe(_refframe, _frame, _state);
    _transformSliderWidget->updateValues(transform);
}


void MovableFrameTab::transformChanged(const Transform3D<>& transform) {
    Transform3D<> parent2ref = Kinematics::frameTframe(_frame->getParent(_state), _refframe, _state);
    Transform3D<> result = parent2ref*transform;
    _frame->setTransform(result, _state);

	_updating = true;
    stateChanged(_state);
	_updating = false;
}



JointSliderWidget::JointSliderWidget() {
    _layout = new QGridLayout(this); // owned
}



void JointSliderWidget::setup(const std::vector<std::string>& titles,
                              const std::pair<Q,Q>& bounds,
                              const Q& q) {
  
  // Hack so that we can move the first slider with mouse
	QLabel* lbl = new QLabel("");
	_layout->addWidget(lbl, 0,1 ); // own _slider
	
  
  /*
	QPushButton* btnPasteQ = new QPushButton("Paste", this);
	_layout->addWidget(btnPasteQ, 0,0);
	connect(btnPasteQ, SIGNAL(clicked()), this, SLOT(paste()));
  */
  
    for (size_t i = 0; i<bounds.first.size(); i++) {
        Slider* slider = new Slider(titles[i], bounds.first(i), bounds.second(i), _layout, (int)i+2, this);
        slider->setValue(q(i));
        connect(slider, SIGNAL(valueChanged()), this, SLOT(valueChanged()));
        _sliders.push_back(slider);
    }
	_layout->addWidget(new QLabel(""), (int)bounds.first.size()+2,1 ); // own _slider
	_layout->setRowStretch((int)bounds.first.size()+2, 1);
}



void JointSliderWidget::paste() {

	QString txt = "";
	do {
		txt = QInputDialog::getText(this, tr("RobWorkStudio Jog"), tr("Paste Q"), QLineEdit::Normal, txt);
		if (txt.isEmpty())
			return;

		std::istringstream sstr;
		sstr.str(txt.toStdString());

		try {
			Q q;
			sstr >> q;
			
			if (q.size() != _sliders.size()) {
				QMessageBox::critical(this, tr("RobWorkStudio Jog"), tr("Number of elements does not match device!"));
				continue;
			}
      
      for(unsigned int i = 0; i < _sliders.size(); ++i)
        q[i] /= _sliders[i]->getUnitConverter();
        
			updateValues(q);
			return;
		}
		catch (const Exception&) {
			QMessageBox::critical(this, tr("RobWorkStudio Jog"), tr("Unable to parse '%1' as Q").arg(txt));
			continue;
		}
	} while (true);

}

void JointSliderWidget::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    RW_ASSERT(_sliders.size() == converters.size());
    for(size_t i = 0; i < _sliders.size(); ++i) {
        _sliders[i]->setUnitConverter(converters[i]);
        _sliders[i]->setUnitDescription(descriptions[i]);
        _sliders[i]->unitUpdated();
    }

}

void JointSliderWidget::updateValues(const rw::math::Q& q) {
    for (size_t i = 0; i<q.size(); i++) {
        _sliders[i]->setValue(q(i));
    }
}

rw::math::Q JointSliderWidget::getQ() {
    Q q(_sliders.size());
    for (size_t i = 0; i<_sliders.size(); i++) {
        q(i) = _sliders[i]->value();
    }
    return q;
}


void JointSliderWidget::valueChanged() {
    valueChanged(getQ());
}


TransformSliderWidget::TransformSliderWidget(const std::pair<rw::math::Q, rw::math::Q>& bounds, const rw::math::Transform3D<>& transform):
    _updating(false)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned

    _jointSliderWidget = new JointSliderWidget();
    Q q = transform2q(transform);
    std::vector<std::string> titles(6);
    titles[0] = "x"; titles[1] = "y"; titles[2] = "z";
    titles[3] = "R"; titles[4] = "P"; titles[5] = "Y";

    _jointSliderWidget->setup(titles, bounds, q);
    connect(_jointSliderWidget,
            SIGNAL(valueChanged(const rw::math::Q&)),
            this,
            SLOT(valueChanged(const rw::math::Q&)));
    tablayout->addWidget(_jointSliderWidget, 0, 0);
}


void TransformSliderWidget::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    _jointSliderWidget->setUnits(converters, descriptions);
}

void TransformSliderWidget::updateValues(const Transform3D<>& transform) {
    if (_updating)
        return;
    Q q = transform2q(transform);
    _updating = true;
    _jointSliderWidget->updateValues(q);
    _updating = false;
}

rw::math::Transform3D<> TransformSliderWidget::getTransform() {
    Q q = _jointSliderWidget->getQ();
    return q2transform(q);
}

void TransformSliderWidget::valueChanged(const rw::math::Q& q) {
    if (_updating)
        return;
    Transform3D<> transform = q2transform(q);

    valueChanged(transform);
}

void TransformSliderWidget::paste() {
  _jointSliderWidget->paste();
}

CartesianDeviceTab::CartesianDeviceTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
									   Device::Ptr device,
                                       WorkCell* workcell,
                                       const rw::kinematics::State& state):
    _state(state),
    _device(device),
    _updating(true)
{
    QGridLayout* tablayout = new QGridLayout(this); //owned
    QWidget* toppanel = new QWidget();
    QGridLayout* toplayout = new QGridLayout(toppanel);



    _frames = Kinematics::findAllFrames(workcell->getWorldFrame(), state);
    _cmbTcpFrame = new QComboBox();
    _cmbRefFrame = new QComboBox();
    int i = 0;
    for (std::vector<Frame*>::iterator it = _frames.begin(); it != _frames.end(); ++it, ++i) {
        _cmbTcpFrame->addItem((*it)->getName().c_str(), QVariant(i));
        _cmbRefFrame->addItem((*it)->getName().c_str(), QVariant(i));
    }
    connect(_cmbTcpFrame, SIGNAL(currentIndexChanged(int)), this, SLOT(tcpFrameChanged(int)));
    connect(_cmbRefFrame, SIGNAL(currentIndexChanged(int)), this, SLOT(refFrameChanged(int)));


    _tcpFrame = device->getEnd();
    _refFrame = workcell->getWorldFrame(); //Setup the reference frame

    _baseTref = FKRange(_device->getBase(), _refFrame, _state);
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);


    _cmbTcpFrame->setCurrentIndex(_cmbTcpFrame->findText(_tcpFrame->getName().c_str()));
    _cmbRefFrame->setCurrentIndex(_cmbRefFrame->findText(_refFrame->getName().c_str()));

    toplayout->addWidget(new QLabel("TCP Frame: "), 0,0);
    toplayout->addWidget(_cmbTcpFrame, 0,1);

    toplayout->addWidget(new QLabel("Ref. Frame: "), 1,0);
    toplayout->addWidget(_cmbRefFrame, 1,1);


    tablayout->addWidget(toppanel, 0, 0);

    _transformSliderWidget = new TransformSliderWidget(bounds, Kinematics::frameTframe(_refFrame, _tcpFrame, _state));    
    
    QPushButton* btnPasteQ = new QPushButton("Paste", _transformSliderWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(new QLabel(""));
    btnlayout->addWidget(btnPasteQ);
    tablayout->addLayout(btnlayout, 1, 0);
    connect(btnPasteQ, SIGNAL(clicked()), _transformSliderWidget, SLOT(paste()));    
    
    connect(_transformSliderWidget,
            SIGNAL(valueChanged(const rw::math::Transform3D<>&)),
            this,
            SLOT(transformChanged(const rw::math::Transform3D<>&)));

    tablayout->addWidget(_transformSliderWidget, 2, 0);


    _iksolver = ownedPtr(new JacobianIKSolver(_device, _tcpFrame, _state));


    _updating = false; 
}

void CartesianDeviceTab::setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions) {
    _transformSliderWidget->setUnits(converters, descriptions);
}

void CartesianDeviceTab::tcpFrameChanged(int index) {
    _tcpFrame = _frames[index];
    _iksolver = ownedPtr(new JacobianIKSolver(_device, _tcpFrame, _state));
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);

    doUpdateValues();
}


void CartesianDeviceTab::refFrameChanged(int index) {
    _refFrame = _frames[index];
    _baseTref = FKRange(_device->getBase(), _refFrame, _state);
    _refTtcp = FKRange(_refFrame, _tcpFrame, _state);
    doUpdateValues();
}


#include <rw/math/Jacobian.hpp>

void CartesianDeviceTab::transformChanged(const Transform3D<>& refTtcp) {
    Transform3D<> baseTref = _baseTref.get(_state);
    Transform3D<> baseTtcp = baseTref*refTtcp;

    IKMetaSolver metaSolver(_iksolver.get(), _device, (CollisionDetector*)NULL);
    metaSolver.setMaxAttempts(50);
    std::vector<Q> solutions = metaSolver.solve(baseTtcp, _state);
    if (solutions.empty()) {
        doUpdateValues();
    } else {
        _device->setQ(solutions.front(), _state);
        _updating = true;
        stateChanged(_state);
        _updating = false;
    }

}

void CartesianDeviceTab::updateValues(const rw::kinematics::State& state) {
    _state = state;
    doUpdateValues();
}

void CartesianDeviceTab::doUpdateValues() {
    if (_updating)
        return;
    Transform3D<> refTtcp = _refTtcp.get(_state);
    _transformSliderWidget->updateValues(refTtcp);
}



