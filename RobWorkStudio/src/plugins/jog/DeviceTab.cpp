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


#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QLabel>
#include <QSlider>

#include "DeviceTab.hpp"

#include <rw/models/Device.hpp>
#include <rw/models/Joint.hpp>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

namespace
{
    QLabel* makeNumericQLabel(double val)
    {
        std::stringstream s; s << val;
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

        const double step = (high - low) / 100;
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
JointLine::JointLine(
    double low,
    double high,
    QGridLayout* layout,
    int row,
    QWidget* parent)
    :
    QWidget(parent),
    _low(low),
    _high(high),
    _boxChanged(false),
    _sliderChanged(false)
{
    QLabel* lowLabel = makeNumericQLabel(low);
    _slider = makeHSlider(sliderEnd);
    QLabel* highLabel = makeNumericQLabel(high);
    _box = makeDoubleSpinBox(low, high);

    layout->addWidget(lowLabel, row, 0, Qt::AlignRight); // own lowLabel
    layout->addWidget(_slider, row, 1); // own _slider
    layout->addWidget(highLabel, row, 2); // own highLabel
    layout->addWidget(_box, row, 3); // own _box

    connect(_box,
            SIGNAL(valueChanged(double)),
            this,
            SLOT(boxValueChanged(double)));

    connect(_slider,
            SIGNAL(valueChanged(int)),
            this,
            SLOT(sliderValueChanged(int)));

    setValue((low + high) / 2);
}

void JointLine::boxValueChanged(double val)
{
    _boxChanged = true;

    // Change the value of the slider.
    if (!_sliderChanged) {
        setSliderValueFromBox(val);
    }

    _sliderChanged = false;
    emit valueChanged();
}

void JointLine::sliderValueChanged(int val)
{
    _sliderChanged = true;

    // Change the value of the box.
    if (!_boxChanged) {
        setBoxValueFromSlider(val);
    }

    _boxChanged = false;

//    emit valueChanged();
}

void JointLine::setSliderValueFromBox(double val)
{
    _slider->setValue((int)((val - _low) / (_high - _low) * sliderEnd));
}

void JointLine::setBoxValueFromSlider(int val)
{
    _box->setValue(((double)val / sliderEnd) * (_high - _low) + _low);
}

double JointLine::value() const
{
    return _box->value();
}

void JointLine::setValue(double val)
{
    _sliderChanged = true;
    _boxChanged = true;

    if (_low <= val && val <= _high) {
        _slider->setValue(
            (int)((val - _low) / (_high - _low) * sliderEnd));
        _box->setValue(val);
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

//----------------------------------------------------------------------
// DeviceTab

DeviceTab::DeviceTab(
    Device* device,
    State* state)
    :
    _device(device),
    _state(state),
    _n(_device->getDOF()),
    _updating(false)
{
    QGridLayout* layout = new QGridLayout(this); // owned

    const std::pair<Q, Q>& bounds = _device->getBounds();
    for (size_t i = 0; i < _n; i++) {
        const double low = bounds.first(i);
        const double high = bounds.second(i);
        JointLine* line = new JointLine(low, high, layout, i, this); // owned

        connect(line, SIGNAL(valueChanged()), this, SLOT(valueChanged()));
        _joints.push_back(line);
    }

    // The last (empty) row grows! That way we make sure that the rows are
    // aligned towards the top.
    layout->setRowStretch(_n, 1);

    updateDisplayValues();
}

DeviceTab::~DeviceTab() {}

void DeviceTab::updateDisplayValues()
{
    _updating = true;

    const Q& q = getQ();
    for (int i = 0; i < (int)q.size(); i++) {
        // A minor optimization that probably isn't significant:
        if (_joints[i]->value() != q(i)) {
            _joints[i]->setValue(q(i));
        }
    }

    _updating = false;
}

void DeviceTab::valueChanged()
{
    if (_updating) return;

    Q q(_n);
    for (size_t i = 0; i < _n; i++) {
        q(i) = _joints[i]->value();
    }

    setQ(q);
    emit updateSignal();
}
