/************************************************************************
 * RobWorkStudio Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#include "JogGroup.hpp"

#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSlider>
#include <QWheelEvent>

using rw::math::Q;

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
    _sliderChanged(false),
    _sliderResolutionInt(4000),
    _stepSize((high-low)/_sliderResolutionInt)
{


    //QLabel* lowLabel = makeNumericQLabel(low);
    _slider = makeHSlider(_sliderResolutionInt);
    //QLabel* highLabel = makeNumericQLabel(high);
    _box = makeDoubleSpinBox(low, high);

    //layout->addWidget(lowLabel, row, 0, Qt::AlignRight); // own lowLabel
    layout->addWidget(_slider, row, 1); // own _slider
    //layout->addWidget(highLabel, row, 2); // own highLabel
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
    _slider->setValue((int)((val - _low) / (_high - _low) * _sliderResolutionInt));
}

void JointLine::setBoxValueFromSlider(int val)
{
    _box->setValue(((double)val / _sliderResolutionInt) * (_high - _low) + _low);
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
        _slider->setValue((int)((val - _low) / (_high - _low) * _sliderResolutionInt));
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

void JointLine::wheelEvent(QWheelEvent* event)
{
    double sval = event->delta()*(_high-_low)/40;
    boxValueChanged( sval + value() );
}

//----------------------------------------------------------------------
// JogGroup



JogGroup::JogGroup(const std::pair<Q,Q>& bounds):
    _n(bounds.first.size()),
    _updating(false),
    _q(Q::zero((int)bounds.first.size()))
{
    QGridLayout* layout = new QGridLayout(this); // owned

    // hack so that we can move the first lider with mouse
    layout->addWidget(new QLabel(""), 0, 1); // own _slider
    layout->addWidget(new QLabel(""), 0, 3); // own _slider

    //const std::pair<Q, Q>& bounds = device.getBounds();
    std::cout << "Device bounds: "<< bounds.first << " " << bounds.second << std::endl;
    for (size_t i = 1; i < _n+1; i++) {
        const double low = bounds.first(i-1);
        const double high = bounds.second(i-1);
        JointLine* line = new JointLine(low, high, layout, (int)i, this); // owned

        connect(line, SIGNAL(valueChanged()), this, SLOT(valueChanged()));
        _joints.push_back(line);
    }

    // The last (empty) row grows! That way we make sure that the rows are
    // aligned towards the top.
    layout->setRowStretch((int)(_n+1), 1);

    updateDisplayValues();
}

JogGroup::~JogGroup()
{}

void JogGroup::updateDisplayValues()
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

void JogGroup::valueChanged()
{
    if (_updating)
        return;

    Q q(_n);
    for (size_t i = 0; i < _n; i++) {
        q(i) = _joints[i]->value();
    }

    setQ(q);
    emit updateSignal();
}


