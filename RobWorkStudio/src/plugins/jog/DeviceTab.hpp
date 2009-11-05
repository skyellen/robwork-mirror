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


#ifndef DEVICETAB_H
#define DEVICETAB_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QGridLayout>

#include <rw/kinematics/State.hpp>
#include <rw/models/Device.hpp>
#include <rw/math/Q.hpp>

// The widget for a single joint.
class JointLine : public QWidget
{
    Q_OBJECT

public:
    JointLine(double low,
              double high,
              QGridLayout* layout,
              int row,
              QWidget* parent);

    // The current value of the joint.
    double value() const;

    // Set a value for the joint.
    void setValue(double val);

private slots:
    void boxValueChanged(double val);
    void sliderValueChanged(int val);

signals:
    // Emitted whenever the joint value changes.
    void valueChanged();

private:
    void setSliderValueFromBox(double val);
    void setBoxValueFromSlider(int val);

private:
    double _low;
    double _high;

    QSlider* _slider;
    QDoubleSpinBox* _box;

    bool _boxChanged;
    bool _sliderChanged;
};

class DeviceTab: public QWidget
{
    Q_OBJECT

public:
    DeviceTab(
        rw::models::Device* device,
        rw::kinematics::State* state);

    ~DeviceTab();

    void updateDisplayValues();

public slots:
    void valueChanged();

signals:
    void updateSignal();

private:
    rw::models::Device* _device;
    rw::kinematics::State* _state;

    size_t _n;
    bool _updating;
    std::vector<JointLine*> _joints;

private:
    rw::math::Q getQ() const
    { return _device->getQ(*_state); }

    void setQ(const rw::math::Q& q)
    { _device->setQ(q, *_state); }
};

#endif //#ifndef DEVICETAB_H
