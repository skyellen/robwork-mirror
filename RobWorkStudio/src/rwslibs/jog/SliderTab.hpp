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

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/Q.hpp>

#include <rw/common/Ptr.hpp>

namespace rw { namespace invkin { class IterativeIK; } }
namespace rw { namespace kinematics { class MovableFrame; } }
namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class WorkCell; } }

class QDoubleSpinBox;
class QSlider;
class QGridLayout;
class QComboBox;
class QLabel;

// The widget for a single joint.
class Slider : public QWidget
{
    Q_OBJECT

public:
    Slider(const std::string& title,
           double low,
           double high,
           QGridLayout* layout,
           int row,
           QWidget* parent);

    void unitUpdated();

    // The current value of the joint.
    double value() const;

    // Set a value for the joint.
    void setValue(double val);

    void setUnitConverter(double converter){
        _toUnit = converter;
    }

    double getUnitConverter() const {
        return _toUnit;
    }

    void setUnitDescription(const std::string& str){
        _desc = str;
    }

    void showEndLabel(bool enabled){
        _endlabelEnabled = enabled;
    }

private slots:
    void boxValueChanged(double val);
    void sliderValueChanged(int val);

signals:
    // Emitted whenever the joint value changes.
    void valueChanged();

private:
    void setSliderValueFromBox(double val);
    void setBoxValueFromSlider(int val);

    double _low;
    double _high;

    QSlider* _slider;
    QDoubleSpinBox* _box;

    bool _boxChanged;
    bool _sliderChanged;

    QLabel *_title, *_lowLabel, *_highLabel;

    double _toUnit;
    std::string _desc;

    bool _endlabelEnabled;
};


class JointSliderWidget: public QWidget {
  Q_OBJECT

public:
    JointSliderWidget();

    void setup(const std::vector<std::string>& titles,
               const std::pair<rw::math::Q,rw::math::Q>& bounds,
               const rw::math::Q& q);

    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    void updateValues(const rw::math::Q& q);

    rw::math::Q getQ();

signals:
    void valueChanged(const rw::math::Q& q);
    
public slots:
    void paste();

private slots:
    void valueChanged();
    
private:
    std::vector<Slider*> _sliders;

    QGridLayout* _layout;
};




class TransformSliderWidget: public QWidget {
    Q_OBJECT
public:
    TransformSliderWidget(const std::pair<rw::math::Q, rw::math::Q>& bounds, const rw::math::Transform3D<>& transform);

    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    void updateValues(const rw::math::Transform3D<>& transform);

    rw::math::Transform3D<> getTransform();

signals:
    void valueChanged(const rw::math::Transform3D<>& transform);
    
public slots:
    void paste();
    
private slots:
    void valueChanged(const rw::math::Q& q);
    
private:
    JointSliderWidget* _jointSliderWidget;

    bool _updating;
};



class MovableFrameTab: public QWidget {
    Q_OBJECT

public:
    MovableFrameTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
                 rw::kinematics::MovableFrame* frame,
                 rw::models::WorkCell* workcell,
                 const rw::kinematics::State& state);

    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

   // void setup(const std::pair<rw::math::Q, rw::math::Q>& bounds, rw::kinematics::Frame* frame);

    void updateValues(const rw::kinematics::State& state);

signals:
    void stateChanged(const rw::kinematics::State& state);

private slots:
    void transformChanged(const rw::math::Transform3D<>& transform);
    void refFrameChanged(int index);

private:
    std::vector<Slider*> _sliders;
    QGridLayout* _layout;
    QComboBox* _cmbFrames;
    std::vector<rw::kinematics::Frame*> _frames;
    rw::kinematics::State _state;
    rw::kinematics::MovableFrame* _frame;
    rw::kinematics::Frame* _refframe;

    void doUpdateValues();

    TransformSliderWidget* _transformSliderWidget;
	bool _updating;
};



class CartesianDeviceTab: public QWidget {
    Q_OBJECT
public:
    CartesianDeviceTab(const std::pair<rw::math::Q, rw::math::Q>& bounds,
    	rw::common::Ptr<rw::models::Device> device,
		rw::models::WorkCell* workcell,
        const rw::kinematics::State& state);

    void setUnits(const std::vector<double>& converters, const std::vector<std::string>& descriptions);

    void updateValues(const rw::kinematics::State& state);

signals:
    void stateChanged(const rw::kinematics::State& state);

private slots:
    void transformChanged(const rw::math::Transform3D<>& transform);

    void tcpFrameChanged(int index);
    void refFrameChanged(int index);

private:
    QComboBox* _cmbRefFrame;
    QComboBox* _cmbTcpFrame;

    rw::kinematics::State _state;
	rw::common::Ptr<rw::models::Device> _device;
    std::vector<rw::kinematics::Frame*> _frames;
    rw::kinematics::Frame* _tcpFrame;
    rw::kinematics::Frame* _refFrame;

    rw::kinematics::FKRange _baseTref;
    rw::kinematics::FKRange _refTtcp;


    TransformSliderWidget* _transformSliderWidget;
    rw::common::Ptr<rw::invkin::IterativeIK> _iksolver;

    bool _updating;

    void doUpdateValues();
};



#endif //#ifndef DEVICETAB_H
