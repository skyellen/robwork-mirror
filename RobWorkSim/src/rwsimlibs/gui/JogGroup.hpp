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

#ifndef JOGGROUP_H
#define JOGGROUP_H

#include <QWidget>

#include <rw/math/Q.hpp>

class QDoubleSpinBox;
class QSlider;
class QGridLayout;
class QWheelEvent;

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

    void setSliderResolution(double res);

private slots:
    void boxValueChanged(double val);
    void sliderValueChanged(int val);
    void wheelEvent(QWheelEvent* event);

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

    int _sliderResolutionInt;
    double _stepSize;
};

class JogGroup: public QWidget
{
    Q_OBJECT

public:
    JogGroup(const std::pair<rw::math::Q,rw::math::Q>& device);

    ~JogGroup();

    void updateDisplayValues();

    rw::math::Q getQ() const
    { return _q; }

    void setQ(const rw::math::Q& q)
    { _q = q; }

public slots:
    void valueChanged();

signals:
    void updateSignal();

private:

    size_t _n;
    bool _updating;
    std::vector<JointLine*> _joints;
    rw::math::Q _q;
private:


};

#endif //#ifndef JogGroup_H
