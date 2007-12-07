#ifndef DEVICEJOGWIDGET_HPP
#define DEVICEJOGWIDGET_HPP

#include <QWidget>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <vworkcell/Device.hpp>

#include <math/Pose6D.hpp>

class DeviceJogWidget: public QWidget {
Q_OBJECT
public:
    DeviceJogWidget();
    ~DeviceJogWidget();

    void setup(const std::pair<rw::vworkcell::Device::Q, rw::vworkcell::Device::Q>& bounds, const rw::vworkcell::Device::Q& q);
    void setup(const std::pair<rw::vworkcell::Device::Q, rw::vworkcell::Device::Q>& bounds, const rw::math::Pose6D<>& p);

public slots:
    void updatePosition(const rw::vworkcell::Device::Q& qreal); 
    void updatePosition(const rw::math::Pose6D<>& pose); 

private slots:
    void valueChanged(); 


signals:
    void newTarget(const rw::vworkcell::Device::Q& target);


private:
    size_t _n;
    QDoubleSpinBox** _ppJoints;
    QLineEdit** _ppJointReal;
};


#endif //#ifndef DEVICEJOGWIDGET_HPP
