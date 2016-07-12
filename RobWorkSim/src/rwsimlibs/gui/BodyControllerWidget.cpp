#include "BodyControllerWidget.hpp"

#include "JogGroup.hpp"

#include <rwsim/simulator/DynamicSimulator.hpp>

#include <QVBoxLayout>

using namespace rwsim::dynamics;
using namespace rwsim::control;
using namespace rw::math;

using rwsim::control::BodyController;

BodyControlDialog::BodyControlDialog(//DynamicWorkCell::Ptr dwc,
                  BodyController::Ptr bodycontroller,
                  QWidget *parent)
: QDialog(parent),_bodyctrl(bodycontroller)
{
    Q q(Q::zero(6));
    q[0] = 10; q[1] = 10; q[2] = 10;
    q[3] = 3.14; q[4] = 3.14; q[5] = 3.14;

    _jogGroup = new JogGroup( std::make_pair(-q, q) );

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(_jogGroup);
    //mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);
}

BodyControlDialog::BodyControlDialog(//rwsim::dynamics::DynamicWorkCell::Ptr dwc,
        rwsim::simulator::DynamicSimulator::Ptr simulator,
        QWidget *parent):
        QDialog(parent),
        _bodyctrl(simulator->getBodyController())
{
    Q q(Q::zero(6));
    q[0] = 10; q[1] = 10; q[2] = 10;
    q[3] = 3.14; q[4] = 3.14; q[5] = 3.14;

    //_jogGroup = new JogGroup( std::make_pair(-q, q) );
    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(_jogGroup);
    //mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);
}
