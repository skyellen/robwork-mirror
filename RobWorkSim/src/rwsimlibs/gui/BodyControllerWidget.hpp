#ifndef BODYCONTROLLERWIDGET_HPP_
#define BODYCONTROLLERWIDGET_HPP_

#include <rw/common/Ptr.hpp>

#include <QDialog>

namespace rwsim { namespace control { class BodyController; } }
//namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }

class JogGroup;

/**
 * @brief dialog used to control dynamic bodies in the scene
 */
class BodyControlDialog : public QDialog
{
    Q_OBJECT

public:
    BodyControlDialog(//rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc,
                      rw::common::Ptr<rwsim::control::BodyController> bodycontroller,
                      QWidget *parent = 0);

    BodyControlDialog(//rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc,
                      rw::common::Ptr<rwsim::simulator::DynamicSimulator> simulator,
                      QWidget *parent = 0);

    virtual ~BodyControlDialog(){}



private:
    rw::common::Ptr<rwsim::control::BodyController> _bodyctrl;
    //rw::common::Ptr<rwsim::simulator::DynamicSimulator> _sim;
    JogGroup *_jogGroup;
};


#endif /* CONTROLLERWIDGET_HPP_ */
