#ifndef BODYCONTROLLERWIDGET_HPP_
#define BODYCONTROLLERWIDGET_HPP_

#include <rwsim/control/BodyController.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>

#include "JogGroup.hpp"

#include <QDialog>
#include <QFileInfo>
#include <QString>
#include <QTabWidget>
#include <QComboBox>



/**
 * @brief dialog used to control dynamic bodies in the scene
 */
class BodyControlDialog : public QDialog
{
    Q_OBJECT

public:
    BodyControlDialog(rwsim::dynamics::DynamicWorkCell::Ptr dwc,
                      rwsim::control::BodyController::Ptr bodycontroller,
                      QWidget *parent = 0);

    BodyControlDialog(rwsim::dynamics::DynamicWorkCell::Ptr dwc,
                      rwsim::simulator::DynamicSimulator::Ptr simulator,
                      QWidget *parent = 0);

    virtual ~BodyControlDialog(){}



private:
    QComboBox *_bodyBox;
    QTabWidget *tabWidget;

    rwsim::control::BodyController::Ptr _bodyctrl;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    JogGroup *_jogGroup;
};


#endif /* CONTROLLERWIDGET_HPP_ */
