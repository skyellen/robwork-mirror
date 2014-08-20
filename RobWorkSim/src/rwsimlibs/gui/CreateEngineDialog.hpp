/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef CREATEENGINEDIALOG_HPP_
#define CREATEENGINEDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif


#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rw/common/Ptr.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <QDialog>

namespace Ui {
    class CreateEngineDialog;
}

class CreateEngineDialog : public QDialog
{
	Q_OBJECT

public:
	CreateEngineDialog(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, QWidget *parent = 0);

	rwsim::simulator::DynamicSimulator::Ptr getSimulator(){ return _sim; };

private slots:
	void btnPressed();
	void changedEvent();

private:
	Ui::CreateEngineDialog *_ui;
	rwsim::simulator::DynamicSimulator::Ptr _sim;
	rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
};


#endif /* CreateEngineDialog_HPP_ */
