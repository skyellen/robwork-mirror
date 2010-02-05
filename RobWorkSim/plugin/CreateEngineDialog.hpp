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

#include "ui_CreateEngineDialog.h"

#include <rw/kinematics/State.hpp>

#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/Simulator.hpp>
#include <rw/common/Ptr.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

class CreateEngineDialog : public QDialog, private Ui::CreateEngineDialog
{
	Q_OBJECT

public:
	CreateEngineDialog(rw::common::Ptr<dynamics::DynamicWorkcell> dwc,
					   QWidget *parent = 0);

	Simulator *getSimulator(){ return _sim; };

private slots:
	void btnPressed();
	void changedEvent();

private:
	Ui::CreateEngineDialog _ui;
	Simulator *_sim;
	rw::common::Ptr<dynamics::DynamicWorkcell> _dwc;
};


#endif /* CreateEngineDialog_HPP_ */
