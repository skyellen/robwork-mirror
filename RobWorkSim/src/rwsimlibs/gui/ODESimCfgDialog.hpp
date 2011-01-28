/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef ODESIMCFGDIALOG_HPP_
#define ODESIMCFGDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_ODESimCfgForm.h"

#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkcell.hpp>
#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

class ODESimCfgDialog : public QDialog, private Ui::ODESimCfgForm
    {
        Q_OBJECT

    public:
        ODESimCfgDialog(rw::common::Ptr<rwsim::simulator::DynamicSimulator> sim, QWidget *parent = 0);

        void initializeStart();


    signals:
        void updateView();

    private slots:
        void btnPressed();
        void changedEvent();

    private:
        void updateStatus();
        void applyChanges();
        void updateValues();
    private:
        Ui::ODESimCfgForm _ui;

        rw::common::Ptr<rwsim::simulator::DynamicSimulator> _sim;
};


#endif /* ODESimCfgDialog_HPP_ */
