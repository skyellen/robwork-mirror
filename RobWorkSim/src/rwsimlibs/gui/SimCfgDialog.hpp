/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef SIMULATORCFGDIALOG_HPP_
#define SIMULATORCFGDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <QDialog>

namespace Ui {
    class SimCfgDialog;
}

class SimCfgDialog : public QDialog
    {
        Q_OBJECT

    public:
        SimCfgDialog(rw::common::Ptr<rwsim::simulator::DynamicSimulator> sim, QWidget *parent = 0);

    private slots:
        void btnPressed();
        void changedEvent();

    private:
        Ui::SimCfgDialog *_ui;

        rw::common::Ptr<rwsim::simulator::DynamicSimulator> _sim;
};


#endif /* SimulatorCfgDialog_HPP_ */
