/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef SIMULATORCFGDIALOG_HPP_
#define SIMULATORCFGDIALOG_HPP_

#include <rw/common/Ptr.hpp>

#include <QObject>
#include <QDialog>

namespace rwsim { namespace simulator { class DynamicSimulator; } }

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
};


#endif /* SimulatorCfgDialog_HPP_ */
