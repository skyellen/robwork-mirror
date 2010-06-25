#ifndef JOINTCONTROLDIALOG_HPP_
#define JOINTCONTROLDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include <QDialog>
#include <QFileInfo>
#include <QString>
#include <QTabWidget>

#include "JogGroup.hpp"
#include <sandbox/control/JointController.hpp>

class SimulatorDialog : public QDialog
{
    Q_OBJECT

public:
    SimulatorDialog(Simulator* jcontroller, rw::kinematics::State* state, QWidget *parent = 0);

private:
    QTabWidget *tabWidget;
    rw::kinematics::State *_state;
};


class SyncTab : public QWidget
{
    Q_OBJECT

public:
    SyncTab(JointController* jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {

    }
    virtual ~SyncTab(){};

};

class PosTab : public QWidget
{
    Q_OBJECT

public:
    PosTab(JointController* jcontroller, rw::kinematics::State* state,QWidget *parent=0);

    virtual ~PosTab(){};

private:

private slots:
    void targetChanged();
    void setTarget();

private:
    JogGroup *_jogGroup;
    rw::kinematics::State *_state;
    JointController *_jcont;
};

class VelTab : public QWidget
{
    Q_OBJECT

public:
    VelTab(JointController* jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {
        //if( !(jcontroller->getControlModes() & JointController::VELOCITY) )
            this->setDisabled(true);

    }
    virtual ~VelTab(){};
};

class CurTab : public QWidget
{
    Q_OBJECT

public:
    CurTab(JointController* jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {
        if( !(jcontroller->getControlModes() & JointController::CURRENT) )
            this->setDisabled(true);

    }
    virtual ~CurTab(){};
};



#endif /*CUBECONTROLDIALOG_HPP_*/
