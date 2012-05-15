#ifndef JOINTCONTROLDIALOG_HPP_
#define JOINTCONTROLDIALOG_HPP_

#include <QDialog>
#include <QFileInfo>
#include <QString>
#include <QTabWidget>

#include "JogGroup.hpp"
#include <rwlibs/control/JointController.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>

class JointControlDialog : public QDialog
{
    Q_OBJECT

public:
    JointControlDialog(rwlibs::control::JointController::Ptr jcontroller, QWidget *parent = 0);
    JointControlDialog(rwsim::dynamics::DynamicDevice::Ptr device, QWidget *parent = 0);

    virtual ~JointControlDialog(){}

private:
    QTabWidget *tabWidget;

    rwlibs::control::JointController::Ptr _controller;
    rwsim::dynamics::DynamicDevice::Ptr _device;
};


class SyncTab : public QWidget
{
    Q_OBJECT

public:
    SyncTab(rwlibs::control::JointControllerPtr jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {
    }
    virtual ~SyncTab(){};

};

class PosTab : public QWidget
{
    Q_OBJECT

public:
    PosTab(rwlibs::control::JointControllerPtr jcontroller,QWidget *parent=0);

    virtual ~PosTab(){};

private:

private slots:
    void targetChanged();
    void setTarget();

private:
    JogGroup *_jogGroup;
    rwlibs::control::JointControllerPtr _jcont;
};

class VelTab : public QWidget
{
    Q_OBJECT

public:
    VelTab(rwsim::dynamics::DynamicDevice::Ptr device, QWidget *parent=0)
    : QWidget(parent)
    {

    }

    VelTab(rwlibs::control::JointController::Ptr jcontroller, QWidget *parent=0);

    virtual ~VelTab(){};
};

class CurTab : public QWidget
{
    Q_OBJECT

public:
    CurTab(rwlibs::control::JointControllerPtr jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {
        if( !(jcontroller->getControlModes() & rwlibs::control::JointController::CURRENT) )
            this->setDisabled(true);

    }
    virtual ~CurTab(){};
};



#endif /*CUBECONTROLDIALOG_HPP_*/
