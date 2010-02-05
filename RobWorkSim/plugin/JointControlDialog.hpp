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

class JointControlDialog : public QDialog
{
    Q_OBJECT

public:
    JointControlDialog(JointControllerPtr jcontroller, QWidget *parent = 0);

    virtual ~JointControlDialog(){
        std::cout << "*********************** JOINT DIALOG DESTRUCT" << std::endl;
    }

private:
    QTabWidget *tabWidget;
};


class SyncTab : public QWidget
{
    Q_OBJECT

public:
    SyncTab(JointControllerPtr jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {

    }
    virtual ~SyncTab(){};

};

class PosTab : public QWidget
{
    Q_OBJECT

public:
    PosTab(JointControllerPtr jcontroller,QWidget *parent=0);

    virtual ~PosTab(){};

private:

private slots:
    void targetChanged();
    void setTarget();

private:
    JogGroup *_jogGroup;
    JointControllerPtr _jcont;
};

class VelTab : public QWidget
{
    Q_OBJECT

public:
    VelTab(JointControllerPtr jcontroller, QWidget *parent=0)
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
    CurTab(JointControllerPtr jcontroller, QWidget *parent=0)
    : QWidget(parent)
    {
        if( !(jcontroller->getControlModes() & JointController::CURRENT) )
            this->setDisabled(true);

    }
    virtual ~CurTab(){};
};



#endif /*CUBECONTROLDIALOG_HPP_*/
