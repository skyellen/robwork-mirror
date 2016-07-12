#include "JointControlDialog.hpp"

#include "JogGroup.hpp"

#include <rwlibs/control/JointController.hpp>
#include <rwsim/dynamics/DynamicDevice.hpp>

#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

using namespace rwlibs::control;

VelTab::VelTab(rwlibs::control::JointController::Ptr jcontroller, QWidget *parent)
: QWidget(parent)
{

    if( !(jcontroller->getControlModes() & JointController::VELOCITY) )
        this->setDisabled(true);

}



PosTab::PosTab(JointControllerPtr jcontroller, QWidget *parent)
       : QWidget(parent),_jcont(jcontroller)
{
   if( !(jcontroller->getControlModes() & (JointController::POSITION | JointController::CNT_POSITION)) )
       this->setDisabled(true);

   int row = 0;

   QGridLayout* lay = new QGridLayout(this);
   this->setLayout(lay);

   _jogGroup = new JogGroup( jcontroller->getModel().getBounds() );
   lay->addWidget(_jogGroup, row++, 0);
   connect(_jogGroup, SIGNAL(updateSignal()), this, SLOT(targetChanged()));

   {
       QPushButton *button = new QPushButton(tr("Set target"));
       lay->addWidget(button,row++,0);
       connect(button, SIGNAL(clicked()), this, SLOT(setTarget()));

   }
   {
       //QPushButton *button = new QPushButton(tr("Get q state"));
       //lay->addWidget(button,row++,0);
       //connect(button, SIGNAL(clicked()), this, SLOT(getQState()));

   }

}

void PosTab::setTarget(){
    rw::math::Q target = _jogGroup->getQ();
    _jcont->setTargetPos( target );
}

void PosTab::targetChanged(){
    std::cout << "Target Changed: " << _jogGroup->getQ() << std::endl;
}

JointControlDialog::JointControlDialog(rwsim::dynamics::DynamicDevice::Ptr device, QWidget *parent)
{
    tabWidget = new QTabWidget;

    //tabWidget->addTab(new SyncTab(jcontroller), tr("Sync"));
    tabWidget->addTab(new VelTab(device), tr("Vel"));

    QPushButton *okButton = new QPushButton(tr("OK"));
    QPushButton *cancelButton = new QPushButton(tr("Cancel"));

    connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(tabWidget);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    setWindowTitle(tr("Joint Control"));
}


JointControlDialog::JointControlDialog(
        JointController::Ptr jcontroller,
        QWidget *parent)
    : QDialog(parent), _controller(jcontroller)
{
    tabWidget = new QTabWidget;

    //tabWidget->addTab(new SyncTab(jcontroller), tr("Sync"));
    if( jcontroller->getControlModes() & (JointController::POSITION | JointController::CNT_POSITION ) )
        tabWidget->addTab(new PosTab(jcontroller), tr("Pos"));
    if( jcontroller->getControlModes() & (JointController::VELOCITY ) )
        tabWidget->addTab(new VelTab(jcontroller), tr("Vel"));
    if( jcontroller->getControlModes() & (JointController::CURRENT ) )
        tabWidget->addTab(new CurTab(jcontroller), tr("Cur"));

    QPushButton *okButton = new QPushButton(tr("OK"));
    QPushButton *cancelButton = new QPushButton(tr("Cancel"));

    connect(okButton, SIGNAL(clicked()), this, SLOT(accept()));
    connect(cancelButton, SIGNAL(clicked()), this, SLOT(reject()));

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(tabWidget);
    mainLayout->addLayout(buttonLayout);
    setLayout(mainLayout);

    setWindowTitle(tr("Joint Control"));
}

CurTab::CurTab(rw::common::Ptr<rwlibs::control::JointController> jcontroller, QWidget *parent):
		QWidget(parent)
{
	if( !(jcontroller->getControlModes() & rwlibs::control::JointController::CURRENT) )
		this->setDisabled(true);

}
