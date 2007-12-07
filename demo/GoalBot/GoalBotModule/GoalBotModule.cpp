#include "GoalBotModule.hpp"

#include <QPushButton>
#include <QLayout>
#include <QMessageBox>
#include <QIcon>
#include <QLabel>

#include <math/Transform3D.hpp>
#include <math/EAA.hpp>
#include <math/Vector3D.hpp>

using namespace rw::vworkcell;
using namespace rw::math;
using namespace boost::numeric::ublas;

GoalBotModule::GoalBotModule(): 
    QTGuiModule("GoalBot", QIcon(":/images/usermodule.hpp")),
    _device(NULL)
{

    _timer = new QTimer();
    _timer->setInterval(1000/20); //20fps    //TODO If gui is updating to fast or slow adjust this parameter
    connect(_timer, SIGNAL(timeout()), this, SLOT(updateTimer()));
    _timer->start();

    _showGoalBotAction = new QAction(QIcon("GoalBot.gif"), tr("GoalBot"), this);
    connect(_showGoalBotAction, SIGNAL(triggered()), this, SLOT(showGoalBot()));
    QPushButton* btnCalibrate = new QPushButton("Calibrate");
    this->layout()->addWidget(btnCalibrate);
    connect(btnCalibrate, SIGNAL(clicked()), this, SLOT(calibrate()));

    QPushButton* btnCaptureBackground = new QPushButton("Capture Background");
    this->layout()->addWidget(btnCaptureBackground);
    connect(btnCaptureBackground, SIGNAL(clicked()), this, SLOT(captureBackground()));

    QPushButton* btnStartGame = new QPushButton("Start Game");
    this->layout()->addWidget(btnStartGame);
    connect(btnStartGame, SIGNAL(clicked()), this, SLOT(startGame()));

    QPushButton* btnEndGame = new QPushButton("End Game");
    this->layout()->addWidget(btnEndGame);
    connect(btnEndGame, SIGNAL(clicked()), this, SLOT(endGame()));

    QPushButton* btnEmStop = new QPushButton("Emergency Stop");
    this->layout()->addWidget(btnEmStop);
    connect(btnEmStop, SIGNAL(clicked()), this, SLOT(emergencyStop()));


    QPushButton* btnInitialize = new QPushButton("Initialize PA10");
    this->layout()->addWidget(btnInitialize);
    connect(btnInitialize, SIGNAL(clicked()), this, SLOT(initializeRobot()));

  


    _jointJog = new DeviceJogWidget();
    this->layout()->addWidget(new QLabel("Joint Space", this));
    this->layout()->addWidget(_jointJog);
    connect(_jointJog, SIGNAL(newTarget(const rw::vworkcell::Device::Q&)), this, SLOT(moveJoints(const rw::vworkcell::Device::Q&)));

    _cartJog = new DeviceJogWidget();
    this->layout()->addWidget(new QLabel("Cartesian XYZ, Euler RPY", this));
    this->layout()->addWidget(_cartJog);
    connect(_cartJog, SIGNAL(newTarget(const rw::vworkcell::Device::Q&)), this, SLOT(moveCartesian(const rw::vworkcell::Device::Q&)));



}

GoalBotModule::~GoalBotModule() {

}



void GoalBotModule::initializeRobot() {
    std::cout<<"Initialize Robot "<<std::endl;
    if (_device != NULL) {
        _pa10Thread = new PA10ControlThread(_device);
        _pa10Thread->start(QThread::TimeCriticalPriority); //TODO It might be necessary to set a higher priority - consult QThread class doc
        emit updateSignal();
        _jointJog->setup(_device->getBounds(), _device->getConfiguration());

        Device::Q lower(6), upper(6);
        lower(0) = lower(1) = lower(2) = -2;
        lower(3) = lower(4) = lower(5) = -2*M_PI;
        upper(0) = upper(1) = upper(2) = 2;
        upper(3) = upper(4) = upper(5) = 2*M_PI;

        _cartJog->setup(std::pair<Device::Q, Device::Q>(lower, upper), Pose6D<>(_device->bTe()));

    }
    else {
        QMessageBox::information(this, "Information", "No feasible Device is loaded", QMessageBox::Ok);

    }

}



void GoalBotModule::moveJoints(const Device::Q& target) {
    _pa10Thread->moveTo(target);
    emit updateSignal();
}

void GoalBotModule::moveCartesian(const Device::Q& target) {
    _pa10Thread->moveTo(Transform3D<>(Vector3D<>(target(0), target(1), target(2)), EAA<>(target(3), target(4), target(5))));
    emit updateSignal();

}

void GoalBotModule::calibrate() {
    //TODO Event handler for Calibrate Button
    std::cout<<"Calibrate "<<std::endl;
}

void GoalBotModule::captureBackground() {
    //TODO Event handler for captureBackground button
    std::cout<<"capture Background"<<std::endl;
}

void GoalBotModule::startGame() {
    //TODO Event handler for start Game button
    std::cout<<"Start Game "<<std::endl;
}

void GoalBotModule::endGame() {
    //TODO Event handler for end game button
    std::cout<<"End Game"<<std::endl;
}

void GoalBotModule::emergencyStop() {
    _pa10Thread->emergencyStop();
}

void GoalBotModule::setupMenu(QMenu* menu) {
    menu->addAction(_showGoalBotAction);
}

void GoalBotModule::setupToolBar(QToolBar* toolbar) {
    toolbar->addAction(_showGoalBotAction);
}

void GoalBotModule::setup(WorkCell* workcell) {
    _workcell = workcell;

    if(_workcell->getDevices().size() != 0 && _workcell->getDevices()[0]->getName() == "pa10"){

        const std::vector<Device*>& devices = workcell->getDevices();
        if (devices.size() > 0) {
            _device = dynamic_cast<SerialDevice*>(devices.front());
            Device::Q q = zero_vector<double>(7);
            q(3) = 0.4;
            _device->setConfiguration(q);
            initializeRobot();
        } else {
            std::cout<<"WorkCell without devices "<<std::endl;
        }

    }
}

void GoalBotModule::close() {
    _workcell = NULL;
}


QString GoalBotModule::name() const {
    return "GoalBot";
}

void GoalBotModule::showGoalBot() {
    if (isVisible()) {
        setVisible(false);
    } else {
        this->show();
    }
}

void GoalBotModule::updateTimer() {
    if (_device != NULL) {    
        _jointJog->updatePosition(_device->getConfiguration());
        _cartJog->updatePosition(Pose6D<>(_device->bTe()));
    }
    emit updateSignal();
}
