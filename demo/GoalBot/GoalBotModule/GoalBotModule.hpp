#ifndef GOALBOTMODULE_HPP
#define GOALBOTMODULE_HPP

#include <vworkcell/WorkCell.hpp>
#include <vworkcell/SerialDevice.hpp>

#include "../QTGuiModule.hpp"

#include "PA10ControlThread.hpp"
#include "DeviceJogWidget.hpp"

#include <QTimer>


class GoalBotModule: public QTGuiModule {
Q_OBJECT
public:
    GoalBotModule();
    virtual ~GoalBotModule();
    
    virtual void setupMenu(QMenu* menu);
    
    virtual void setupToolBar(QToolBar* toolbar);
    
    virtual void setup(rw::vworkcell::WorkCell* workcell);
    
    virtual void close();
    
    virtual QString name() const; 


public slots:
 void showGoalBot();


private slots:
void initializeRobot();

    void moveJoints(const rw::vworkcell::Device::Q& target);
    void moveCartesian(const rw::vworkcell::Device::Q& target);

    void calibrate();
    void captureBackground();
    void startGame();
    void endGame();
    void emergencyStop();

    void updateTimer();

  
private:
    QAction* _showGoalBotAction;
    QTimer* _timer;
    rw::vworkcell::WorkCell* _workcell;
    rw::vworkcell::SerialDevice* _device;
    PA10ControlThread* _pa10Thread;
    DeviceJogWidget* _jointJog;
    DeviceJogWidget* _cartJog;
};


#endif //GOALBOTMODULE_HPP
