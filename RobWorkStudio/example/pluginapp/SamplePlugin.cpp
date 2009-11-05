#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;

//using namespace rwhw;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("Sample", QIcon(":/pa_icon.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Button0");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Button1");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    pLayout->setRowStretch(row,1);
}


SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::open(WorkCell* workcell)
{
}

void SamplePlugin::close() {
}

void SamplePlugin::clickEvent() {
    QObject *obj = sender();
    if(obj == _btn0){
        log().info() << "Button 0 pressed!\n";
    } else if(obj == _btn1){
        log().info() << "Button 1 pressed!\n";
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

Q_EXPORT_PLUGIN(SamplePlugin);
