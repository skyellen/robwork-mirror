#include "SamplePlugin.hpp"

#include <QPushButton>
#include <QGridLayout>

#include <rw/geometry/QHullND.hpp>
#include <rw/math/Random.hpp>
#include <RobWorkStudio.hpp>

#include <boost/bind.hpp>

using rw::geometry::QHullND;
using rw::kinematics::State;
using namespace rw::math;
using rw::models::WorkCell;

using rws::RobWorkStudioPlugin;

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


        std::vector<VectorND<2> > vertices, hullvertices;
        vertices.resize(100);

        for(size_t i=0;i<vertices.size();i++){
            vertices[i][0] = Random::ran(-10,10);
            vertices[i][1] = Random::ran(-10,10);
        }

        QHullND<2> qhull;
        qhull.rebuild(vertices);

        hullvertices = qhull.getHullVertices();

        std::cout << "\n INPUT: \n";
        for(size_t i=0; i<vertices.size();i++){
            std::cout << vertices[i][0] << "\t" << vertices[i][1] << "\n";
        }

        std::cout << "\n OUTPUT: \n";
        for(size_t i=0; i<hullvertices.size();i++){
            std::cout << hullvertices[i][0] << "\t" << hullvertices[i][1] << "\n";
        }



    }
}

void SamplePlugin::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
