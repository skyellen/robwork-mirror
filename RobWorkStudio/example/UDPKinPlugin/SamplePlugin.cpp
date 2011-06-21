#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>


#include <iostream>

using boost::asio::ip::udp;

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;

using namespace rws;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

    _timer = new QTimer( NULL );
    _timer->setInterval( 10 );
    connect( _timer, SIGNAL(timeout()), this, SLOT(btnPressed()) );


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

namespace {


    Vector3D<> toXZX(const Quaternion<>& qn){
        Quaternion<> q = qn;
        q.normalize();
        Vector3D<> xzx;


        double u[4];
        u[0] = q(0);// W
        u[1] = q(1);
        u[2] = q(2);
        u[3] = q(3);

        double fcn1 =  2*(u[1]*u[3] - u[0]*u[2]);
        double fcn2 =  2*(u[1]*u[2] + u[0]*u[3]);
        double fcn3 =  u[0]*u[0] + u[1]*u[1] - u[2]*u[2] - u[3]*u[3];
        double fcn4 =  2*(u[1]*u[3] + u[0]*u[2]);
        double fcn5 = -2*(u[1]*u[2] - u[0]*u[3]);

        std::cout << fcn1  << " " << fcn2  << " " << fcn3  << " " << fcn4  << " " << fcn5  << std::endl;

        xzx(0) = atan2(fcn1, fcn2);
        xzx(1) = acos(fcn3);
        xzx(2) = atan2(fcn4, fcn5);
        return xzx;
    }

    Vector3D<> toXZX(const Rotation3D<>& R){
        return toXZX(Quaternion<>(R));
    }

}

const int TORSO_IDX = 2;
const int LEFT_SHOULDER_IDX = 3;
const int LEFT_ELBOW_IDX = 4;

const int RIGHT_SHOULDER_IDX = 6;
const int RIGHT_ELBOW_IDX = 7;



void SamplePlugin::readUpdateFromUDP(){
    std::vector<Transform3D<> > transforms(15);
    try
      {

        //boost::array<char, 1> send_buf  = { 0 };
        //socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);

        boost::array<float, 720> recv_buf;
        for(int i=0;i<720;i++){
            recv_buf.data()[i]=0.0;
        }

        udp::endpoint sender_endpoint;
        for(int j=0;j<1;j++){
            size_t len = socket->receive_from( boost::asio::buffer(recv_buf), sender_endpoint);

            for(size_t i=0;i<15;i++){
                Vector3D<> pos(recv_buf.data()[12*i+0],
                               recv_buf.data()[12*i+1],
                               recv_buf.data()[12*i+2]);
                Rotation3D<> rot(
                        recv_buf.data()[12*i+3],
                        recv_buf.data()[12*i+4],
                        recv_buf.data()[12*i+5],
                        recv_buf.data()[12*i+6],
                        recv_buf.data()[12*i+7],
                        recv_buf.data()[12*i+8],
                        recv_buf.data()[12*i+9],
                        recv_buf.data()[12*i+10],
                        recv_buf.data()[12*i+11]);


                transforms[i].R() = rot;
                transforms[i].P() = pos;
            }

            double angle_left = angle(transforms[LEFT_SHOULDER_IDX]*Vector3D<>::x(), transforms[LEFT_ELBOW_IDX]*Vector3D<>::x());
            double angle_right = angle(transforms[RIGHT_SHOULDER_IDX]*Vector3D<>::x(), transforms[RIGHT_ELBOW_IDX]*Vector3D<>::x());

            transforms[RIGHT_SHOULDER_IDX] = inverse(transforms[TORSO_IDX])*transforms[RIGHT_SHOULDER_IDX];
            transforms[LEFT_SHOULDER_IDX ] = inverse(transforms[TORSO_IDX])*transforms[LEFT_SHOULDER_IDX];

            Vector3D<> xzx_left = toXZX(transforms[LEFT_SHOULDER_IDX].R() );
            Vector3D<> xzx_right = toXZX(transforms[RIGHT_SHOULDER_IDX].R() );

            Q q_right(6,0.0);
            q_right(0) = xzx_right(0);
            q_right(1) = xzx_right(1);
            q_right(2) = angle_right;

            Q q_left(6,0.0);
            q_left(0) = xzx_left(0);
            q_left(1) = xzx_left(1);
            q_left(2) = angle_left;

        }
      }
      catch (std::exception& e)
      {
        std::cerr << e.what() << std::endl;
      }




}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
        log().info() << "Button 0\n";
        //resolver = new udp::resolver(io_service);
      //  udp::resolver::query query(udp::v4(),1001);
      //  udp::endpoint receiver_endpoint =  endpoint(ip::udp::v4(), 12345);

        //endpoint = new udp::endpoint(udp::v4(), 1001);
        //socket = new udp::socket(io_service, endpoint);
        _timer->start();

        std::cout << "BUMBUIM " << toXZX(Quaternion<>(1,0,0,0)) << std::endl;
    } else if(obj==_btn1){
        log().info() << "Button 1\n";
        _timer->stop();
    } else if(obj==_spinBox){
        log().info() << "spin value:" << _spinBox->value() << "\n";
    } else if(obj==_timer){
        readUpdateFromUDP();
    }


}

void SamplePlugin::stateChangedListener(const State& state) {

}


Q_EXPORT_PLUGIN(SamplePlugin);
