#include "DockWelder.hpp"


#include <rw/common/macros.hpp>
#include <boost/asio.hpp>

using namespace rw::math;
using namespace rwlibs::devices;

using boost::asio::ip::tcp;

namespace {
    //boost::asio::ip::tcp::tcp::socket* _socket = NULL;
	boost::asio::ip::tcp::socket* _socket = NULL;
    const char* PORT = "8000";
    const size_t VGTBLOCKSIZE = 1000;
};

DockWelder::DockWelder()
{
    
}


DockWelder::~DockWelder() {

}


void DockWelder::write(const char* buf) {
    char buffer[VGTBLOCKSIZE];
    memset(buffer, 0, VGTBLOCKSIZE);

    size_t i = 0;
    while (buf[i] != 0) {
        buffer[i] = buf[i];
        i++;
    }
    try {
        size_t n = boost::asio::write(*_socket, boost::asio::buffer(&buffer, VGTBLOCKSIZE));
        if (n != VGTBLOCKSIZE)
            RW_THROW("Could not send data to server");
    } catch (boost::asio::error& error) {
        RW_THROW(error.what());
    }
}

void DockWelder::read(char* buffer) {
    try {
        size_t n = boost::asio::read(*_socket, boost::asio::buffer(buffer, VGTBLOCKSIZE));
        if (n != VGTBLOCKSIZE)
            RW_THROW("Could not read data from server");
    } catch (boost::asio::error& error) {
        RW_THROW(error.what());
    }
}

void DockWelder::openConnection(const std::string& serveraddr) {

    try {
        boost::asio::io_service io_service;
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), serveraddr.c_str(), PORT);
        tcp::resolver::iterator iterator = resolver.resolve(query);
        
        _socket = new tcp::socket(io_service);     
        _socket->connect(*iterator);
    } catch (boost::asio::error& error) {
        _socket = NULL;
        RW_THROW(error.what());
    }
}

void DockWelder::closeConnection() {
    if (_socket == NULL)
        return;

    boost::asio::error error;
    try {
        _socket->shutdown(boost::asio::ip::tcp::socket::shutdown_send,boost::asio::assign_error(error));
    } catch (boost::asio::system_exception exp) {
        std::cout<<"Could not close socket:"<<exp.what()<<std::endl;
    }
    if (error) {
        RW_THROW(error.what());
    }
}


void DockWelder::servoOn() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"SERVOON");
    write(buf);
}

void DockWelder::servoOff() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"SERVOOFF");
    write(buf);
}

void DockWelder::setVelocity(double velocity) {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    if (velocity < 0 || velocity > 100)
        RW_THROW("Velocity must be between 0 and 100");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"SETVEL %9g",velocity);
    write(buf);
}

void DockWelder::move(const rw::math::Q& q) {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    if (q.size() != 6) {
        RW_THROW("Configuration must have exactly 6 values");
    }
    char buf[VGTBLOCKSIZE];
    sprintf(buf,"AJA %9g %9g %9g %9g %9g %9g %9g",q(0),q(1),q(2),q(3),q(4),q(5),0.0);
    write(buf);
}


rw::math::Q DockWelder::getQ() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    const Status s = status();
    return s.q;
    
}


void DockWelder::startMotion() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"START");
    write(buf);

}

void DockWelder::pauseMotion() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"PAUSE");
    write(buf);
}

void DockWelder::stopMotion() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    sprintf(buf,"STOP");
    write(buf);
}


void DockWelder::printStatus(std::ostream& ostr) {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    Status s = status();
    ostr<<"VGT Status: "<<std::endl;
    ostr<<"Time: "<<s.t<<std::endl;
    ostr<<"isServoOn: "<<s.isServoOn<<std::endl;
    ostr<<"isLoaded: "<<s.isLoaded<<std::endl;
    ostr<<"isMoving: "<<s.isMoving<<std::endl;
    ostr<<"isPaused: "<<s.isPaused<<std::endl;
    ostr<<"isError: "<<s.isError<<std::endl;
    ostr<<"isLimit: "<<s.isLimit<<std::endl;
    ostr<<"q = "<<s.q<<std::endl;
    ostr<<"frezed = "<<s.fj00<<" "<<s.fj01<<" "<<s.fj02<<" "<<s.fj10<<" "<<s.fj11<<" "<<s.fj12<<std::endl;
    ostr<<"Lower Limits"<<s.lj00<<"\t"<<s.lj01<<"\t"<<s.lj02<<"\t"<<s.lj10<<"\t"<<s.lj11<<"\t"<<s.lj12<<std::endl;
    ostr<<"Upper Limits"<<s.hj00<<"\t"<<s.hj01<<"\t"<<s.hj02<<"\t"<<s.hj10<<"\t"<<s.hj11<<"\t"<<s.hj12<<std::endl;
}

DockWelder::Status DockWelder::status() {
    if (_socket == NULL)
        RW_THROW("Not Connected to Device");

    char buf[VGTBLOCKSIZE];
    double dummy;
    sprintf(buf,"GETSTAT");
    write(buf);
    read(buf);
    buf[VGTBLOCKSIZE-1] = 0;
    Status s;
    if(sscanf(buf,
              "%ld\n"
              "%d %d %d %d %d %d\n"
              "%lg %lg %lg %lg %lg %lg %lg\n"
              "%d %d %d %d %d %d %d\n"
              "%lg %lg %lg %lg %lg\n"
              "%lg %lg %lg %lg %lg %lg\n"
              "%d %d %d %d %d %d\n"
              "%d %d %d %d %d %d\n"
              "%d %d\n"
              "%d %80c",
              &(s.t),
              &(s.isServoOn),
              &(s.isLoaded),
              &(s.isMoving),
              &(s.isPaused),
              &(s.isError),
              &(s.isLimit),
              &(s.q(0)), &(s.q(1)), &(s.q(2)),
              &(s.q(3)), &(s.q(4)), &(s.q(5)),
              &(dummy), //this joint does not exists anymore
              &(s.fj00), &(s.fj01), &(s.fj02),
              &(s.fj10), &(s.fj11), &(s.fj12), &(s.fj20),
              &dummy,&dummy,
              &dummy,&dummy,&dummy,
              &dummy, &dummy, &dummy, //position for old kinematic
              &dummy, &dummy, &(dummy), //rotation for old kinematic
              &(s.lj00),&(s.hj00),
              &(s.lj01),&(s.hj01),
              &(s.lj02),&(s.hj02),
              &(s.lj10),&(s.hj10),
              &(s.lj11),&(s.hj11),
              &(s.lj12),&(s.hj12),
              &(s.lj20),&(s.hj20),
              &(s.nError),(s.errbuf))!=48)
        RW_THROW("Error in returned status");
    return s;

}
