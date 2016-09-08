/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "DockWelder.hpp"

#include <rw/common/macros.hpp>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>

using namespace rw::math;
using namespace rwhw;
using namespace boost;
using boost::asio::ip::tcp;

namespace {
    //asio::ip::tcp::tcp::socket* _socket = NULL;
	asio::ip::tcp::socket* _socket = NULL;
    const char* PORT = "8000";
    const size_t VGTBLOCKSIZE = 1000;
}

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
        size_t n = asio::write(*_socket, asio::buffer(&buffer, VGTBLOCKSIZE));
        if (n != VGTBLOCKSIZE)
            RW_THROW("Could not send data to server");
    } catch (const std::exception& error) {
        RW_THROW(error.what());
    }
}

void DockWelder::read(char* buffer) {
    try {
        size_t n = asio::read(*_socket, asio::buffer(buffer, VGTBLOCKSIZE));
        if (n != VGTBLOCKSIZE)
            RW_THROW("Could not read data from server");
    } catch (const std::exception& error) {
        RW_THROW(error.what());
    }
}

void DockWelder::openConnection(const std::string& serveraddr) {

    try {
        asio::io_service io_service;
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(tcp::v4(), serveraddr.c_str(), PORT);
        tcp::resolver::iterator iterator = resolver.resolve(query);

        _socket = new tcp::socket(io_service);
        _socket->connect(*iterator);
    } catch  (const std::exception& error) {
        _socket = NULL;
        RW_THROW(error.what());
    }
}

void DockWelder::closeConnection() {
    if (_socket == NULL)
        return;

    boost::system::error_code error;
    try {
        _socket->shutdown(asio::ip::tcp::socket::shutdown_send, error);
    } catch (const std::exception& error) {
        std::cout << "Could not close socket:" << error.what() << std::endl;
    }
    if (error) {
        RW_THROW(error.message());
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
    ostr
        << "VGT Status: " << "\n"
        << "Time: " << s.t << "\n"
        << "isServoOn: " << s.isServoOn << "\n"
        << "isLoaded: " << s.isLoaded << "\n"
        << "isMoving: " << s.isMoving << "\n"
        << "isPaused: " << s.isPaused << "\n"
        << "isError: " << s.isError << "\n"
        << "isLimit: " << s.isLimit << "\n"
        << "q: " << s.q << "\n"

        << "frezed: " << s.fj00 << " " << s.fj01 << " " << s.fj02
        << " " << s.fj10 << " " << s.fj11 << " " << s.fj12 << "\n"

        << "Lower Limits: " << s.lj00 << "\t" << s.lj01 << "\t" << s.lj02 << "\t"
        << s.lj10 << "\t" << s.lj11 << "\t" << s.lj12 << "\n"

        << "Upper Limits: " << s.hj00 << "\t" << s.hj01 << "\t" << s.hj02 << "\t"
        << s.hj10 << "\t" << s.hj11 << "\t" << s.hj12

        << std::endl;
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
