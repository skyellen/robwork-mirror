/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "PA10Driver.hpp"

#include <rw/common/TimerUtil.hpp>

#include <iostream>
#include <float.h>

namespace
{
    //! This is the ARCNET address of the PC (host) controller
    const short PC = 0xff;

    //! This is the ARCNET address of the PA10 servo driver
    const short PA10ADR = 0xfe;

    const int MECHANICAL_BRAKE = (1<<0); // 1 = ON, 0 = OFF 
    const int SERVO = (1<<1); // 1 = ON, 0 = OFF
    const int TORQUE_SPEED = (1<<2); // 1 = TORQUE, 0 = SPEED
    const int DEADMAN_SWITCH = (1<<3); // 1 = ON, 0 = OFF
        
    const int WAIT = 100; // Just an arbitrary number
}

using namespace rwlibs::devices;
using namespace rw::math;
using namespace rw::common;

PA10Driver::PA10Driver()    
{}

bool PA10Driver::receive_pos()
{
    smsc_packet b;
    Q joint_pos_real(7);
    while(!smsc_recvpacket(0,&b)) TimerUtil::SleepUs(100);
    if (b.type=='C') {
        for (unsigned int i=0;i<joint_pos_real.size();i++) {
            joint_pos_real(i) =
                (double)(
                    (int)((b.data[i*8+2]&0xff)+
                          (b.data[i*8+3]<<8)+
                          (b.data[i*8+4]<<16)+
                          (b.data[i*8+5]<<24)));

            joint_pos_real(i) = 2.44140625e-06 * M_PI * joint_pos_real(i); //To radians
        }
        return true;
    }
    return false;
}

Q PA10Driver::start(bool& b){
    if (smsc_init(0, 0, PC) != 1) {
        b = false;
        return Q(7);
    }

    pa10_send_S();
    pa10_recv_S();

    COut output;
    CIn input;
    Q velocities(Q::ZeroBase(7));
    Q positions(7);
    convertVelocities(velocities, &output); 
    pa10_send_C(&output);
    pa10_recv_C(&input);  
    Q q(7);
    convertPositions(&input, q);

    b = true;
    return q;
}

void PA10Driver::initializeThread()
{
    smsc_threadinit();
}

Q PA10Driver::update(const Q& dq)
{
    COut output;
    CIn input;

    convertVelocities(dq, &output);
    pa10_send_C(&output);
    pa10_recv_C(&input);

    Q positions(7);
    convertPositions(&input, positions);
    
    return positions;

}

void PA10Driver::stop()
{
    pa10_send_E();
    pa10_recv_E();
}

void PA10Driver::convertVelocities(const Q& dq, COut* data)
{
    for(int i=0;i<7;i++){
        uint16 val = (uint16)(dq[i] * 5000);
        data->axis[i].mode = SERVO;
        data->axis[i].torque[0] = 0;
        data->axis[i].torque[0] = 0;
        data->axis[i].speed[0] = val & 0xff;
        data->axis[i].speed[1] = (val >> 8);
    }
}

void PA10Driver::convertPositions(const CIn* data, Q& q)
{
    for(int i=0;i<7;i++){
        long val = data->axis[i].angle[0]+ 
            (data->axis[i].angle[1] << 8) +
            (data->axis[i].angle[2] << 16) +
            (data->axis[i].angle[3] << 24);
        q[i] = (2.0 * M_PI) / (16384.0 * 50.0) * val; 
    }
}

void PA10Driver::pa10_send_S()
{
    smsc_packet p;
    p.ID=PA10ADR;
    p.seqn=0;
    p.type='S';
    p.len=0;
    while(!smsc_sendpacket(0,&p)) std::cout<<".";// TimerUtil::SleepUs(WAIT);
    TimerUtil::SleepUs(5);
}

void PA10Driver::pa10_recv_S()
{
    smsc_packet p;
    while(!smsc_recvpacket(0,&p)) std::cout<<",";// TimerUtil::SleepUs(WAIT);
    RW_ASSERT(p.ID==PA10ADR);
    RW_ASSERT(p.seqn==0);
    RW_ASSERT(p.type=='S');
    RW_ASSERT(p.len==0);
}

void PA10Driver::pa10_send_E()
{
    smsc_packet p;
    p.ID=PA10ADR;
    p.seqn=0;
    p.type='E';
    p.len=0;
    while(!smsc_sendpacket(0,&p)) TimerUtil::SleepUs(WAIT);    
}

void PA10Driver::pa10_recv_E()
{
    smsc_packet p;
    while(!smsc_recvpacket(0,&p)) TimerUtil::SleepUs(WAIT);
    RW_ASSERT(p.ID==PA10ADR);
    RW_ASSERT(p.seqn==0);
    RW_ASSERT(p.type=='E');
    RW_ASSERT(p.len==0);
}

void PA10Driver::pa10_send_C(const COut* data)
{
    smsc_packet p;
    p.ID=PA10ADR;
    p.seqn=0;
    p.type='C';
    p.len=35;
    memcpy(p.data, data, 35);
    while(!smsc_sendpacket(0,&p)) TimerUtil::SleepUs(WAIT);    
}

void PA10Driver::pa10_recv_C(CIn* data)
{
    smsc_packet p;
    while(!smsc_recvpacket(0,&p)) TimerUtil::SleepUs(WAIT);
    RW_ASSERT(p.ID==PA10ADR);
    RW_ASSERT(p.seqn==0);
    RW_ASSERT(p.type=='C');
    RW_ASSERT(p.len==58);
    memcpy(data, p.data, 58);
}
