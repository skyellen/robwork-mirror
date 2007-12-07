/**
 * FILE: smsc.h
 * PURPOSE: functions to talk to the ARCnet board
 */

#ifndef SMSCLIB_H
#define SMSCLIB_H

typedef unsigned char  uint8;
typedef unsigned short uint16;
typedef unsigned int   uint32;

/* short packages */
#define PACKETSIZE  256

/**
 * \brief An ARCNet (small) packet
 */
typedef struct packet{	
  //! The packet ID
  uint8 ID;

  //! The packet-data length (number of bytes stored in the data field) 
  uint8 len;

  //! Sequence counter
  uint8 seqn;

  //! The packet type
  uint8 type;

  //! The packet data, the packet can cary up to 252 bytes of data
  uint8 data[PACKETSIZE-4];
} smsc_packet;

int smsc_init(uint8 n, uint32 ioaddr, uint8 nodeid);
int smsc_sendpacket(uint8 n, smsc_packet *p);
int smsc_recvpacket(uint8 n, smsc_packet *p);
void smsc_updatestats(uint8 n);
void smsc_printstats(uint8 n);

void smsc_threadinit();

#endif // SMSCLIB_H
