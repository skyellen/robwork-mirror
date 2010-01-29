/**
 * FILE: arcnetload/smsclib.c
 * PURPOSE: linux functions to talk to the ARCnet board
 * MODIFIED: $Date: 2007-05-22 10:39:19 $
 */
#include "smsclib.h"

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>

#include <sys/io.h>
#include <sys/time.h>

#ifndef CYGWIN
#define __always_inline inline
#include <linux/delay.h>
#include <asm/types.h>

typedef uint8 u8;
typedef uint16 u16;
typedef uint32 u32;

#include <asm/system.h>
//
#define mb() alternative("lock; addl $0,0(%%esp)", "mfence", X86_FEATURE_XMM2)
#else
#define mb()
#define iopl(A)
#endif

/**
 * array of io addresses (base addresses of arcnet ic)
 */
#define NCHIPS 1
static unsigned int io_addr[NCHIPS] = { 0x300 };
static uint8 nodeids[NCHIPS] = { 0 };

#define PEEK8(N,REG)      inb_p(io_addr[N]+REG)
//#define PEEK16(N,REG)     inw_p(io_addr[N]+REG)

#define PEEK16(N,REG)     (PEEK8(N,REG)|(PEEK8(N,REG)<<8))

#define POKE8(N,REG,VAL)  outb_p(VAL,io_addr[N]+REG)
//#define POKE16(N,REG,VAL) outw(VAL,io_addr[N]+REG)
#define POKE16(N,REG,VAL) POKE8(N,REG,((VAL)&0xff));POKE8(N,REG,(((VAL)>>8)&0xff))

/* Define relative adresser til registrene */
#define GET_STATUS(N)       PEEK8(N,0)
#define GET_INTMASK(N)      PEEK8(N,0)
#define GET_DIAG(N)         PEEK8(N,1)
#define GET_ADDR_L(N)       PEEK8(N,3)
#define GET_DATA(N)         PEEK16(N,4)
#define GET_DATA8(N)        PEEK8(N,4)
#define GET_REG7(N)         PEEK8(N,7)

#define SET_INTMASK(N,V)    POKE8(N,0,V)
#define SET_COMMAND(N,V)    POKE8(N,1,V)
#define SET_ADDR_H(N,V)     POKE8(N,2,V)
#define SET_ADDR_L(N,V)     POKE8(N,3,V)
#define SET_DATA(N,V)       POKE16(N,4,V)
#define SET_DATA8(N,V)      POKE8(N,4,V)
#define SET_SUBADDR(N,V)    POKE8(N,5,V)
#define SET_CONFIG(N,V)     POKE8(N,6,V)
#define SET_REG7(N,V)       POKE8(N,7,V)

#define GET_NEXTID(N)       ( SET_SUBADDR(N,3), GET_REG7(N) )

#define SET_TENTATIVEID(N,V)  SET_SUBADDR(N,0); SET_REG7(N,V)
#define SET_NODEID(N,V)       SET_SUBADDR(N,1); SET_REG7(N,V)
#define SET_SETUP1(N,V)       SET_SUBADDR(N,2); SET_REG7(N,V)
#define SET_SETUP2(N,V)       SET_SUBADDR(N,4); SET_REG7(N,V)
#define SET_BUSCONTROL(N,V)   SET_SUBADDR(N,5); SET_REG7(N,V)
#define SET_DMACOUNT(N,V)     SET_SUBADDR(N,6); SET_REG7(N,V)

/* Nedenstående OR'es med ADDR_PTR_H for at vælge addresseringsmode */
#define READ        0x80 //NO DMA
#define WRITE       0x00 //NO DMA
#define WRITEAUTO   0x40 //autoincrement on
#define READAUTO    0xc0 //autoincrement on

/**
 * Detects the arcnet PCI IO address
 */
int smsc_pci_detect()
{
  FILE *f;
  unsigned int ioaddr=0;
  unsigned int vendor=0;
  f=fopen("/proc/bus/pci/devices","r");
  if(f==NULL) fprintf(stderr,"couldn't open /proc/bus/pci/devices");
  while( (!feof(f)) &&
	 ((vendor!=0x1571a202) &&
	  (vendor!=0x1571a205)) ) {
    if(fscanf(f,"%*x %x %*x %*x %*x %x %*[^\n]\n",&vendor,&ioaddr)!=2)
      fprintf(stderr,"unknown format in /proc/bus/pci/devices");
    else {
      printf("vendorid: 0x%08x, ioaddr: 0x%04x\n",vendor,ioaddr);
    }
  }
  fclose(f);
  return ioaddr&0xfffe;
}

/*

*---> Initialize the COM 20020 IC

                clr.l   d1                              Clear all of d1
                move.w  M$DevCon(a1),d1                 get DevCon offset in low part of d1

                move.b  PD_20020Setup(a1,d1.w),d0       Get setupbyte from descriptor
                move.b  #%00011010,CONF(a3)             SETUP/ID register will be used as setup register.
                move.b  d0,SETUP(a3)                    write setup byte to COM20020

                add.l   a1,d1                           add Descriptor module address to offset
                move.l  d1,V_DevCon(a2)                 Save global address of DevCon for future use

                move.b  DIPSW(a3),d0                    Read the dipswitch (NODE ID)
                eor.b   #$ff,d0                         complement DIP switch reading

                move.b  #%00011001,CONF(a3)             SETUP/ID register will be used as NODE ID register.
                move.b  d0,ID(a3)                       write NODE ID to COM20020

                bsr     Sleep100ticks                   We want to sleep for 100 timeslices

                move.b  DIAGSTAT(a3),d0                 Read diagnostics register
                btst    #5,d0                           Is there receive activity?
                beq     Err_netdown                     No, exit
                btst    #4,d0                           Is token seen?
                beq     Err_unreach                     No, exit
                btst    #6,d0                           ID conflict ?
                bne     Err_addrinuse                   Yes, exit

*----> Enable transmitter

                move.b  #%00111000,CONF(a3)             TXEN=1 (join the network)
                bsr     Sleep100ticks                   We want to sleep for 100 timeslices

*----> Enable Receiver

                move.b  #%00000101,COMMAND(a3)          Handle only short packets (define configuration)
                move.b  #%00000100,COMMAND(a3)          Enable receive to page 0
*/


void smsc_threadinit() {
  if(iopl(3)!=0){
    fprintf(stderr,"smsclib requires thread to have I/O port privileges, run as root\n");
  }
}

/**
 * initializes the arcnet controller
 * @arg n is used for identifying the the arcnet chip in later calls (counting from 0)
 * @arg ioaddr is the io address of the arcnet chip. If 0 the default value is used.
 * @arg thisnodeid is the node id of this node
 */
int smsc_init(uint8 n, uint32 ioaddr, uint8 thisnodeid)
{
  uint16 val;
  int i;
  if(!(n<NCHIPS)) fprintf(stderr,"smsclib is not compiled for that many arcnet ICs\n");

  if(iopl(3)!=0){
    fprintf(stderr,"smsclib requires I/O port privileges, run as root\n");
    return 0;
  }

  if(ioaddr==0) ioaddr=smsc_pci_detect();
  if(ioaddr!=0) io_addr[n]=ioaddr;

  printf("ARCnet io-address: 0x%04x\n",io_addr[n]);

  //iopl(3);
  i=PEEK8(n,8);

  if(i!=0) thisnodeid=i;

  nodeids[n]=thisnodeid;

  SET_ADDR_L(n,0);          /* Autodetect bus interface */
  GET_ADDR_L(n);

  //  try this if we still get "lockups"
  SET_COMMAND(n,0x01); /* Disable transmitter */
  SET_COMMAND(n,0x02); /* Disable receiver */

  SET_CONFIG(n,0x80);       /* Reset */
  SET_CONFIG(n,0x00);       /* End reset */
  SET_NODEID(n,thisnodeid); /* This Node ID is written to wake up the core (workaround) */

  SET_SETUP1(n,0x80);       /* PULSE1 push/pull driver, and >2.5 mbps operation, slowarb=1 */  
  SET_SETUP2(n,0x1e);       /* Normal Bus read-speed, 40 MHz clock, EF=1, NOSYNC and 26.25 ms reconf */ 

  SET_COMMAND(n,0x18);      /* Magical Restart core (to enable 10Mbps) */
  SET_CONFIG(n, 0x00);

  //SET_ADDR_L(n,0x00);       /* Make sure SWAP bit (A0) is cleared */
  SET_COMMAND(n,0x05);      /* Handle only short packages */

  //SET_BUSCONTROL(n,0x80);   /* Enabling 16 bit acccess for Data register */

  SET_BUSCONTROL(n,0x00);   /* Enabling 8 bit acccess for Data register */
  SET_COMMAND(n,0x1E);      /* Reset flags */

  SET_NODEID(n,thisnodeid); /* This Node ID */
  SET_CONFIG(n,0x3c);       /* Tjaa gad vide hvad det er vi har valgt? */ 

  /* Test microcore og nodeid */
  SET_ADDR_H(n,READAUTO);   /* Read prepare, AUTO INCR, NO DMA, ADR10-8 = 0 */
  SET_ADDR_L(n,0x00);       /* Read to ADR=00 upcoming */
  val=GET_DATA(n);

  if(val!=((thisnodeid<<8)|0xd1)) {
    printf("The core of COM20020 is not working properly! value=0x%04x (should have been 0x%04x)\n",val,(thisnodeid<<8)|0x1d);
    return 0;
  }
  printf("COM20020 detected at address 0x%x and initialized with nodeid %d\n",io_addr[n],thisnodeid);


  /**
   * memory test
   */
  SET_ADDR_H(n,WRITEAUTO+(512>>8));
  SET_ADDR_L(n,0);
  mb();
  for(i=0;i<0xff;i++) {
    POKE8(n,4,i);
  }
  mb();
  SET_ADDR_H(n,READAUTO+(512>>8));
  SET_ADDR_L(n,0x00);
  mb();
  for(i=0;i<0xff;i++) {
    if(PEEK8(n,4)!=i) printf("fejl: %d\n",i);
  }

  SET_COMMAND(n,0x04);      /* Enable receiver to page 0 offset 0, clear recv_inhibit flag */
  printf("smsclib init done!\n");
  return 1;
}

/**
 * send the packet p
 */
int smsc_sendpacket(uint8 n, smsc_packet *p)
{
  uint8 count;

  //smsc_updatestats(n);
  //smsc_printstats(n);
  fflush(stdout);
  if(!(GET_STATUS(n)&0x01)) {
      return 0; // Transmitter not ready yet
  }
  SET_ADDR_H(n,WRITEAUTO+(512>>8));   // prepare to write to page 1 with autoincrement
  SET_ADDR_L(n,0);                    // offset 0
  // Receivers address (DID). SID is written automatically
  SET_DATA8(n,p->ID);
  SET_DATA8(n,p->ID);
  count = 256 - (p->len + 2);         // COUNT is 256 minus number of bytes to be transmitted
  SET_DATA8(n,count);
  SET_ADDR_L(n,count);
  SET_DATA8(n,p->seqn);
  SET_DATA8(n,p->type);
  outsb(io_addr[n]+4, p->data, p->len);
  SET_COMMAND(n,11);                  // Transmit from page 1 offset 0

  return 1;                           /* Success */
}

/**
 * returns 1 when TA and TMA bit is set (last send has been
 * acknowledged)
 */

int smsc_acknowledged(uint8 n)
{
  return ((GET_STATUS(n)&0x03)!=0);
}

int smsc_recvpacket(uint8 n, smsc_packet *p)
{
  uint8 count;
  uint16 val;

  //smsc_updatestats(n);
  //smsc_printstats(n);

  if(!(GET_STATUS(n)&0x80)) return 0; /* return false if no packet has been received. */

  SET_ADDR_H(n,READAUTO);             /* Prepare to read from page 0 with autoincrement */
  SET_ADDR_L(n,0);                    /* offset 0 */

  p->ID=GET_DATA8(n)&0xff;            /* SID */
  val=GET_DATA8(n);                   /* DID */
  count=GET_DATA8(n);                 /* Count */
  p->len = 256 - 2 - count; 

  SET_ADDR_L(n,count&0xff);
  p->seqn=GET_DATA8(n);               /* sequential no. */
  p->type=GET_DATA8(n);               /* type */
  insb(io_addr[n]+4, p->data, p->len);

  //if(p->type==0x02) printf(">%d %s",p->ID,p->data);     /* handle stdio */

  SET_COMMAND(n,0x04); /* enable receiver to page 0 offset 0, clear recv_inhibit flag */
  return 1;
}

struct smsc_stat_struct {
  int send;
  int read;

  int recon;
  int por;

  int newxid;
  int tentid[256];
  int excnak;
  int token;
  int rcvact;
  int dupid;
  int myrecon;

  int nextid;
} smsc_stat[NCHIPS], smsc_copystat[NCHIPS];


double timegetclockinternal()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  return tv.tv_sec+(1.0*tv.tv_usec)/1000000;
}

static int ntentid=0;
static int ntentid_temp=0;

void nexttentativeid(uint8 n,int found)
{
  static double last=0;
  double now=timegetclockinternal();
  if((now-last<.1)&&!found) return;

  last=now;

  // nextid will not show up in tentative id, so count it anyway
  if(ntentid==smsc_stat[n].nextid) {
    smsc_stat[n].tentid[ntentid]++;
  }

  // if we have not seen the node in this period
  if(smsc_stat[n].tentid[ntentid]==ntentid_temp) {
    // if the node has been seen mark it as gone by negating
    if(ntentid_temp>0) smsc_stat[n].tentid[ntentid]*=-1;
  }

  ntentid++;
  if(ntentid>3) ntentid=1;

  // remember how many times we have seen this node before trying this time.
  ntentid_temp=smsc_stat[n].tentid[ntentid];

  SET_TENTATIVEID(n,ntentid);
}

void smsc_updatestats(uint8 n)
{
  int s,d;

  mb();
  s=GET_STATUS(n)&~(32+64);
  d=GET_DIAG(n)&~1;

  //    if(s&1)   smsc_stat[n].ta++;
  //    if(s&2)   smsc_stat[n].tma++;
  
  // RECON og MYRECON
  if(s&4) {
    smsc_stat[n].recon++;
    if(d&128) smsc_stat[n].myrecon++;
    SET_COMMAND(n,0x1E); // Reset flags
  }

  // if(s&8)   smsc_stat[n].test++;
  // if(s&16)  smsc_stat[n].por++;
  // if(s&128) smsc_stat[n].ri++;

  // NEW NXTID
  if(d&2) {
    smsc_stat[n].newxid++;
    smsc_stat[n].tentid[smsc_stat[n].nextid]*=-1; // mark as gone
    
    smsc_stat[n].nextid=GET_NEXTID(n);
    smsc_stat[n].tentid[smsc_stat[n].nextid]=0; // mark as new
  }

  if(d&4) { smsc_stat[n].tentid[ntentid]++; nexttentativeid(n,1); }


  nexttentativeid(n,0);


  // EXCNAK
  if(d&8) {
    smsc_stat[n].excnak++;
    SET_COMMAND(n,0x1E); // Reset flags
  }

  /*
  if(d&16)  smsc_stat[n].token++;
  if(d&32)  smsc_stat[n].rcvact++;
  if(d&64)  smsc_stat[n].dupid++;
  */

}

int stamp_printf(const char* _format, ...)
{
  va_list _ap;
  int l;
  printf("%10.2f ",timegetclockinternal());
  va_start(_ap, _format);
  l=vprintf(_format, _ap);
  va_end(_ap);
  return l;
}
/*
void smsc_printstats(uint8 n)
{
  int i,recon, myrecon, excnak, newxid, read, send;

  recon=smsc_stat[n].recon-smsc_copystat[n].recon;
  myrecon=smsc_stat[n].myrecon-smsc_copystat[n].myrecon;
  excnak=smsc_stat[n].excnak-smsc_copystat[n].excnak;
  newxid=smsc_stat[n].newxid-smsc_copystat[n].newxid;
  read=smsc_stat[n].read-smsc_copystat[n].read;
  send=smsc_stat[n].send-smsc_copystat[n].send;

  if(recon)   stamp_printf("# reconfigure: %d\n",smsc_stat[n].recon);
  if(myrecon) stamp_printf("# myrecons: %d\n",smsc_stat[n].myrecon);
  if(excnak)  stamp_printf("# ExcNak: %d\n",smsc_stat[n].excnak);
  if(newxid)  stamp_printf("# NewNextIDs: %d, NextID: %d\n",smsc_stat[n].newxid,smsc_stat[n].nextid);
  if(read)    stamp_printf("# Reads: %d\n",smsc_stat[n].read);
  if(send)    stamp_printf("# Sends: %d\n",smsc_stat[n].send);

  for(i=1;i<256;i++) {
    int t=smsc_stat[n].tentid[i]-smsc_copystat[n].tentid[i];
    if(t) {
      if(smsc_stat[n].tentid[i]<0) {
	int j;
	stamp_printf("# Node %d has left the network (online:",i);
	for(j=1;j<256;j++) if(smsc_stat[n].tentid[j]>0) printf(" %d",j);
	printf(" %d)\n",nodeids[n]);
	smsc_stat[n].tentid[i]=0; // start from scratch
      }
      if(smsc_stat[n].tentid[i]==1) {
	int j;
	stamp_printf("# Node %d has joined the network (online:",i);
	for(j=1;j<256;j++) if(smsc_stat[n].tentid[j]>0) printf(" %d",j);
	printf(" %d)\n",nodeids[n]);
      }
    }
  }

  smsc_copystat[n]=smsc_stat[n];
}

void smsc_printdiffstats(uint8 n)
{
  static int i;
  int recon, myrecon, excnak, newxid, read, send;

  recon=smsc_stat[n].recon-smsc_copystat[n].recon;
  myrecon=smsc_stat[n].myrecon-smsc_copystat[n].myrecon;
  excnak=smsc_stat[n].excnak-smsc_copystat[n].excnak;
  newxid=smsc_stat[n].newxid-smsc_copystat[n].newxid;
  read=smsc_stat[n].read-smsc_copystat[n].read;
  send=smsc_stat[n].send-smsc_copystat[n].send;

  if(recon+myrecon+excnak+newxid+read+send>0) {

    if(i++%10==0) printf("  Reconfig   MyReconf     ExcNak  NewNextID      Reads      Sends\n");
    printf("%10d %10d %10d %10d %10d %10d\n",
	   smsc_stat[n].recon-smsc_copystat[n].recon,
	   smsc_stat[n].myrecon-smsc_copystat[n].myrecon,
	   smsc_stat[n].excnak-smsc_copystat[n].excnak,
	   smsc_stat[n].newxid-smsc_copystat[n].newxid,
	   smsc_stat[n].read-smsc_copystat[n].read,
	   smsc_stat[n].send-smsc_copystat[n].send);
    smsc_copystat[n]=smsc_stat[n];
  }

  smsc_updatestats(n);
}
*/
void smsc_printstats(uint8 n)
{
  static int oldstat=65536;
  static int olddiag=65536;
  static int oldnxid=65536;
  int s,d,nxid;
  mb();
  s=GET_STATUS(n)&~(32+64);
  d=GET_DIAG(n)&~1;
  nxid=GET_NEXTID(n);

  if((s!=oldstat)||(d!=olddiag)||(nxid!=oldnxid)) {
    if(s&1)   printf("TA    :"); else printf("      :");
    if(s&2)   printf("TMA   :"); else printf("      :");
    if(s&4)   printf("RECON :"); else printf("      :");
    if(s&8)   printf("TEST  :"); else printf("      :");
    if(s&16)  printf("POR   :"); else printf("      :");
    if(s&128) printf("RI    :"); else printf("      :");

    if(d&2)   printf("NewXID:"); else printf("      :");
    if(d&4)   printf("TentID:"); else printf("      :");
    if(d&8)   printf("ExcNAK:"); else printf("      :");
    if(d&16)  printf("Token :"); else printf("      :");
    if(d&32)  printf("RcvAct:"); else printf("      :");
    if(d&64)  printf("DupID :"); else printf("      :");
    if(d&128) printf("MyReco:"); else printf("      :");

    printf("NxtID %d\n",nxid);

    oldstat=s;
    olddiag=d&(~11);
    oldnxid=nxid;
  }
}

