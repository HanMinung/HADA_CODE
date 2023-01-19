#ifndef ERP42_H
#define ERP42_H

#include "Include.h"

// ERP 42 PROTOCOL LENGTH
#define nWRITE (uint8_t)(14)
#define nREAD  (uint8_t)(18)

// ERP 42 PROTOCOL DEFAULT SETTING
#define S (uint8_t)(0x53)
#define T (uint8_t)(0x54)
#define X (uint8_t)(0x58)

#define AorM_Manual   (uint8_t)(0x00)
#define AorM_AutoMode (uint8_t)(0x01)

#define ESTOP_ON  (uint8_t)(0x01)
#define ESTOP_OFF (uint8_t)(0x00)

#define GEAR_FOWARD   (uint8_t)(0x00)
#define GEAR_NEUTRAL  (uint8_t)(0x01)
#define GEAR_BACKWORD (uint8_t)(0x02)

#define ETX0 (uint8_t)(0x0D)
#define ETX1 (uint8_t)(0x0A)

#define SPEED0 (uint8_t)(0x00) // range      0 ~ 200
#define SPEED1 (uint8_t)(0x00) // range      0 ~ 200
#define STEER0 (uint8_t)(0x00) // range  -2000 ~ 2000
#define STEER1 (uint8_t)(0x00) // range  -2000 ~ 2000
#define BRAKE  (uint8_t)(0x01) // range      1 ~ 200
#define ALIVE  (uint8_t)(0x00) // range       0 ~ 255

//extern void PlatformControl(void);
enum ERP_Protocol {
	eS = 0,
	eT, eX, eAUTO, eESTOP, eGEAR, eSPEED0, eSPEED1, eSTEER0, eSTEER1, eBRAKE, eALIVE, eETX0, eETX1
};

int          cport_nr = 5, bdrate = 115200;  /*com3 => cport_nr = 2*/
char mode[] = { '8','N','1',0 };
char writebuffer[nWRITE] = { S,T,X,AorM_AutoMode,ESTOP_OFF,GEAR_FOWARD,SPEED0,SPEED1,STEER0,STEER1,BRAKE,ALIVE,ETX0,ETX1 };
int          speedcmd = 0;
int          steercmd = 0;
int          write_len = 0;


double buf_speedcmd[BUF_SIZE] = { 0.0, };

unsigned int idx_alive = 0x00;
unsigned int idx_ndata = 0;


#endif // !ERP42_H