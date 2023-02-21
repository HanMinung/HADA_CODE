#pragma once
#ifndef ERP42_C
#define ERP42_C

#include "ERP42.h"

extern int RecvArray_rubber[];
void Open_ERPconnect()
{
	if (erpMOD)
	{
		if (RS232_OpenComport(cport_nr, bdrate, mode, 0))
		{
			printf("RS232  : Can not open comport\n");
			return(0);
        }
        else
            printf("ERP42 RS232 Port open!!\n");
	}
}

void Command_ERP42(void) {
    /*======================== SPEED & STEER ==========================*/
    // !! Please CHECK the Speed & Steer MODE values !!

    // SPEED RANGE  : Decimal [0 ~ 200] / HexaDecimal [0x00 ~ 0xc8]
    // Actual Speed : SPEED / 10 [KPH]


    double abs_delta_f;
    if (delta_f < 0)
    {
        abs_delta_f = -delta_f;
    }
    else
    {
        abs_delta_f =  delta_f;
    }
    if (RecvArray_rubber[0] == 0)
    {
        speedcmd = MAXVEL1 - (MAXVEL1 - MINVEL) * abs_delta_f / 28.0;
    }
    else
    {
        speedcmd = MAXVEL2 - (MAXVEL2 - MINVEL) * abs_delta_f / 28.0;
    }


    buf_speedcmd[count] = speedcmd;


    if (SpeedMODE) {
        //writebuffer[eSPEED0] = (speedcmd &0xff00)>>8;            // Decimal 2 Hexa
        writebuffer[eSPEED1] = (speedcmd);// &0xff);               // Decimal 2 Hexa
        writebuffer[eGEAR] = 0x00; //건들지 마시오
    }

    // STEER RANGE : Decimal [-2000 ~2000] / HexaDecimal [ ~ ]
    // Actual Steer : STEER / 71 [Degree]
    if (SteerMODE)
    {
        //CMD_delta_f();

        steercmd = -1 * 71 * delta_f;   // Degree 2 Decimal
        writebuffer[eSTEER0] = (steercmd & 0xff00) >> 8;   // Decimal 2 Hexa
        writebuffer[eSTEER1] = (steercmd & 0xff);   // Decimal 2 Hexa
    }

    /*========================== write data ===========================*/
    write_len = RS232_SendBuf(cport_nr, &writebuffer, sizeof(writebuffer));
    writebuffer[eALIVE] = idx_alive; // loop마다 1씩 증가해서 오류 잡기 alive
    idx_alive++; idx_alive %= 0xff;   // 최대 255까지만 해서 버퍼 오버플로우 안나게
    idx_ndata++;
}

void CMD_delta_f()
{
    if (delta_f > MAX_STEER)
        delta_f = MAX_STEER;
    else if (delta_f < -MAX_STEER)
        delta_f = -MAX_STEER;
}

void CommandERP(int _spdcmd)
{
    CMD_delta_f();

    //if (idx_way == idx_max[0])
    //{
    //    speedcmd = 0;
    //    writebuffer[eBRAKE] = 50;
    //}
    
    //{
        speedcmd = _spdcmd;
        //printf("command OK\n");
    //}

    if (fmod(simul_cnt, 2 == 0)) {
        Command_ERP42();
    }
}

#endif // !ERP42_C
