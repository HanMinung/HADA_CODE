#pragma once
#ifndef UDP_H
#define UDP_H

#include "Include.h"

typedef unsigned short PORT;
int RecvArray_rubber[1] = { 0 }; // rubber cone fail 유무 0 -성공 / 1-왼쪽 / 2-오른쪽 못잡음 
int nread_UDP = 0; // n_ : length 
int clntAdrSz = 0; // _sz : sizeof
int BREAK_FLAG = 0;

int buf_camFlag[BUF_SIZE] = { 0 };

// socket variable
WSADATA     wsaData;
SOCKET      servSock;
SOCKADDR_IN servAdr, clntAdr;

PORT Port = 1101;

#endif // !UDP_H
