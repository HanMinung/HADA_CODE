#pragma once
#ifndef _INCLUDE
#define _INCLUDE

#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#define _CRT_SECURE_NO_WARNINGS

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "wsock32.lib")

#include	<winsock2.h>
#include    <WS2tcpip.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <stdint.h>
#include    <math.h>
#include    <string.h>
#include    <memory.h>
#include    <conio.h>
#include    <time.h>
#include    <Windows.h>
#include    <tchar.h>
#include    <stdbool.h>

#include    "Common.h"
#include    "Extern.h"

#endif // !INCLUDE