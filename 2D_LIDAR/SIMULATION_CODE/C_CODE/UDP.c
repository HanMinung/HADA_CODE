#ifndef UDP_C
#define UDP_C

#include "UDP.h"

extern int C;
//extern int RecvArray_rubber[];

void ErrorHandling(char* message);

void Open_UDP_Socket()
{

	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		ErrorHandling("WSAStartup() eror!");

	servSock = socket(PF_INET, SOCK_DGRAM, 0);
	if (servSock == INVALID_SOCKET)
		ErrorHandling("UDP socket creation error");
	int timeout = 10000; //[ms]
	setsockopt(servSock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

	memset(&servAdr, 0, sizeof(servAdr));
	servAdr.sin_family = AF_INET;
	servAdr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAdr.sin_port = htons(Port);

	if (bind(servSock, (SOCKADDR*)&servAdr, sizeof(servAdr)) == SOCKET_ERROR)
		ErrorHandling("BIND() error");

	printf("UDP setting Complete!\n");
		//if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		//	ErrorHandling("WSAStartup() eror!");
		//servSock = socket(PF_INET, SOCK_DGRAM, 0);
		//if (servSock == INVALID_SOCKET)
		//	ErrorHandling("UDP socket creation error");

				//
		//memset(&servAdr, 0, sizeof(servAdr));
		//servAdr.sin_family = AF_INET;
		//servAdr.sin_addr.s_addr = htonl(INADDR_ANY);
		//servAdr.sin_port = htons(Port);
		//if (bind(servSock, (SOCKADDR*)&servAdr, sizeof(servAdr)) == SOCKET_ERROR)
		//	ErrorHandling("BIND() error");

		//printf("UDP setting Complete!\n");
		
}

void wait_UDP(int _recvData[])
{
	printf("Waiting UDP...\n");
	do
	{
		clntAdrSz = sizeof(clntAdr);
		nread_UDP = recvfrom(servSock, _recvData, sizeof(_recvData), 0,
			(SOCKADDR*)&clntAdr, &clntAdrSz);
	} while (_recvData[0] != 1 && _recvData[0] != 0 && _recvData[0] != 2);

	printf("UDP connected!\n");
}

// Preliminary - Rubber cone
void Recvdata_UDP_Ru()//, int _recvData[])
{
		//Read UDP Data
		//if ((SIM_count % NAVI_SAMPLING) == 0)
		//{
		clntAdrSz = sizeof(clntAdr);
		
		nread_UDP = recvfrom(servSock, RecvArray_rubber, sizeof(RecvArray_rubber), 0,
			(SOCKADDR*)&clntAdr, &clntAdrSz);

		printf("nread_UDP : %d\n", nread_UDP);
		printf("rubber cone flag : %d\n", RecvArray_rubber[0]);

		if (RecvArray_rubber[0] == 5)
			BREAK_FLAG = 1;
		else
			BREAK_FLAG = 0;

		buf_camFlag[count] = RecvArray_rubber[0];


		//sendto(servSock, &_sendData, sizeof(_sendData), 0,
		//	(SOCKADDR*)&clntAdr, sizeof(clntAdr));
	//}


}
void Senddata_Cluster_Ru()//, int _recvData[])
{
	//printf("SEND C: %d\n", C);
	sendto(servSock, &C, sizeof(C), 0, (SOCKADDR*)&clntAdr, sizeof(clntAdr));
}

// Close & Error UDP
void ErrorHandling(char* message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}

void Close_UDP()
{
	closesocket(servSock);
	WSACleanup();
}

#endif // !UDP_C
