//
// Created by XxiaoLove8 on 2025/12/1.
//



#ifdef _WIN32

#include "SerialWin.h"
#include <stdio.h>

int Serial_Open(SerialPort *port, const char *comName, DWORD baud)
{
    char fullName[32];
    // Windows 下 COM10 以上要用这种 \\.\ 前缀
    snprintf(fullName, sizeof(fullName), "\\\\.\\%s", comName);

    HANDLE h = CreateFileA(
        fullName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );
    if (h == INVALID_HANDLE_VALUE) {
        printf("Failed to open %s\n", fullName);
        return 0;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return 0;
    }

    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return 0;
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(h, &timeouts);

    port->h = h;
    return 1;
}

int Serial_SendByte(SerialPort *port, char c)
{
    DWORD written = 0;
    if (!WriteFile(port->h, &c, 1, &written, NULL)) {
        return 0;
    }
    return (written == 1);
}

void Serial_Close(SerialPort *port)
{
    if (port->h && port->h != INVALID_HANDLE_VALUE) {
        CloseHandle(port->h);
        port->h = INVALID_HANDLE_VALUE;
    }
}

#endif // _WIN32