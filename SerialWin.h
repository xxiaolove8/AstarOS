//
// Created by XxiaoLove8 on 2025/12/1.
//

#ifndef ASTAROS_SERIALWIN_H
#define ASTAROS_SERIALWIN_H

#pragma once

#ifdef _WIN32

#include <windows.h>

// 简单串口句柄封装
typedef struct {
    HANDLE h;
} SerialPort;

// 打开串口，例如 "COM3"
int Serial_Open(SerialPort *port, const char *comName, DWORD baud);

// 发送一个字节
int Serial_SendByte(SerialPort *port, char c);

// 关闭串口
void Serial_Close(SerialPort *port);

int Serial_RecvByte(SerialPort *port, char *outChar, DWORD timeout_ms);//收到一个字节

#endif // _WIN32

#endif //ASTAROS_SERIALWIN_H