
/**
 * @file uart.c
 * @author Raphael Thurnherr (raphael.thurnherr@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-21
 * 
 * @copyright Copyright (c) 2020
 * 
 */

//#define SERIAL_DEBUG

#include <Windows.h>
#include <stdio.h>
#include <string.h>

#include "serial.h"

HANDLE hComm;  // Handle to the Serial port

void Exit1(){
    CloseHandle(hComm);                         //Closing the Serial Port
}
void Exit2(){
    system("pause");
}


int serial_init(char * port, int baudRate){
    char portName[25];

    BOOL   Status; // Status
    DCB dcbSerialParams = { 0 };                // Initializing DCB structure
    COMMTIMEOUTS timeouts = { 0 };              //Initializing timeouts structure

    wchar_t pszPortName[10] = { 0 };            //com port id
    wchar_t PortNo[20] = { 0 };                 //contain friendly name

    sprintf(portName, "\\\\.\\%s", port);
    hComm = CreateFile(portName,                            
                       GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
                       0,                                 // No Sharing, ports cant be shared
                       NULL,                              // No Security
                       OPEN_EXISTING,                     // Open existing port only
                       0,                                 // Non Overlapped I/O
                       NULL);                             // Null for Comm Devices
    if (hComm == INVALID_HANDLE_VALUE)
    {
        #ifdef SERIAL_DEBUG
         printf_s("\n Port can't be opened\n\n");
        #endif
        Exit2();
    }


    //Setting the Parameters for the SerialPort
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    Status = GetCommState(hComm, &dcbSerialParams); //retreives  the current settings
    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nError to Get the Com state\n\n");
        #endif
        Exit1();
    }
    
    // dcbSerialParams.BaudRate = CBR_9600;      //BaudRate = 9600
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;             //ByteSize = 8
    dcbSerialParams.Parity = NOPARITY;          //Parity = None
    dcbSerialParams.StopBits = ONESTOPBIT;    //StopBits = 1

    Status = SetCommState(hComm, &dcbSerialParams);
    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nError to Setting DCB Structure\n\n");
        #endif
        Exit1();

    }

    //Setting Timeouts
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hComm, &timeouts) == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nError to Setting Time outs");
        #endif
        Exit1();
    }
}


int serial_write(char * ptrDataToWrite, int count){
    BOOL   Status; // Status
    DWORD BytesWritten = 0;          // No of bytes written to the port

    //Writing data to Serial Port
    Status = WriteFile(hComm,// Handle to the Serialport
                       ptrDataToWrite,            // Data to be written to the port
                       count,   // No of bytes to write into the port
                       &BytesWritten,  // No of bytes written to the port
                       NULL);
    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nFail to Written");
        #endif
        Exit1();
    }
    //print numbers of byte written to the serial port
    
    #ifdef SERIAL_DEBUG
    printf_s("\nNumber of bytes written to the serial port = %d\n\n", BytesWritten);
    #endif
    return 0;
}

int serial_write_string(char * ptrDataToWrite){
    BOOL   Status; // Status
    DWORD BytesWritten = 0;          // No of bytes written to the port

    //Writing data to Serial Port
    Status = WriteFile(hComm,// Handle to the Serialport
                       ptrDataToWrite,            // Data to be written to the port
                       strlen(ptrDataToWrite),   // No of bytes to write into the port
                       &BytesWritten,  // No of bytes written to the port
                       NULL);
    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nFail to Written");
        #endif
        Exit1();
    }
    //print numbers of byte written to the serial port
    #ifdef SERIAL_DEBUG
    printf_s("\nNumber of bytes written to the serail port = %d\n\n", BytesWritten);
    #endif
    return 0;
}

int serial_read(char * ptrDataReceived){
    BOOL   Status; // Status
    unsigned char charCount = 0;
    DWORD dwEventMask;     // Event mask to trigger
    char  ReadData;        //temperory Character
    DWORD NoBytesRead;     // Bytes read by ReadFile()
    

    //Setting Receive Mask
    Status = SetCommMask(hComm, EV_RXCHAR);

    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nError to in Setting CommMask\n\n");
        #endif
        Exit1();
    }

    //Setting WaitComm() Event
    Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received
    if (Status == FALSE)
    {
        #ifdef SERIAL_DEBUG
        printf_s("\nError! in Setting WaitCommEvent()\n\n");
        #endif
        Exit1();
    }

    //Read data and store in a buffer
    do
    {
        Status = ReadFile(hComm, &ReadData, sizeof(ReadData), &NoBytesRead, NULL);
        ptrDataReceived[charCount] = ReadData;
        ++charCount;
    }
    while (NoBytesRead > 0);

    --charCount; //Get Actual length of received data

    return charCount;
}

