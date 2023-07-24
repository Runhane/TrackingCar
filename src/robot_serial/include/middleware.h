#ifndef MIDDLEWARE_H
#define MIDDLEWARE_H

#include <serial/serial.h>
#include <iostream>
#include <string>
#include <queue>
#include <thread>

#define receiveDataSize 39
#define sendDataSize    7
#define frameHeader 0x5B
#define frameTail   0x5D

struct recData
{
    // update wheel odometry or not
    uint8_t flag;
    uint64_t dataTime;
    // velocity
    float velX;
    float velY;
    float velZ;
    // IMU
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    // power voltage
    float powVol;
};


class middleWare
{
public:
    middleWare();
    bool init();
    // get data from serial
    void getData();
    // send data through serial
    bool sendData(uint8_t flag,short velX,short velY);
    // store transformed data 
    std::queue<recData> recQue;
    // new a serial object
    serial::Serial mSerial;
    // transform raw data 
    uint64_t timeTrans(uint8_t dataHighH,uint8_t dataHighL,uint8_t dataLowH,uint8_t dataLowL);
    float velpowTrans(uint8_t dataHigh,uint8_t dataLow);
    float imuTrans(uint8_t dataHighH,uint8_t dataHighL,uint8_t dataLowH,uint8_t dataLowL);
    // receive bag
    recData transData;

};

#endif