#include "middleware.h"
#include <ros/ros.h>

middleWare::middleWare()
{

}
bool middleWare::init()
{
    mSerial.setPort("/dev/ttyUSB0");
    mSerial.setBaudrate(115200);
    serial::Timeout _time = serial::Timeout::simpleTimeout(200); 
    mSerial.setTimeout(_time);
    mSerial.open();
    if (mSerial.isOpen())
    {
        std::cout << "serial opened!" << std::endl;
    }
    else
    {
        std::cout << "cannot open serial port" << std::endl;
        return false;
    }
    std::thread mThread(&middleWare::getData, this);
    mThread.detach();
    return true;
}

bool middleWare::sendData(uint8_t flag,short velX,short velY)
{
    uint8_t tx[sendDataSize];
    tx[0] = frameHeader;
    // reserved 
    tx[1] = flag;
    // velX
    tx[3] = velX;
    tx[2] = velX >> 8;
    // velY
    tx[5] = velY;
    tx[4] = velY >> 8;

    tx[6] = frameTail;
    mSerial.write(tx,sizeof (tx)); 
}

void middleWare::getData()
{
    static int count;
    uint8_t rxData[receiveDataSize],rxPtr[1];
    while (ros::ok())
    {
        mSerial.read(rxPtr,sizeof(rxPtr)); 
        rxData[count] = rxPtr[0];
        //std::cout << rxPtr[0] << std::endl;
        if (rxPtr[0] == frameHeader || count > 0)
        {
            count++;
        }
        else
            count = 0;
        if(count == receiveDataSize)
        {
            count = 0;
            if(rxData[receiveDataSize - 1] == frameTail)
            {
                transData.flag = rxData[1];
                transData.dataTime = timeTrans(rxData[2],rxData[3],rxData[4],rxData[5]);
                if(transData.flag == 1)
                {
                    transData.velX = velpowTrans(rxData[6],rxData[7]);
                    transData.velY = velpowTrans(rxData[8],rxData[9]);
                    transData.velZ = velpowTrans(rxData[10],rxData[11]);
                }
                transData.accX = imuTrans(rxData[12],rxData[13],rxData[14],rxData[15]);
                transData.accY = imuTrans(rxData[16],rxData[17],rxData[18],rxData[19]);
                transData.accZ = imuTrans(rxData[20],rxData[21],rxData[22],rxData[23]);
                transData.gyroX = imuTrans(rxData[24],rxData[25],rxData[26],rxData[27]);
                transData.gyroY = imuTrans(rxData[28],rxData[29],rxData[30],rxData[31]);
                transData.gyroZ = imuTrans(rxData[32],rxData[33],rxData[34],rxData[35]);
                transData.powVol = velpowTrans(rxData[36],rxData[37]);
                recQue.push(transData);
            }
        }
    }
}

uint64_t middleWare::timeTrans(uint8_t dataHighH,uint8_t dataHighL,uint8_t dataLowH,uint8_t dataLowL)
{
    uint32_t rawTime =  (static_cast<uint32_t>(dataHighH) << 24) | (static_cast<uint32_t>(dataHighL) << 16) | (static_cast<uint32_t>(dataLowH) << 8) | static_cast<uint32_t>(dataLowL);
    uint64_t time = static_cast<uint64_t>(rawTime)*1e6;
    return time;
}

float middleWare::velpowTrans(uint8_t dataHigh,uint8_t dataLow)
{
    float data_return;
    int16_t transition_16;
    transition_16 = (static_cast<int16_t>(dataHigh) << 8) | (static_cast<int16_t>(dataLow));
    data_return = static_cast<float>(transition_16) * 0.001;
    return data_return;
}

float middleWare::imuTrans(uint8_t dataHighH,uint8_t dataHighL,uint8_t dataLowH,uint8_t dataLowL)
{
    uint32_t imu = (static_cast<uint32_t>(dataHighH) << 24) | (static_cast<uint32_t>(dataHighL) << 16) | (static_cast<uint32_t>(dataLowH) << 8) | static_cast<uint32_t>(dataLowL);
    float returnData = static_cast<int32_t>(imu) * 0.001;
    return returnData;
}
