#pragma once

#include <cstring>

#ifndef RB_MAX_LEN
#define RB_MAX_LEN 2048
#endif
#ifndef min
#define min(a, b) (a)<(b)?(a):(b)
#endif
class RingBuffer
{
public:
    RingBuffer(int size = RB_MAX_LEN);
    ~RingBuffer();

    int canRead();    //how much can read
    int canWrite();   //how much can write
    int read(void* data, int count);  //read data frome ringbuffer
    int write(const void* data, int count);
    int size();

private:
    int bufferSize;       //buffer size
    unsigned char* rbBuf = new unsigned char[bufferSize];
    /*���λ���������*/
    int rbCapacity; //����
    unsigned char* rbHead;
    unsigned char* rbTail;
    unsigned char* rbBuff;
};
