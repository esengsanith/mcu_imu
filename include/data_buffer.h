// data_buffer.h
#pragma once
#include "imu_handler.h"
#include "config.h"
#include <string>
#include <mutex>

class DataBuffer {
public:
    DataBuffer();
    bool addPoint(const IMUDataPoint& point);
    int copyAndClear(IMUDataPoint* destination);

private:
    IMUDataPoint buffer[BUFFER_SIZE];
    int head;
    mutable std::mutex buffer_mutex; 
};

