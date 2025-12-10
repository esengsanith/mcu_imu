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
    int copy(IMUDataPoint* destination);
    void clear();

private:
    IMUDataPoint buffer[BUFFER_SIZE];
    int head;
    int count; // number of valid items currently in the buffer
    mutable std::mutex buffer_mutex; 
};

