#include "data_buffer.h"
#include <cstring> // For memcpy

/**
 * @brief Constructor for the DataBuffer class
 * Initializes the buffer head to the starting position
 */
DataBuffer::DataBuffer() : head(0) {
}

/**
 * @brief Adds a new IMU data point to the buffer 
 * @param point The IMUDataPoint struct to add
 * @return TRUE if the point was added, FALSE if the buffer was full
 */
bool DataBuffer::addPoint(const IMUDataPoint& point) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (head < BUFFER_SIZE) {
        buffer[head++] = point;
        return true;
    }
    // buffer full; new data point dropped
    return false;
}

/**
 * @brief Performs a fast, non-blocking copy of the buffer's contents and then clears it
 * This holds the mutex for the shortest possible time to prevent priority inversion
 * @param destination A pointer to a local buffer where the data will be copied
 * @return The number of data points that were copied from the buffer
 */
int DataBuffer::copyAndClear(IMUDataPoint* destination) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (head == 0) {
        return 0;
    }
    // copy data and clear buffer in one operation
    memcpy(destination, buffer, head * sizeof(IMUDataPoint));
    int copied_count = head;
    head = 0; // reset head to indicate buffer is empty
    return copied_count;
}

