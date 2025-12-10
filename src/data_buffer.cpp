#include "data_buffer.h"
#include <cstring> // For memcpy

/**
 * @brief Constructor for the DataBuffer class
 * Initializes the buffer head to the starting position
 */
DataBuffer::DataBuffer() : head(0), count(0) {
}

/**
 * @brief Adds a new IMU data point to the buffer 
 * @param point The IMUDataPoint struct to add
 * @return TRUE if the point was added, FALSE if the buffer was full
 */
// bool DataBuffer::addPoint(const IMUDataPoint& point) {
//     std::lock_guard<std::mutex> lock(buffer_mutex);
//     if (head < BUFFER_SIZE) {
//         buffer[head++] = point;
//         return true;
//     }
//     // buffer full; new data point dropped
//     return false;
// }
// circular buffer implementation
bool DataBuffer::addPoint(const IMUDataPoint& point) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    buffer[head] = point;
    head = (head + 1) % BUFFER_SIZE; // wrap around if needed
    // update count (max BUFFER_SIZE)
    if (count < BUFFER_SIZE) {
        ++count;
    }
    return true; // always succeeds in circular buffer mode
}

/**
 * @brief Performs a fast, non-blocking copy of the buffer's contents and then clears it
 * This holds the mutex for the shortest possible time to prevent priority inversion
 * @param destination A pointer to a local buffer where the data will be copied
 * @return The number of data points that were copied from the buffer
 */
// int DataBuffer::copyAndClear(IMUDataPoint* destination) {
//     std::lock_guard<std::mutex> lock(buffer_mutex);
//     if (head == 0) {
//         return 0;
//     }
//     // copy data and clear buffer in one operation
//     memcpy(destination, buffer, head * sizeof(IMUDataPoint));
//     int copied_count = head;
//     head = 0; ied// reset head to indicate buffer is empty
//     return copied_count;
// }
// ensure the buffer is copied
int DataBuffer::copy(IMUDataPoint* destination) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (count == 0) {
        return 0;
    }
    int copied_count = count;
    // oldest element index (start) = head - count (mod BUFFER_SIZE)
    int start = (head + BUFFER_SIZE - count) % BUFFER_SIZE;

    // If the elements are contiguous from start to start+count-1 (no wrap), do single memcpy
    if (start + count <= BUFFER_SIZE) {
        memcpy(destination, &buffer[start], copied_count * sizeof(IMUDataPoint));
    } else {
        // wrapped: copy from start..BUFFER_SIZE-1, then from 0..remaining-1
        int first_part = BUFFER_SIZE - start;
        memcpy(destination, &buffer[start], first_part * sizeof(IMUDataPoint));
        memcpy(destination + first_part, &buffer[0], (copied_count - first_part) * sizeof(IMUDataPoint));
    }

    return copied_count;
}

/**
 * @brief Clears the buffer without copying any data
 */
void DataBuffer::clear() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    head = 0; // reset head to indicate buffer is empty
    count = 0;
}