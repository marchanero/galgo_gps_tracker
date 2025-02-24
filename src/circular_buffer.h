#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <Arduino.h>

template<typename T, size_t SIZE>
class CircularBuffer {
private:
    T buffer[SIZE];
    size_t head;
    size_t count;
    T sum;
    
public:
    CircularBuffer() : head(0), count(0), sum(0) {}
    
    void push(T value) {
        // Si el buffer está lleno, restar el valor más antiguo de la suma
        if (count == SIZE) {
            sum -= buffer[head];
        } else {
            count++;
        }
        
        // Añadir nuevo valor
        buffer[head] = value;
        sum += value;
        
        // Mover head
        head = (head + 1) % SIZE;
    }
    
    T getAverage() const {
        if (count == 0) return T(0);
        return sum / count;
    }
    
    T* getData() { return buffer; }
    size_t getCount() const { return count; }
    size_t getCapacity() const { return SIZE; }
    
    void reset() {
        head = 0;
        count = 0;
        sum = 0;
    }
};

// Estructura para datos del IMU con buffers circulares
struct IMUHistory {
    static const size_t BUFFER_SIZE = 32;  // Tamaño configurable
    
    CircularBuffer<float, BUFFER_SIZE> accel_magnitude;
    CircularBuffer<float, BUFFER_SIZE> gyro_magnitude;
    
    void update(float accel_mag, float gyro_mag) {
        accel_magnitude.push(accel_mag);
        gyro_magnitude.push(gyro_mag);
    }
    
    float getAccelAvg() const { return accel_magnitude.getAverage(); }
    float getGyroAvg() const { return gyro_magnitude.getAverage(); }
};

#endif // CIRCULAR_BUFFER_H