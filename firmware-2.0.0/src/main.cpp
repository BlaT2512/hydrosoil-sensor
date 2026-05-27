/**
 * main.cpp
 * Initialises and runs HydroSensor code
 */

#include "sensor.hpp"

Sensor sensor;

void setup() {
    sensor.init();
    sensor.dispatchData();
}

void loop() {}
