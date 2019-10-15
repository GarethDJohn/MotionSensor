# Arduino MotionSensor Library

## DESCRIPTION

This library is intended to make connecting PIR motion sensors to Arduino-like
devices more straightforward. 

## USAGE

```c++
#include "MotionSensor.h"

MotionSensor * motion_sensor;

void motion_sensor_state_change_callback(uint8_t pin, bool motion_detected) {
    if (motion_detected) {
        Serial.print("motion detected by sensor on pin ");
    } else {
        Serial.print("no motion detected by sensor on pin ");
    }
    Serial.println(pin);
}

void setup() {
    Serial.begin(115200);
    motion_sensor = new MotionSensor(5, 5000, motion_sensor_state_change_callback);
}

void loop() {
    MotionSensor::loop();
}
```

## AUTHOR

Gareth John <gdjohn@logicalparadox.co.uk>
