#include "MotionSensor.h"

#define EI_ARDUINO_INTERRUPTED_PIN

#include <EnableInterrupt.h>

typedef struct {
	uint8_t pin;
	MotionSensorEvent event;
} MotionSensorEventQueueEntry;

// Create a single queue with enough records to have one in flight per
// possible motion sensor.
volatile Queue * MotionSensor::event_queue = new Queue(sizeof(MotionSensorEventQueueEntry), MaxMotionSensors, FIFO);

// Static array that will hold pointers to our MotionSensor structures
// when they are allocated. Adjust MaxMotionSensors to suit.
MotionSensor * MotionSensor::motion_sensors[MaxMotionSensors] = {0};

MotionSensor::MotionSensor(uint8_t pin,
                           uint32_t delay_time_ms,
                           MotionSensorStateChangeCallback state_change_callback) {
	this->pin = pin;
	this->delay_time_ms = delay_time_ms;
	this->state_change_callback = state_change_callback;
	this->state = MotionSensorStateNoMotion;

	pinMode(this->pin, INPUT_PULLUP);
	enableInterrupt(this->pin, MotionSensor::isr, CHANGE);

	// Find an empty slot in motion_sensors and fill it in
	for (int i = 0; i < MaxMotionSensors; i++) {
		if (MotionSensor::motion_sensors[i] == NULL) {
			MotionSensor::motion_sensors[i] = this;
			break;
		}
	}
}

MotionSensor::~MotionSensor() {
	for (int i = 0; i < MaxMotionSensors; i++) {
		if (MotionSensor::motion_sensors[i] == this) {
			MotionSensor::motion_sensors[i] = NULL;
			break;
		}
	}
}

/**
 * Motion Sensor state machine.
 * 
 * NoMotion -> Motion -> Blockade -> NoMotion
 */
void MotionSensor::state_machine(MotionSensorEvent event) {
	switch (this->state) {
		case MotionSensorStateNoMotion:
			switch (event) {
				case MotionSensorEventMotionDetected:
                    // Motion detected, change state and call callback.
					this->motion_detected_time = millis();
					this->state = MotionSensorStateMotion;
					this->state_change_callback(this->pin, true);
					break;
			}
			break;

		case MotionSensorStateMotion:
			switch (event) {
				case MotionSensorEventNoMotionDetected:
					this->state = MotionSensorStateBlockade;
					break;
			}
			break;

		case MotionSensorStateBlockade:
			switch (event) {
				case MotionSensorEventMotionDetected:
					this->state = MotionSensorStateMotion;
					break;
				
				case MotionSensorEventCheckTime:
					if (millis() - this->motion_detected_time >= this->delay_time_ms) {
						this->state = MotionSensorStateNoMotion;
						this->state_change_callback(this->pin, false);
					}
					break;
			}
			break;
	}
}

void MotionSensor::isr() {
	MotionSensorEventQueueEntry event_queue_entry = {
		.pin = arduinoInterruptedPin,
		.event = arduinoPinState == 0 ? MotionSensorEventNoMotionDetected : MotionSensorEventMotionDetected
	};

	MotionSensor::event_queue->push(&event_queue_entry);
}

void MotionSensor::loop() {
	MotionSensorEventQueueEntry event_queue_entry;

	while (1) {
        uint32_t time_before = millis();
		noInterrupts();
        // Fetch an entry from the event queue.
		bool event_queue_entry_valid = MotionSensor::event_queue->pop(&event_queue_entry);
		interrupts();
        uint32_t time_after = millis();

		if (event_queue_entry_valid) {
            // Search the active motion sensors for one that
            // matches the event queue entry. If we find one,
            // pass the event to it's state machine.
			for (int i = 0; i < MaxMotionSensors; i++) {
				if (MotionSensor::motion_sensors[i] != NULL &&
				    MotionSensor::motion_sensors[i]->pin == event_queue_entry.pin) {
					MotionSensor::motion_sensors[i]->state_machine(event_queue_entry.event);
					break;
				}
			}
		} else {
            // No more events on the event queue, so
            // exit the while loop.
			break;
		}
	}
    
	for (int i = 0; i < MaxMotionSensors; i++) {
		MotionSensor::motion_sensors[i]->state_machine(MotionSensorEventCheckTime);
	}
}
