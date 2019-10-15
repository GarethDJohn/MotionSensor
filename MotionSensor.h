#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include "Arduino.h"
#include <cppQueue.h>

const uint8_t MaxMotionSensors = 4;

enum MotionSensorState {
	MotionSensorStateNoMotion,
	MotionSensorStateMotion,
    MotionSensorStateBlockade
};

typedef void (* MotionSensorStateChangeCallback)(uint8_t pin, bool motion_detected);

enum MotionSensorEvent {
	MotionSensorEventNoMotionDetected,
	MotionSensorEventMotionDetected,
	MotionSensorEventCheckTime
};

/// @brief Class for managing one or more motion sensors attached to Arduino GPIOs.
/// @author Gareth John <gdjohn@logicalparadox.co.uk>
///
/// @details
/// The MotionSensor will indicate 'motion detected' via it's state-change
/// callback as soon as it's GPIO pin is driven HIGH. It will remain in this
/// state until both the delay period has passed and the GPIO pin is driven
/// LOW. If the GPIO pin is driven LOW during the delay period and then driven
/// HIGH then the delay period is restarted.
/// When both the GPIO pin is driven LOW and the delay period has passed, the
/// MotionSensor will indicate 'no motion detected via it's state-change
/// callback.
class MotionSensor {
	private:
        /** The pin that the sensor is attached to. */
		uint8_t pin;

        /**
         * Minimum time for which motion will be indicated after GPIO pin is driven HIGH.
         */
		uint32_t delay_time_ms;

        /**
		 * Time for which motion will be ignored after GPIO pin is driven LOW.
         */
        uint32_t blockade_time_ms;

        /** Current state. */
		MotionSensorState state;

        /** Callback to be called when motion state changes. */
		MotionSensorStateChangeCallback state_change_callback;

        /** Time at which motion was detected. */
		uint32_t motion_detected_time;

        /** State machine */
		void state_machine(MotionSensorEvent event);

	public:
	    /**
		 * Constructs a new MotionSensor.
		 *
		 * @param pin The GPIO pin to which the motion sensor is attached.
		 * @param delay_time_ms Period for which motion will be indicated after
		 *                      GPIO is driven HIGH.
		 * @param state_change_callback Callback used to indicate state changes.
		 */
		MotionSensor(uint8_t pin,
					 uint32_t delay_time_ms,
					 MotionSensorStateChangeCallback state_change_callback);

		/**
		 * MotionSensor destructor.
		 */
		~MotionSensor();

		/**
		 * Get the pin to which the motion sensor is attached.
		 */
		uint8_t get_pin() { return this->pin; }

		/**
		 * Queue used for communication between the ISR and task loop.
		 */
		volatile static Queue * event_queue;

		/**
		 * Internal list used to keep track of instantiated MotionSensor objects.
		 */
		static MotionSensor * motion_sensors[MaxMotionSensors];

		/**
		 * Shared ISR. This is shared between all instances of the MotionSensor.
		 */
		static void isr();

		/**
		 * Shared task loop routine.
		 */
		static void loop();
};

#endif /* MOTION_SENSOR_H */
