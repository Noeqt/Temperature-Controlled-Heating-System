Temperature-Controlled Heating System

Overview

The Temperature-Controlled Heating System is a mini-project designed to maintain a desired temperature using a feedback control mechanism. The system utilizes a Proportional-Integral-Derivative (PID) controller to adjust the heating element's output dynamically, ensuring that the setpoint temperature is achieved and maintained.

This project integrates:

A temperature sensor for real-time temperature measurement.

A relay to control the heating element.

An LCD display for user feedback.

A potentiometer to set the desired temperature.

Features

Automatic Temperature Control: Uses a PID controller to regulate the heating system.

Real-Time Feedback: Displays the set temperature and current temperature on an LCD.

User-Friendly Interface: Allows users to set the desired temperature using a potentiometer.

Logging: Logs real-time data to the serial monitor for debugging and analysis.

Hardware Requirements

Microcontroller: Arduino Uno (or compatible).

LCD Display: I2C 16x2 LCD.

Temperature Sensor: LM35 or equivalent analog temperature sensor.

Relay Module: For controlling the heating element.

Potentiometer: For setting the desired temperature.

Resistors and Wires: For connections.

Power Supply: As required by the heating element and Arduino board.

Software Requirements

Arduino IDE

Libraries:

Wire.h: For I2C communication with the LCD.

LiquidCrystal_I2C.h: To drive the LCD.

PID_v1.h: For PID control.

System Workflow

The potentiometer sets the desired temperature (“Set Value”).

The LM35 sensor measures the current temperature.

The PID controller compares the set temperature with the current temperature and adjusts the heating element via a relay.

The LCD displays the set and current temperatures in real-time.

Temperature data and PID output are logged to the serial monitor for analysis.

How to Use

Hardware Setup

Connect the temperature sensor to the A1 pin of the Arduino.

Connect the potentiometer to the A0 pin.

Connect the relay module to pin 3.

Connect the LCD via I2C to the Arduino.

Ensure proper power connections to all components.

Software Setup

Install the necessary libraries in the Arduino IDE.

Upload the provided code to the Arduino.

Power up the system and observe the LCD display.

Operation

Turn the potentiometer to set the desired temperature (range: 30°C to 100°C).

The system will automatically adjust the heating to maintain the set temperature.

Observe the real-time data on the LCD and serial monitor.

Code Explanation

The code for this project performs the following key functions:

Temperature Measurement: Reads the analog value from the LM35 sensor and converts it to Celsius.

PID Control:

Compares the set and current temperatures.

Generates an output signal to control the heating element via the relay.

LCD Updates: Displays the current and set temperatures every second.

Relay Control Delay: Implements a delay mechanism to prevent rapid switching of the relay.

Code Highlights

// PID Controller Setup
PID myPID(&currentTemperature, &output, &setTemperature, Kp, Ki, Kd, DIRECT);
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(0, 255);

// Read Temperature Sensor
int adcVal = analogRead(tempPin);
double milliVolt = adcVal * (5000.0 / 1024.0);
currentTemperature = (milliVolt / 10) + offset;

// Control Relay
if (output > 128) {
  digitalWrite(relayPin, LOW); // Heater ON
} else {
  digitalWrite(relayPin, HIGH); // Heater OFF
}

Future Enhancements

Add support for more precise temperature sensors like the DS18B20.

Implement a mobile app interface for remote control and monitoring.

Add a safety feature to shut down the system if the temperature exceeds a critical threshold.

Acknowledgments

Arduino Community for open-source libraries and resources.

PID Library by Brett Beauregard for robust control algorithms.

License

This project is open-source and free to use under the MIT License.

