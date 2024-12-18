# Temperature-Controlled Heating System

## Overview

The **Temperature-Controlled Heating System** is a project designed to maintain a desired temperature using a feedback control mechanism. The system utilizes a **Proportional-Integral-Derivative (PID)** controller to adjust the heating element's output dynamically, ensuring that the setpoint temperature is achieved and maintained.

This project integrates:

- A temperature sensor for real-time temperature measurement.
- A relay to control the heating element.
- An LCD display for user feedback.
- A potentiometer to set the desired temperature.

## Features

1. **Automatic Temperature Control**: Uses a PID controller to regulate the heating system.
2. **Real-Time Feedback**: Displays the set temperature and current temperature on an LCD.
3. **User-Friendly Interface**: Allows users to set the desired temperature using a potentiometer.
4. **Logging**: Logs real-time data to the serial monitor for debugging and analysis.

## Hardware Requirements

1. **Microcontroller**: Arduino Uno 
2. **LCD Display**: I2C 16x2 LCD.
3. **Temperature Sensor**: LM35 
4. **Relay Module**: For controlling the heating element.
5. **Potentiometer**: For setting the desired temperature.
6. **Resistors and Wires**: For connections.
7. **Power Supply**: As required by the heating element and Arduino board.

## Software Requirements

- Arduino IDE
- Libraries:
  - **Wire.h**: For I2C communication with the LCD.
  - **LiquidCrystal\_I2C.h**: To drive the LCD.
  - **PID\_v1.h**: For PID control.

## System Workflow

1. The potentiometer sets the desired temperature (“Set Value”).
2. The LM35 sensor measures the current temperature.
3. The PID controller compares the set temperature with the current temperature and adjusts the heating element via a relay.
4. The LCD displays the set and current temperatures in real-time.
5. Temperature data and PID output are logged to the serial monitor for analysis.

## How to Use

### Hardware Setup

1. Connect the temperature sensor to the **A1** pin of the Arduino.
2. Connect the potentiometer to the **A0** pin.
3. Connect the relay module to pin **3**.
4. Connect the LCD via I2C to the Arduino.
5. Ensure proper power connections to all components.

### Software Setup

1. Install the necessary libraries in the Arduino IDE.
2. Upload the provided code to the Arduino.
3. Power up the system and observe the LCD display.

### Operation

1. Turn the potentiometer to set the desired temperature (range: 30°C to 100°C).
2. The system will automatically adjust the heating to maintain the set temperature.
3. Observe the real-time data on the LCD and serial monitor.

## Code Explanation

The code for this project performs the following key functions:

1. **Temperature Measurement**: Reads the analog value from the LM35 sensor and converts it to Celsius. Also, it reads the analog value from the potentiometer for the desired temperature. 
2. **PID Control**:
   - Compares the set and current temperatures.
   - Generates an output signal to control the heating element via the relay.
3. **LCD Updates**: Displays the current and set temperatures every second.
4. **Relay Control Delay**: Implements a delay mechanism to prevent rapid switching of the relay.




## Schematics and Documentation

### Block Diagram

![image](https://github.com/user-attachments/assets/c421032a-a2e9-4ecf-be87-3ea155484541)


### Circuit Diagram

![circuit_image](https://github.com/user-attachments/assets/7a1f19fe-6378-490d-9740-224c6d239ce8)

https://app.cirkitdesigner.com/project/2b7538c4-bf4b-4e95-8bf0-32d53cf81fa6
### Additional Documentation
![image](https://github.com/user-attachments/assets/633c63cd-0c9c-434f-9f9c-c5f3e512e26f)
![image](https://github.com/user-attachments/assets/c53d7071-8507-4d20-b915-e0edc607afef)


## Future Enhancements

1. Add support for more precise temperature sensors like the DS18B20.
2. Implement a mobile app interface for remote control and monitoring.
3. Add a safety feature to shut down the system if the temperature exceeds a critical threshold.
