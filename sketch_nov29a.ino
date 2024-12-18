#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pin Definitions
const int relayPin = 3;   // Relay control pin
const int potPin = A0;    // Potentiometer for set value
const int tempPin = A1;   // Temperature sensor pin

// Variables for Temperature Control
double setTemperature = 30;    // Desired temperature
double currentTemperature = 0; // Measured temperature
double output = 0;            // PID output

// PID Tuning Parameters
double Kp = 20.0, Ki = 20.0, Kd = 6.0;

// Create a PID instance
PID myPID(&currentTemperature, &output, &setTemperature, Kp, Ki, Kd, DIRECT);

// for lcd delay
unsigned long lastLogTime = 0;
const unsigned long logInterval = 1000;

//for delay in activation upon measured being bigger or lesser than set temperature
unsigned long lastSwitchTime = 0;
const unsigned long delayTime = 1000;

// for potentiometer debounce
int lastPotValue = 0;
const int potDeadband = 50; // Minimum change in pot value to update setpoint

void setup() {
  // Initialize LCD and Serial Communication
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);

  // Configure Pins
  pinMode(relayPin, OUTPUT);

  // Start the PID Controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // Output scaled for PWM (0-255)
  myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
  int potValue = analogRead(potPin);
  setTemperature = map(potValue, 0, 1023, 30, 100);
  setTemperature = constrain(setTemperature, 30, 100);

  // Measure temperature from sensor
  int adcVal = analogRead(tempPin) + map(potValue, 0, 1023, 0, 12); //this extra value added here is to deal w potentiometer noise
  double milliVolt = adcVal * (5000.0 / 1024.0);
  currentTemperature = milliVolt / 10;

  // Run PID Control
  myPID.Compute();

  //define current time
  unsigned long currentTime = millis();

  // control relay
  if (currentTime - lastSwitchTime >= delayTime) {
    lastSwitchTime = currentTime;

    if (output > 128) {
      digitalWrite(relayPin, LOW);
    }
    else {
      digitalWrite(relayPin, HIGH);
    }
  }

  // output every 1 s
  if (currentTime - lastLogTime >= logInterval) {
    lastLogTime = currentTime;

    // display on LCD
    lcd.setCursor(0, 0);
    lcd.print("Set Value: ");
    lcd.print((int) setTemperature);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Temp.: ");
    lcd.print((int) currentTemperature);
    lcd.print("C");

    // Log data
    Serial.print("Set Temp: ");
    Serial.print(setTemperature);
    Serial.print(" C | Current Temp: ");
    Serial.print(currentTemperature);
    Serial.print(" C | Output: ");
    Serial.println(output);
  }
}