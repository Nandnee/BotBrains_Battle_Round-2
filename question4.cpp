#include <Servo.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

const int soilMoisturePin = A0;
const int pumpPin = 9;
const int potentiometerPin = A1;
const int buttonPin = 2;
const int buzzerPin = 3;
const int ledPin = 4;

Servo servoMotor;
volatile bool buttonPressed = false;

void setup() {
  pinMode(soilMoisturePin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(potentiometerPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(buttonPin), wakeUp, FALLING);
  servoMotor.attach(10);
  
  Serial.begin(9600);
}

void loop() {
  if (buttonPressed) {
    int threshold = analogRead(potentiometerPin) / 4; // Adjust to 0-255 range
    int soilMoisture = analogRead(soilMoisturePin);

    Serial.print("Soil Moisture: ");
    Serial.println(soilMoisture);
    Serial.print("Threshold: ");
    Serial.println(threshold);

    if (soilMoisture < threshold) {
      digitalWrite(pumpPin, HIGH);
      servoMotor.write(90);
      feedback(true);
      delay(5000);           // Watering duration
      digitalWrite(pumpPin, LOW);
      servoMotor.write(0);
      feedback(false);
    }

    delay(1000); // Check soil moisture every second

    enterSleepMode();
  }
}

void feedback(bool watering) {
  if (watering) {
    tone(buzzerPin, 1000, 200); // Buzzer beep
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
  } else {
    tone(buzzerPin, 500, 200); // Buzzer beep
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
  }
}

void wakeUp() {
  buttonPressed = true;
  detachInterrupt(digitalPinToInterrupt(buttonPin));
}

void enterSleepMode() {
  buttonPressed = false;
  attachInterrupt(digitalPinToInterrupt(buttonPin), wakeUp, FALLING);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(buttonPin));
}
