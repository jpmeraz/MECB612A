#include <Arduino.h>

void handleInterrupt();
float frequency_calc();

const int inputPin = 19; // Pin donde conectas la salida del generador
volatile int pulsos = 0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  pinMode(inputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, RISING);
}

void loop() {
    float frequency = frequency_calc();

    if (frequency > 0) {
        float rpm = frequency * 60.0; // Convertir Hz a RPM
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.print(" Hz, RPM: ");
        Serial.println(rpm);
    }
    delay(50); // Puedes considerar eliminar este delay para una lectura más continua
}

void handleInterrupt() {
  pulsos++;
}

float frequency_calc() {
  float frequency = 0.0;
  unsigned long currT = millis();
  float deltaT = (currT - prevTime) / 1000.0;

  if (deltaT >= 0.5) {
    noInterrupts();
    int pulseCount = pulsos;
    pulsos = 0;
    interrupts();

    frequency = pulseCount / deltaT;
    prevTime = currT;
  }
  return frequency;
}