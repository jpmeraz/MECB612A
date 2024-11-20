#include <Arduino.h>

void handleInterrupt();
float frequency_calc();
void updateFrequency(float newFrequency);

const int inputPin = 34; // Pin donde conectas la salida del generador
volatile int pulsos = 0;
unsigned long prevTime = 0;

// Configuración para el filtro de promedio móvil
const int sampleSize = 4;
float freqSamples[sampleSize];
int sampleIndex = 0;

void setup() {
  Serial.begin(115200);
  pinMode(inputPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, CHANGE);
  Serial.println("Iniciando...");

  // Inicializar el arreglo del filtro con ceros
  for (int i = 0; i < sampleSize; i++) {
    freqSamples[i] = 0.0;
  }
}

void loop() {
    float frequency = frequency_calc();

    if (frequency > 0) {
        updateFrequency(frequency); // Actualizamos la frecuencia filtrada
        float averageFrequency = 0;
        for (int i = 0; i < sampleSize; i++) {
            averageFrequency += freqSamples[i];
        }
        averageFrequency /= sampleSize;

        float rpm = averageFrequency * 60.0; // Convertir Hz a RPM
        Serial.print("Frequency: ");
        Serial.print(averageFrequency);
        Serial.print(" Hz, RPM: ");
        Serial.println(rpm);
    }
    delay(100); // Puedes considerar eliminar este delay para una lectura más continua
}

void handleInterrupt() {
  pulsos++;
}

float frequency_calc() {
  float frequency = 0.0;
  unsigned long currT = millis();
  float deltaT = (currT - prevTime) / 1000.0;

  if (deltaT >= 0.5) { // Intervalo de muestreo de 0.5 segundos
    noInterrupts();
    int pulseCount = pulsos / 2;
    pulsos = 0;
    interrupts();

    frequency = pulseCount / deltaT;
    prevTime = currT;
  }
  return frequency;
}

void updateFrequency(float newFrequency) {
    freqSamples[sampleIndex] = newFrequency;
    sampleIndex = (sampleIndex + 1) % sampleSize; // Avanzar al siguiente índice en el buffer
}