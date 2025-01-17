#include <Arduino.h>
#include <math.h>

// Function declarations
void handleInterrupt();
float calculateFrequency();
float interpolateTemperature();
void updateValue(float *array, float newValue);

// Pin configuration
const int inputPin = 8; // Pin used to measure the generator's frequency
const int pinNTC = 3;   // Pin used to measure the temperature sensor's voltage

volatile int pulses = 0;       // Pulse counter for frequency measurement
volatile unsigned long lastPulseTime = 0; // Time of the last valid pulse
unsigned long prevTime = 0;    // Previous time for frequency calculation

// Moving average filter configuration
const int sampleSize = 4;      // Number of samples for the moving average
float freqSamples[sampleSize]; // Frequency samples for averaging
float tempSamples[sampleSize]; // Temperature samples for averaging
int sampleIndex = 0;           // Index for circular buffer

const unsigned long debounceTime = 5; // Minimum time (ms) between valid pulses

void setup() {
  Serial.begin(115200);              // Initialize serial communication
  pinMode(inputPin, INPUT);          // Configure the frequency input pin as input
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, CHANGE); // Attach interrupt for pulse counting

  // Initialize filter arrays with zeros
  for (int i = 0; i < sampleSize; i++) {
    freqSamples[i] = 0.0;
    tempSamples[i] = 0.0;
  }
}

void loop() {
  // Calculate frequency
  float frequency = calculateFrequency();

  // Calculate temperature
  float temperature = interpolateTemperature();

  // Display temperature if valid
  if (temperature != -999) {
    updateValue(tempSamples, temperature); // Update moving average for temperature
    float averageTemperature = 0;
    for (int i = 0; i < sampleSize; i++) {
      averageTemperature += tempSamples[i];
    }
    averageTemperature /= sampleSize; // Calculate the average temperature

    Serial.print("Temperature: ");
    Serial.print(averageTemperature);
    Serial.println(" °C");
  }

  // Display frequency and RPM if valid
  if (frequency > 0) {
    updateValue(freqSamples, frequency); // Update moving average for frequency
    float averageFrequency = 0;
    for (int i = 0; i < sampleSize; i++) {
      averageFrequency += freqSamples[i];
    }
    averageFrequency /= sampleSize; // Calculate the average frequency

    float rpm = averageFrequency * 60.0; // Convert Hz to RPM
    Serial.print("Frequency: ");
    Serial.print(averageFrequency);
    Serial.print(" Hz, RPM: ");
    Serial.println(rpm);
  }

  delay(100); // Small delay for stability
}

// Interrupt service routine for pulse counting with debounce
void handleInterrupt() {
  unsigned long currentTime = millis(); // Get the current time
  if (currentTime - lastPulseTime > debounceTime) {
    pulses++;                  // Increment pulse count if debounce condition is met
    lastPulseTime = currentTime; // Update the time of the last valid pulse
  }
}

// Update moving average filter with a new value
void updateValue(float *array, float newValue) {
  array[sampleIndex] = newValue;             // Replace the oldest value with the new one
  sampleIndex = (sampleIndex + 1) % sampleSize; // Increment and wrap around the index
}

// Calculate frequency based on pulse counts
float calculateFrequency() {
  float frequency = 0.0;
  unsigned long currTime = millis();            // Get the current time in milliseconds
  float deltaTime = (currTime - prevTime) / 1000.0; // Time difference in seconds

  if (deltaTime >= 0.35) { // Sampling interval of 0.35 seconds
    noInterrupts();        // Disable interrupts to safely access pulse count
    int pulseCount = pulses;
    pulses = 0;            // Reset pulse counter
    interrupts();          // Re-enable interrupts

    frequency = pulseCount / deltaTime; // Calculate frequency in Hz
    prevTime = currTime;   // Update the previous time
  }
  return frequency;
}

// Interpolate temperature based on sensor readings
float interpolateTemperature() {
  // Convert ADC value to voltage
  float voltageNTC = 3.3 - (analogRead(pinNTC) * (3.3 / 4095.0));
  // Calculate NTC resistance in ohms
  float resistanceNTC = (10.0 * voltageNTC) / (3.3 - voltageNTC);
  // Calculate temperature using a logarithmic approximation
  float temperature = -23.45 * log(resistanceNTC) + 79.344;
  return temperature;
}
