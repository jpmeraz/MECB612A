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
volatile int pulses = 0;
unsigned long prevTime = 0;

// Moving average filter configuration
const int sampleSize = 4;
float freqSamples[sampleSize]; // Frequency samples for averaging
float tempSamples[sampleSize]; // Temperature samples for averaging
int sampleIndex = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(inputPin, INPUT); // Set the input pin for frequency measurement
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, CHANGE);

  // Initialize the filter arrays with zeros
  for (int i = 0; i < sampleSize; i++)
  {
    freqSamples[i] = 0.0;
    tempSamples[i] = 0.0;
  }
}

void loop()
{
  // Calculate the frequency
  float frequency = calculateFrequency();

  // Calculate the temperature
  float temperature = interpolateTemperature();

  // Print the temperature if valid
  if (temperature != -999)
  {
    updateValue(tempSamples, temperature); // Update the filtered temperature
    float averageTemperature = 0;
    for (int i = 0; i < sampleSize; i++)
    {
      averageTemperature += tempSamples[i];
    }
    averageTemperature /= sampleSize;

    Serial.print("Temperature: ");
    Serial.print(averageTemperature);
    Serial.println(" °C");
  }

  // Print the frequency and RPM if valid
  if (frequency > 0)
  {
    updateValue(freqSamples, frequency); // Update the filtered frequency
    float averageFrequency = 0;
    for (int i = 0; i < sampleSize; i++)
    {
      averageFrequency += freqSamples[i];
    }
    averageFrequency /= sampleSize;

    float rpm = averageFrequency * 60.0; // Convert Hz to RPM
    float periodo = 1.0 / averageFrequency;
    float litros = (-1.2186*periodo*1000) + 20.091;
    Serial.print("Frequency: ");
    Serial.print(averageFrequency);
    Serial.print(" Hz, RPM: ");
    Serial.print(rpm);
    Serial.print(" L/min: ");
    Serial.println(litros);
  }
  delay(100); // Small delay for stability
}

// Interrupt service routine to count pulses
void handleInterrupt()
{
  pulses++;
}

// Update the moving average filter with a new value
void updateValue(float *array, float newValue)
{
  array[sampleIndex] = newValue;                // Replace the oldest value with the new one
  sampleIndex = (sampleIndex + 1) % sampleSize; // Increment and wrap around
}

// Calculate the frequency from pulses
float calculateFrequency()
{
  float frequency = 0.0;
  unsigned long currTime = millis();
  float deltaTime = (currTime - prevTime) / 1000.0; // Time difference in seconds

  if (deltaTime >= 0.35)
  {                              // Sampling interval of 0.35 seconds
    noInterrupts();              // Disable interrupts to safely read the pulse count
    int pulseCount = pulses / 2; // Divide by 2 to account for pulse pairs
    pulses = 0;                  // Reset pulse counter
    interrupts();                // Re-enable interrupts

    frequency = pulseCount / deltaTime;
    prevTime = currTime;
  }
  if (frequency > 600){
    return 0;
  }

  return frequency;
}

// Interpolate the temperature from sensor readings
float interpolateTemperature()
{
  // Convert ADC value to voltage
  float voltageNTC = 3.3 - (analogRead(pinNTC) * (3.3 / 4095.0));
  // Calculate resistance of the NTC thermistor
  float resistanceNTC = (10.0 * voltageNTC) / (3.3 - voltageNTC);
  // Calculate temperature using a logarithmic approximation
  float temperature = -23.45 * log(resistanceNTC) + 79.344;
  return temperature;
}