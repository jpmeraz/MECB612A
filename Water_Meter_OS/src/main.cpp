#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// Function declarations
void handleInterrupt();
float calculateFrequency();
float interpolateTemperature();
void updateValue(float *array, float newValue);
void reconnect();
void publishData();
void calculateTotalConsumption(float flowRate);

// WiFi configuration
const char *ssid = "UPC96676FA_EXT";
const char *password = "2sjfeQ8zsZbh";

// MQTT configuration
const char *mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char *mqttTopicTemperature = "esp32/temperature";
const char *mqttTopicConsumption = "esp32/consumption";
const char *mqttTopicTotalConsumption = "esp32/totalConsumption";

WiFiClient espClient;
PubSubClient client(espClient);

// Pin configuration
const int inputPin = 8; // Pin used to measure the generator's frequency
const int pinNTC = 3;   // Pin used to measure the temperature sensor's voltage
volatile int pulses = 0;
unsigned long prevTime = 0;

// Debouncing variables
volatile unsigned long lastInterruptTime = 0; // Time of the last valid interrupt
const unsigned long debounceTime = 10;        // Debounce time in milliseconds

// Moving average filter configuration
const int sampleSize = 4;
float freqSamples[sampleSize]; // Frequency samples for averaging
float tempSamples[sampleSize]; // Temperature samples for averaging
int sampleIndex = 0;

// Total consumption variables
float totalConsumption = 0.0;  // Total volume in liters
unsigned long lastIntegrationTime = 0; // Last time the integration was performed

void setup()
{
  Serial.begin(115200);

  // Initialize pins
  pinMode(inputPin, INPUT); // Set the input pin for frequency measurement
  attachInterrupt(digitalPinToInterrupt(inputPin), handleInterrupt, CHANGE);

  // Initialize the filter arrays with zeros
  for (int i = 0; i < sampleSize; i++)
  {
    freqSamples[i] = 0.0;
    tempSamples[i] = 0.0;
  }

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Set up MQTT
  client.setServer(mqttServer, mqttPort);
}

void loop()
{
  // Ensure MQTT connection
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Calculate the frequency
  float frequency = calculateFrequency();

  // Calculate the temperature
  float temperature = interpolateTemperature();

  // Publish temperature if valid
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

    // Publish to MQTT
    String tempMessage = String(averageTemperature);
    client.publish(mqttTopicTemperature, tempMessage.c_str());
  }

  // Publish frequency and calculate consumption if valid
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
    float litros = (-1.2186 * periodo * 1000) + 20.091; // Flow rate in L/min

    Serial.print("Frequency: ");
    Serial.print(averageFrequency);
    Serial.print(" Hz, RPM: ");
    Serial.print(rpm);
    Serial.print(" L/min: ");
    Serial.println(litros);

    // Update and publish total consumption
    calculateTotalConsumption(litros);

    // Publish frequency to MQTT
    String consumptionMessage = String(litros);
    client.publish(mqttTopicConsumption, consumptionMessage.c_str());
  }

  delay(100); // Small delay for stability
}

// Interrupt service routine to count pulses
void handleInterrupt()
{
  unsigned long currentInterruptTime = millis();
  if (currentInterruptTime - lastInterruptTime > debounceTime) // Check if debounce time has passed
  {
    pulses++;
    lastInterruptTime = currentInterruptTime; // Update last interrupt time
  }
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
  if (frequency > 600)
  {
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

// Calculate and publish total consumption
void calculateTotalConsumption(float flowRate)
{
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastIntegrationTime) / 60000.0; // Time in minutes
  lastIntegrationTime = currentTime;

  totalConsumption += flowRate * deltaTime; // Add flow (L/min * minutes) to total consumption

  Serial.print("Total Consumption: ");
  Serial.print(totalConsumption);
  Serial.println(" L");

  // Publish total consumption to MQTT
  String consumptionMessage = String(totalConsumption);
  client.publish(mqttTopicTotalConsumption, consumptionMessage.c_str());
}

// Reconnect to MQTT broker
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
