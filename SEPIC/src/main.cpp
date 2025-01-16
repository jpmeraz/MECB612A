#include <Arduino.h>

const int PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 330000;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 7; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1); 

const int LED_OUTPUT_PIN = 18;

// put function declarations here:


void setup() {
  Serial.begin(115200);

  // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(LED_OUTPUT_PIN, PWM_CHANNEL);
}

void loop() {

  // fade up PWM on given channel
  for (float percentage = 1; (percentage <= 9 ); percentage++)
  {
    ledcWrite(PWM_CHANNEL, MAX_DUTY_CYCLE*(percentage/10)); 
    Serial.println(percentage);  
    delay(1000);
    
  
    }

  for (float percentage = 9; (percentage >= 1 ); percentage--)
  {
    ledcWrite(PWM_CHANNEL, MAX_DUTY_CYCLE*(percentage/10));   
    Serial.println(percentage);  
    delay(1000);
    
  }
}

