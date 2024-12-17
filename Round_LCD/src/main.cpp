#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// Define los pines utilizados para la pantalla TFT
#define TFT_CS    5
#define TFT_DC    16
#define TFT_RST   17
#define TFT_SCLK  18
#define TFT_MISO  19
#define TFT_MOSI  23

SPIClass mySPI(VSPI);  // O HSPI si prefieres usar otro bus SPI

// Crear objeto TFT con los pines y SPI personalizados
Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC);

#define TFT_RED 0xF800  // Define red color in RGB565 format
#define TFT_WHITE 0xFFFF  // Define white color in RGB565 format

void setup() {
  // Inicializar el bus SPI con los pines correspondientes
  mySPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  
  // Inicializar la pantalla TFT
  tft.begin();
  
  // Limpiar la pantalla con un color de fondo
  tft.fillScreen(TFT_RED);
  
  // Escribir algo en la pantalla
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.println("¡Hola, ESP32!");
}

void loop() {
  // Aquí puedes agregar más código para actualizar la pantalla
}