// All the mcufriend.com UNO shields have the same pinout.
// i.e. control pins A0-A4.  Data D2-D9.  microSD D10-D13.
// Touchscreens are normally A1, A2, D7, D6 but the order varies
//
// This demo should work with most Adafruit TFT libraries
// If you are not using a shield,  use a full Adafruit constructor()
// e.g. Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

const String OS_NAME = "THUNDERBOLT";
const String OS_VERSION = "V1.2.0";

#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
//#include <Adafruit_TFTLCD.h>
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

void setup(void);
void loop(void);
unsigned long testFillScreen();
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
void progmemPrint(const char *str);
void progmemPrintln(const char *str);

void runtests(void);

uint16_t g_identifier;

// WEIGHT SENSOR DATA
const int gnd1 = 13, gnd2 = 10, vcc1 = 14, vcc2 = 11;
const int sck = 15, dout = 12;

// IR SENSOR DATA
const int irGND = A13, irVCC = A14, irSIG = A15;
bool hasRotated = false;
float rotations = 0.0;
float timeElapsed = millis();
float rpm = 0.0;
#include <Hx711.h>
Hx711 scale(A12, A15);


bool startSystem = false;

void setup(void) {
  digitalWrite(gnd1, LOW);
  digitalWrite(gnd2, LOW);
  digitalWrite(vcc1, HIGH);
  digitalWrite(vcc2, HIGH);

  pinMode(irSIG, INPUT);
  pinMode(irGND, OUTPUT);
  pinMode(irVCC, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(irGND, LOW);
  digitalWrite(irVCC, HIGH);
  
    Serial.begin(9600);
    uint32_t when = millis();
    //    while (!Serial) ;   //hangs a Leonardo until you connect a Serial
    if (!Serial) delay(5000);           //allow some time for Leonardo
    Serial.println("Serial took " + String((millis() - when)) + "ms to start");
    //    tft.reset();                 //hardware reset
    uint16_t ID = tft.readID(); //
    Serial.print("ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
//    ID = 0x9329;                             // force ID
    tft.begin(ID);
  
  startupInit();
    
}


void printmsg(int row, const char *msg)
{
    tft.setTextColor(YELLOW, BLACK);
    tft.setCursor(0, row);
    tft.println(msg);
}

void loop(void) {
  Serial.println(analogRead(A15));
  irSense(); 
}

int lineIteration = 0;
float total = 0.0;
float t = 0.0;

void senseWeight(void){
  if(startSystem){
    if(lineIteration == 20){
      tft.setTextColor(GREEN); 
      tft.setTextSize(2);
      tft.fillScreen(BLACK);
      tft.setCursor(0,0); 
      lineIteration = 0;
    }
    
    /*Serial.print(scale.getGram(), 1);
    Serial.println(" g");
    tft.println("Weight: " + String(scale.getGram()) + " g");*/
    
    lineIteration++;
    delay(200);
  }
}


void irSense(void){
  if(startSystem){
    if(lineIteration == 20){
      tft.setTextColor(GREEN); 
      tft.setTextSize(2);
      tft.fillScreen(BLACK);
      tft.setCursor(0,0); 
      lineIteration = 0;
    }
    
    //tft.println(analogRead(irSIG));    
    if(analogRead(irSIG)<100 && !hasRotated){
      
      hasRotated = true;
      rotations++;
    }
    if(analogRead(irSIG)>500 && hasRotated){
      hasRotated = false;
    }
    lineIteration++;
    timeElapsed = millis();    
    rpm = rotations/(timeElapsed/1000)/60;
    tft.println(rotations);

    Serial.println(rotations);
    t = timeElapsed/60000;
    Serial.println(t);
    total = rotations/t;
    Serial.println(total);
    
    
  }
}

// Standard Adafruit tests.  will adjust to screen size

unsigned long startupInit() {
    unsigned long start;
    tft.setRotation(1);
    tft.fillScreen(BLACK);
    start = micros();
    tft.setCursor(0, 0);
    tft.setTextColor(WHITE);  tft.setTextSize(1);
    tft.println("UOE Racing");
    tft.setTextColor(YELLOW); tft.setTextSize(2);
    tft.println(OS_VERSION);
    tft.setTextColor(WHITE);    tft.setTextSize(2);
    tft.println("Telemetry Unit");
    tft.println();
    tft.setTextColor(RED);
    tft.setTextSize(3);
    tft.println("'" + OS_NAME + "' OS");
    tft.setTextSize(2);
    tft.println("Starting up...");
    tft.println();
    tft.println();
    tft.println();
    tft.setTextColor(GREEN);
    tft.setTextSize(2);
    delay(500);
    tft.println(">Weight Sensor");
    delay(500);
    tft.println(">Voltage Regulator");
    delay(500);
    tft.println(">Voltage Sensor");
    delay(500);
    tft.println(">Speed Sensor");
    delay(500);
    tft.setCursor(0,0);
    tft.fillScreen(BLACK);
    startSystem = true;
    return micros() - start;
}




