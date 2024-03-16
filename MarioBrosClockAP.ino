//*******************************************************************
// MarioBros Clock.ino
// Displays animated Mario sequence on 64x64 Matrix Display
// Program is modified from "jnthas" Mario Bros. Clock
//
// by RCI
//
// 1-8-2024
// Added to Clock Seconds and Mario updates it every second
//
// 3-16-2024
// Set TimeZone for AP mode in CWDateTime.cpp, CWDateTime::begin()
//
//*******************************************************************
#include <WiFi.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include "Clockface.h"
#include "CWDateTime.h"
#include <ezTime.h>

MatrixPanel_I2S_DMA *dma_display = nullptr;

CWDateTime cwDateTime;
Clockface *clockface;
Timezone myTZ;

// Replace with your network credentials
//const char *ssid = "RCINet2.4";
//const char *password = "cae4844bb2";

// Replace with your network credentials
const char *ssid = "josephine";
const char *password = "drain_8490follow";

#define panelResX 64     // Number of pixels wide of each INDIVIDUAL panel module. 
#define panelResY 64     // Number of pixels tall of each INDIVIDUAL panel module.
#define panel_chain 1      // Total number of panels chained one to another

#define displayBrightness 127

// #define SDA  21
// #define SCL  22
//**********************************************
//   WEMOS D1 MINI 32 Daughterboard PINOUTS
//**********************************************
//        VAR   IO PIN   BOARD LABEL    COLOR        DIN
#define R1_PIN  33    // IO33           BROWN 1       1        
#define G1_PIN  26    // IO26-D0        RED 1         2
#define B1_PIN  27    // IO27           ORANGE 1      3
                      // GND            YELLOW 1      4
#define R2_PIN  0     // IO0            GREEN 1       5
#define G2_PIN  18    // IO18-D5-SCK    BLUE 1        6
#define B2_PIN  19    // IO19-D6-MISO   PURPLE        7
#define E_PIN   32    // IO32           GREY          8
#define A_PIN   23    // IO23-D7-MOSI   WHITE         9   
#define B_PIN   12    // IO12-TDI       BLACK         10
#define C_PIN   5     // IO5-D8-CS      BROWN 2       11
#define D_PIN   17    // IO17           RED 2         12         
#define CLK_PIN 16    // IO16           ORANGE 2      13
#define LAT_PIN 4     // IO4            YELLOW 2      14
#define OE_PIN  25    // IO25           GREEN 2       15
                      //                BLUE 2 SPARE  16
#define INC_PB  14    // TMS
#define MODEPB  13    // TCK

uint16_t myBLACK = dma_display->color565(0, 0, 0);
uint16_t myWHITE = dma_display->color565(255, 255, 255);
uint16_t myBLUE = dma_display->color565(0, 0, 255);

//**********************************************************************************************
void displaySetup() 
{
  HUB75_I2S_CFG mxconfig(
    panelResX,   // module width
    panelResY,   // module height
    panel_chain  // Chain length
    );

  //******* 64x64 RGB MAtirx Display Setup *******
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  //dma_display->begin();
  // ***** Use this if you want to use your own Pins ****
  dma_display->begin(R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN);   
  dma_display->setBrightness8(displayBrightness); //0-255
  dma_display->clearScreen();
}
//**********************************************************************************************
void setup() 
{

  Serial.begin(115200);

  pinMode(INC_PB, INPUT_PULLUP);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) 
       {
       delay(500);
       Serial.print(".");
       }
  
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  displaySetup();

  clockface = new Clockface(dma_display);

  dma_display->setTextSize(1);
  dma_display->setTextColor(myWHITE);
  dma_display->setCursor(0, 0);
  dma_display->println("Mario Bros");
  dma_display->println(" Clock");
  dma_display->setTextColor(myBLUE);
  dma_display->setCursor(0, 25);

  //dma_display->println("Mario-Wifi");
  //dma_display->println("Enter Your");
  //dma_display->println("SSID/Pass");

  cwDateTime.begin();

  clockface->setup(&cwDateTime);
  
}
//**********************************************************************************************
void loop() 
{
  
  clockface->update();
  /*
  if(digitalRead(INC_PB) == 0)
     {
     while(digitalRead(INC_PB) == 0){} 
     WiFi.disconnect();
     delay(1000);
     wifiManager.resetSettings(); 
     delay(1000);
     ESP.restart();
     delay(1000);
     wifi.connect(); 
     }
  */

}
//**********************************************************************************************
//**********************************************************************************************
