
#include "Clockface.h"

EventBus eventBus;

const char* FORMAT_TWO_DIGITS = "%02d";

// Graphical
Tile ground(GROUND, 8, 8); 

Object bush(BUSH, 21, 9);
Object cloud1(CLOUD1, 13, 12);  // was 13, 12
Object cloud2(CLOUD2, 13, 12);   // was 13, 12
Object cloud3(CLOUD3, 17, 6);
Object hill(HILL, 20, 22);

Mario mario(21, 40);       //was 26
Block hourBlock(3, 8);
Block minuteBlock(22, 8);
Block secondBlock(41, 8);

unsigned long lastMillis = 0;
int lastSecond;

//**********************************************************************************************
Clockface::Clockface(Adafruit_GFX* display) 
{
  _display = display;

  Locator::provide(display);
  Locator::provide(&eventBus);
}
//**********************************************************************************************
void Clockface::setup(CWDateTime *dateTime) 
{
  _dateTime = dateTime;

  Locator::getDisplay()->setFont(&Super_Mario_Bros__24pt7b);
  Locator::getDisplay()->fillRect(0, 0, 64, 64, SKY_COLOR);

  ground.fillRow(DISPLAY_HEIGHT - ground._height);

  bush.draw(43, 47);
  hill.draw(0, 34);
  cloud1.draw(0, 25);
  cloud2.draw(51, 30);
  cloud3.draw(30, 0);

  updateTime();

  hourBlock.init();
  minuteBlock.init();
  secondBlock.init();
  mario.init();
}
//**********************************************************************************************
void Clockface::update() 
{
  int updatedSecond;
  
  hourBlock.update();
  minuteBlock.update();
  secondBlock.update();
  mario.update();
   
  updatedSecond = _dateTime->getSecond();
 
  if (lastSecond != updatedSecond) 
    {    
    mario.jump(); 
    updateTime();      
    //lastMillis = millis();
    lastSecond = updatedSecond;
    Serial.println(_dateTime->getFormattedTime());
    }


}
//**********************************************************************************************
void Clockface::externalEvent(int type) 
{
  if (type == 0) 
    {  //TODO create an enum
    mario.jump();
    updateTime();
    }
}
//**********************************************************************************************
void Clockface::updateTime() 
{
  hourBlock.setText(String(_dateTime->getHour()));
  minuteBlock.setText(String(_dateTime->getMinute(FORMAT_TWO_DIGITS)));
  secondBlock.setText(String(_dateTime->getSecond(FORMAT_TWO_DIGITS)));
}
//**********************************************************************************************
