//********************************************************************************
// BlinkingEyes.h
//
// Be sure to include in Main Program:      
//     mx.begin();
//     mx.control(MD_MAX72XX::INTENSITY, 1);
// Routines:
// void LookLeft(int dly);
// void EyesLeft();  
// void LookRight(int dly);
// void EyesRight(); 
// void BlankEyes();
// void OpenEyes();
// void ClosedEyes();
// void EvilEyes();
// void CloseEyes();
// void RaiseEyes();
// void QuickBlink(int dly) // delay between blinks
//
// by RCI
// 11/11/2023
//********************************************************************************
#include <MD_MAX72xx.h>

void delayms(int loops); 

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2

// FOR BLINKIN EYES.H
#define DATA_PIN  25      // IO25  
#define CS_PIN    26      // IO26 or D0
#define CLK_PIN   27      // IO27

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

int leftEye = 0;     // you can change the value of i from 0 to 3
int rightEye = 1;     // you can change the value of i from 0 to 3
//********************************************
const byte noEyes[8] = {
  
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};
//********************************************
const byte lookRight1[8] = {
  
  B00111100,
  B01000010,
  B01000010,
  B01000010,
  B01001110,
  B01001110,
  B01000010,
  B00111100
};
//********************************************
const byte lookRight2[8] = {
  
  B00011110,
  B00100001,
  B00100001,
  B00100001,
  B00100111,
  B00100111,
  B00100001,
  B00011110
};
//********************************************
const byte lookLeft1[8] = {
  
  B00111100,
  B01000010,
  B01000010,
  B01000010,
  B01110010,
  B01110010,
  B01000010,
  B00111100
};
//********************************************
const byte lookLeft2[8] = {
  
  B01111000,
  B10000100,
  B10000100,
  B10000100,
  B11100100,
  B11100100,
  B10000100,
  B01111000
};
//********************************************
const byte openEye[8] = {
  
  B00111100,
  B01000010,
  B01000010,
  B01000010,
  B01011010,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte closeEye1[8] = {
  
  B00000000,
  B00111100,
  B01000010,
  B01000010,
  B01011010,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte closeEye2[8] = {
  
  B00000000,
  B00000000,
  B00111100,
  B01000010,
  B01011010,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte closeEye3[8] = {
  
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B01011010,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte closeEye4[8] = {
  
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte closeEye5[8] = {

  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B01111110,
  B00111100

};
//********************************************
const byte closeEye6[8] = {

  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B01111110,
  B00111100

};
//********************************************
const byte raiseEye1[8] = {
  
  B00111100,
  B01000010,
  B01000010,
  B01011010,
  B01011010,
  B01000010,
  B01000010,
  B00111100
};
//********************************************
const byte raiseEye2[8] = {
  
  B00111100,
  B01000010,
  B01011010,
  B01011010,
  B01000010,
  B01000010,
  B01000010,
  B00111100
};
//********************************************
const byte raiseEye3[8] = {
  
  B00111100,
  B01011010,
  B01011010,
  B01000010,
  B01000010,
  B01000010,
  B01000010,
  B00111100
};
//********************************************
const byte closedEye1[8] = {
  
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B01111110,
  B00111100,
  B00000000,
  B00000000
};
//********************************************
const byte closedEye2[8] = {
  
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B01011010,
  B00111100,
  B00000000
};
//********************************************
const byte evilEyeLeft[8] = {
  
  B01000000,
  B01100000,
  B01010000,
  B01001000,
  B01010100,
  B01011010,
  B01000010,
  B00111100
};
//********************************************
const byte evilEyeRight[8] = {
  
  B00000010,
  B00000110,
  B00001010,
  B00010010,
  B00101010,
  B01011010,
  B01000010,
  B00111100
};
//***************************************************************************
void EyesLeft() 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookLeft2[j]);
           mx.setRow(1, j, lookLeft2[j]);  
           }
  
}
//***************************************************************************
void LookLeft(int dly) 
{
int moveDly = 60;
         
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookLeft1[j]); 
           mx.setRow(1, j, lookLeft1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookLeft2[j]);
           mx.setRow(1, j, lookLeft2[j]);  
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookLeft1[j]); 
           mx.setRow(1, j, lookLeft1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(dly);

}
//***************************************************************************
void EyesRight() 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookRight2[j]);
           mx.setRow(1, j, lookRight2[j]);  
           }
  
}
//***************************************************************************
void LookRight(int dly) 
{
int moveDly = 60;

         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookRight1[j]); 
           mx.setRow(1, j, lookRight1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookRight2[j]);
           mx.setRow(1, j, lookRight2[j]);  
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, lookRight1[j]); 
           mx.setRow(1, j, lookRight1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(dly);

}
//***************************************************************************
void BlankEyes() 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, noEyes[j]); 
           mx.setRow(1, j, noEyes[j]);              
           }

}
//***************************************************************************
void OpenEyes(int dly) 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(dly);
}
//***************************************************************************
void ClosedEyes(int dly) 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closedEye1[j]); 
           mx.setRow(1, j, closedEye1[j]);              
           }
           delayms(dly);
}
//***************************************************************************
void EvilEyes() 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, evilEyeRight[j]);
           mx.setRow(1, j, evilEyeLeft[j]);           
           }

}
//***************************************************************************
void RaisedEyes() 
{
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye3[j]);
           mx.setRow(1, j, raiseEye3[j]); 
           }
 
}
//***************************************************************************
void CloseEyes() 
{
  int moveDly = 60;
  
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye1[j]); 
           mx.setRow(1, j, closeEye1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye2[j]);
           mx.setRow(1, j, closeEye2[j]);  
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye3[j]);
           mx.setRow(1, j, closeEye3[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye4[j]);
           mx.setRow(1, j, closeEye4[j]);              
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye5[j]);
           mx.setRow(1, j, closeEye5[j]);                
           }           
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye6[j]);
           mx.setRow(1, j, closeEye6[j]);                
           }           
           delayms(moveDly);           
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye5[j]);
           mx.setRow(1, j, closeEye5[j]);                
           }           
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye4[j]);
           mx.setRow(1, j, closeEye4[j]);              
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye3[j]);
           mx.setRow(1, j, closeEye3[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye2[j]);
           mx.setRow(1, j, closeEye2[j]);  
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, closeEye1[j]); 
           mx.setRow(1, j, closeEye1[j]); 
           }
           delayms(moveDly);                       
}           
//***************************************************************************
void RaiseEyes() 
{
int moveDly = 60;
         
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, openEye[j]); 
           mx.setRow(1, j, openEye[j]);              
           }
           delayms(1000);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye1[j]); 
           mx.setRow(1, j, raiseEye1[j]); 
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye2[j]);
           mx.setRow(1, j, raiseEye2[j]);  
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye3[j]);
           mx.setRow(1, j, raiseEye3[j]); 
           }
           delayms(500);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye2[j]);
           mx.setRow(1, j, raiseEye2[j]);              
           }
           delayms(moveDly);
         for (int j = 0; j < 8; j++)
           {
           mx.setRow(0, j, raiseEye1[j]); 
           mx.setRow(1, j, raiseEye1[j]);              
           }           
           delayms(moveDly);
                      
}
//***************************************************************************
void QuickBlink(int dly) 
{
         OpenEyes(2000); 
         ClosedEyes(200); 
         OpenEyes(200); 
         ClosedEyes(200);
         OpenEyes(200);              
         delayms(dly);
}
