//**************************************************************************************
// ESP32BRobotiPhone.ino
// AP Version.
// IP Address: 192.168.4.20
// Balance Robot using TouchOSC on iPad/iPhone.
// *** Requires use of TWO IP Addresses ***.
// See void receiveOSC() for TouchOSC control definitions.
// Works with ESP32 and iPAD/iPhone version of TouchOSC "B-Robot Control" template.
// In iPad/iPhone version of TouchOSC, click on "CHAIN" icon.
// ******************
// Connection 1: UDP
// Host: apIP 192.168.4.20 (ESP32 IP Address)
// Send Port: 8000
// Receive Port: 9000
// Zeroconf: Default
// ******************
//
// *** NOTE *** Receive outIP (iPad Address) must be set to one more than apIP (192.168.4.21)
//
// ***To create a new template for iPAD/iPhone that was created on PC:***
// First select the template you wish to transmit.
// Click on Wifi icon and Server Tab.
// Click "Enabled" checkbox.
// Then, open TouchOSC app on iPAD/iPhone and 
// Click on Wifi icon and Client Tab.
// MainWin7-PC should appear under "Available Servers".
// Press "Connect button".
// New template should appear on iPAD/iPhone.
// Press stop cirle at top right
// You should see the new temple under the name "Untitled".
// Press folder with "Down Arrow" on it, and "Untitled" should appear at bottom right.
// Change Filename to ***desired name*** and click on check mark.
// Tab should now show new filename.
// Press folder with "Up Arrow" on it, and new filename should appear in list.
// 
// 
// by RCI
// 11-8-2023
//
//**************************************************************************************
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>                
#include <OSCBundle.h>                
#include <Arduino.h>
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include "BlinkingEyes.h"

// **** Variables *****
int openEyeFlag;
int LREyeFlag;
int evilEyeFlag;
int count;
int eventcount;
int toggle;
int ssidFlag;
int battFlag;
int proButtonVal;
int servoButtonVal;
int LEDButtonVal;
int robotFaceVal;
int PIDButtonVal;
int eyeProgram;
unsigned long time_now;

//float BatteryValue;
float BatteryFloat;
char StringBattery[5];
String inputMessage;

float s1faderVal1;
String Strings1faderVal1;
float s1faderVal2;
String Strings1faderVal2;
float s2fader1Val;
float s2fader1ValLast;
String Strings2fader1Val;
float s2fader2Val;
float s2fader2ValLast;
String Strings2fader2Val;
float s2fader3Val;
float s2fader3ValLast;
String Strings2fader3Val;
float s2fader4Val;
float s2fader4ValLast;
String Strings2fader4Val;

// ********************** Wifi Credentials ********************
// Replace with your network credentials
const char* ssid = "BRobotiPhone";

IPAddress apIP(192,168,4,20);        // Address of ESP
IPAddress outIP(192,168,4,21);       // Address of ESP

WiFiUDP Udp;                         // A UDP instance to let us send and receive packets over UDP

const unsigned int inPort = 8000;    // local port to listen for UDP packets at the NodeMCU (another device must send OSC messages to this port)
const unsigned int outPort = 9000;   // remote port of the target device where the NodeMCU sends OSC to

String ipString;
unsigned long previousMillis = 0;

void initTimers();

//**************************************************************************************
void pidScan() 
{
  timer_value = micros();

  if (MPU6050_newData()) 
    {
    
    MPU6050_read_3axis();
    
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    //Serial.println(timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
      angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;


    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float) estimated_speed * 0.1; // low pass filter on estimated speed


    if (positionControlMode) 
      {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
      }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT,  MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    if (OSCpush[0])     // If we press the SERVO button we start to move
      angle_ready = 82;
    else
      angle_ready = 74;  // Default angle
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
      {
      // NORMAL MODE
      digitalWrite(PIN_ENABLE_MOTORS, LOW);  // Motors enable
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
      } 
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
      {
      digitalWrite(PIN_ENABLE_MOTORS, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
      }
    
    // Push1 Move servo arm
    if (OSCpush[0]) 
      {
      if (angle_adjusted > -40)
        ledcWrite(6, SERVO_MIN_PULSEWIDTH);   // was SERVO_MAX_PULSEWIDTH
      else
        ledcWrite(6, SERVO_MAX_PULSEWIDTH);   // was SERVO_MIN_PULSEWIDTH
      } 
    else
      ledcWrite(6, SERVO_AUX_NEUTRO);

    // Servo2
    //ledcWrite(6, SERVO2_NEUTRO + (OSCfader[2] - 0.5) * SERVO2_RANGE);

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56)) 
      {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
      } 
    else // We are in the raise up procedure => we use special control parameters
      {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      }

    } // End of new IMU data
     
}// End pidScan
//**************************************************************************************
// Delay (loops) Milliseconds
void delayms(int loops) 
{
    time_now = millis();
    
    while(millis() < time_now + loops)
      {
      pidScan(); 
      delay(1);   
      }
   
}
//**************************************************************************************
void initMPU6050() 
{
  MPU6050_setup();
  delay(500);
  MPU6050_calibrate();
}
//**************************************************************************************
// output : Battery voltage*10 (aprox) and noise filtered
float BROBOT_readBattery(bool first_time)
{
  float battery;
  if (first_time)
  battery = analogRead(A0);
  else
    battery = (battery*9 + (analogRead(A0)))/10;
  return battery;
}
//**************************************************************************************
void setup() 
{
  Serial.begin(115200);         // set up seriamonitor at 115200 bps
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println(ssid);
  Serial.println("--------------------------------------------------------");

  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);
  
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);

  pinMode(PIN_WIFI_LED, OUTPUT);
  digitalWrite(PIN_WIFI_LED, LOW);
  
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 1);  

  BlankEyes();  

  ledcSetup(6, 50, 16); // channel 6, 50 Hz, 16-bit width
  ledcAttachPin(PIN_SERVO, 6);   // GPIO 22 assigned to channel 1
  delay(50);
  ledcWrite(6, SERVO_AUX_NEUTRO);
  
  Wire.begin();
  initMPU6050();

  // Set NodeMCU Wifi hostname based on chip mac address
  String hostname = ssid;

  Serial.println();
  Serial.println("Hostname: "+hostname);

    // Wifi to access point mode.
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)); 
    delay(1000);
    Serial.println(WiFi.softAPIP());

  // try to connect with Wifi network about 8 seconds
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
  
  while (WiFi.status() != WL_CONNECTED && currentMillis - previousMillis <= 2000) 
    {
    delay(500);
    Serial.print(".");
    currentMillis = millis();
    }

    Udp.begin(apIP, inPort);
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.softAPIP());

  initTimers();

  // Check if these work!!
  OSCfader[0] = 0.5;
  OSCfader[1] = 0.5;
  OSCfader[2] = 0.1;
  OSCfader[3] = 0.080;
  OSCfader[4] = 0.1;
  OSCfader[5] = 0.1;

  digitalWrite(PIN_ENABLE_MOTORS, LOW);
  for (uint8_t k = 0; k < 5; k++) 
    {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    ledcWrite(6, SERVO_AUX_NEUTRO + 250);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    ledcWrite(6, SERVO_AUX_NEUTRO - 250);
    delay(200);
    }
  ledcWrite(6, SERVO_AUX_NEUTRO);
  eyeProgram = 0;
  count = 0;
  openEyeFlag = 1;
  LREyeFlag = 0;
  evilEyeFlag = 0; 
  
}
//**************************************************************************************
void loop() 
{
  receiveOSC();
  pidScan(); 
  
  if (OSCnewMessage) 
    {
    OSCnewMessage = 0;
    processOSCMsg();
    }
   
  if((OSCfader[0] == 0.5) || (OSCfader[1] == 0.5))   
    {                 
    openEyeFlag = 1;
    }
        
  if(openEyeFlag == 1)
    {
    OpenEyes(1);
    openEyeFlag = 0;
    }
    
       
}
//**************************************************************************************
void sendOSC(const char *message, const char *message2)
{
    OSCMessage msgOut(message);      
    msgOut.add(message2);
    Udp.beginPacket(outIP, outPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
    delay(10);
}
//**************************************************************************************
void sendOSCVal(const char *message, int val)
{
    OSCMessage msgOut(message);      
    msgOut.add(val);
    Udp.beginPacket(outIP, outPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
    delay(10);
}
//**************************************************************************************
void sendOSCFloat(const char *message, float val)
{
    OSCMessage msgOut(message);      
    msgOut.add(val);
    Udp.beginPacket(outIP, outPort);
    msgOut.send(Udp);
    Udp.endPacket();
    msgOut.empty();
    delay(10);
}
//**************************************************************************************
void receiveOSC() 
{
    OSCMessage msgIN;    
    int size;
    if ((size = Udp.parsePacket()) > 0) 
        {
        while (size--) 
          {
          msgIN.fill(Udp.read());
          }

        if (!msgIN.hasError()) 
          {
          msgIN.route("/1/xy1", s1fader1);          // Throttle
          msgIN.route("/1/xy1", s1fader2);          // Steering  
          msgIN.route("/1/push1",  servoButton);    // Activate Arm
          msgIN.route("/1/toggle1", LEDButton);     // Activate ESP LED
          msgIN.route("/1/button2", robotFace);     // Activate Eyes                            
          msgIN.route("/2/fader1", s2fader1);       // KP
          msgIN.route("/2/fader2", s2fader2);       // KD
          msgIN.route("/2/fader3", s2fader3);       // PThr 
          msgIN.route("/2/fader4", s2fader4);       // IThr
          msgIN.route("/2/button1", proButton);     // Send PID Values to TouchOSC               
          msgIN.route("/2/push1",  refreshPID);     // Refresh PID Values                                  
          }
        }//End If Size       
}
//**************************************************************************************
// Get value for Forward/Backward
void s1fader1(OSCMessage &msg, int addrOffset) 
{
      s1faderVal1 = msg.getFloat(1);
      Strings1faderVal1 = String(s1faderVal1);      
      OSCnewMessage = 1;
      OSCpage = 1;
      OSCfader[0] = s1faderVal1;
//      Serial.print("Y = ");
//      Serial.println(OSCfader[0]);

      if(OSCfader[0] < 0.4) 
        {
        EvilEyes(); 
        }
            
      if(s1faderVal1 == 0) 
        {
        OpenEyes(1);  
        PID_errorSum = 0;   
        OSCfader[0] = 0.5;  // May want to add to Servo Button
        OSCfader[1] = 0.5;  // May want to add to Servo Button              
        }
}
//**************************************************************************************
// Steer
void s1fader2(OSCMessage &msg, int addrOffset) 
{
      s1faderVal2 = msg.getFloat(0);
      Strings1faderVal2 = String(s1faderVal2);  
      OSCnewMessage = 1;
      OSCpage = 1;
      OSCfader[1] = s1faderVal2;    
//      Serial.print("X = ");
//      Serial.println(OSCfader[1]);
      
      if(s1faderVal2 == 0) 
        {  
        PID_errorSum = 0;  
        OSCfader[0] = 0.5;  // May want to add to Servo Button
        OSCfader[1] = 0.5;  // May want to add to Servo Button     
        }
}
//**************************************************************************************
// KP Value
void s2fader1(OSCMessage &msg, int addrOffset) 
{
    s2fader1Val = msg.getFloat(0);
    Strings2fader1Val = String(s2fader1Val, 1);  
    OSCnewMessage = 1;
    OSCpage = 2;
    OSCfader[2] = s2fader1Val;        
    //Serial.print("KP = ");
    //Serial.println(s2fader1Val); 
    if(s2fader1Val != s2fader1ValLast)
      {
      Kp_user = OSCfader[2];
      processOSCMsg();   
      sendOSCFloat("/2/label1", OSCfader[2]);
      //Serial.print("Kp_user = ");
      //Serial.println(Kp_user);           
      }
    s2fader1ValLast = s2fader1Val;   
    
}
//**************************************************************************************
// KD Value
void s2fader2(OSCMessage &msg, int addrOffset) 
{
    s2fader2Val = msg.getFloat(0);
    Strings2fader2Val = String(s2fader2Val, 1);     
    OSCnewMessage = 1;
    OSCpage = 2;
    OSCfader[3] = s2fader2Val;     
    //Serial.print("KD = ");
    //Serial.println(s2fader2Val);  
    if(s2fader2Val != s2fader2ValLast)
      {
      Kd_user = OSCfader[3];
      processOSCMsg();
      sendOSCFloat("/2/label2", OSCfader[3]); 
      //Serial.print("Kd_user = ");
      //Serial.println(Kd_user);         
      }
    s2fader2ValLast = s2fader2Val;    
}
//**************************************************************************************
// PThr Value
void s2fader3(OSCMessage &msg, int addrOffset) 
{
    s2fader3Val = msg.getFloat(0);
    Strings2fader3Val = String(s2fader3Val, 1); 
    OSCpage = 2;
    OSCnewMessage = 1;      
    OSCfader[4] = s2fader3Val;         
    //Serial.print("PThr = ");
    //Serial.println(s2fader3Val);  
    if(s2fader3Val != s2fader3ValLast)
      {
      Kp_thr_user = OSCfader[4];
      processOSCMsg();
      sendOSCFloat("/2/label3", OSCfader[4]);
      //Serial.print("Kp_thr_user = ");
      //Serial.println(Kp_thr_user);        
      }
    s2fader3ValLast = s2fader3Val;    
}
//**************************************************************************************
// IThr Value
void s2fader4(OSCMessage &msg, int addrOffset) 
{
    s2fader4Val = msg.getFloat(0);
    Strings2fader4Val = String(s2fader4Val, 1); 
    OSCpage = 2;
    OSCnewMessage = 1;
    OSCfader[5] = s2fader4Val;        
    //Serial.print("IThr = ");
    //Serial.println(s2fader4Val);  
    if(s2fader4Val != s2fader4ValLast)
      {
      Ki_thr_user = OSCfader[5];
      processOSCMsg();
      sendOSCFloat("/2/label4", OSCfader[5]);
      //Serial.print("Ki_thr_user = ");
      //Serial.println(Ki_thr_user);          
      }
    s2fader4ValLast = s2fader4Val;    
}
//**************************************************************************************
// PRO Push Button
void proButton(OSCMessage &msg, int addrOffset) 
{
    proButtonVal = msg.getInt(0);
    OSCpage = 1;
    if (proButtonVal == 0) 
      {     
      Serial.println("PRO OFF "); 
      } 
    else 
      {
      Serial.println("PRO ON "); 
      Kp_user = KP;
      Kd_user = KD;
      Kp_thr_user = KP_THROTTLE;
      Ki_thr_user = KI_THROTTLE;   
      }
}
//**************************************************************************************
// Servo Push Button
void servoButton(OSCMessage &msg, int addrOffset) 
{
    servoButtonVal = msg.getInt(0);
    
    if (servoButtonVal == 0) 
      {
      OSCpage = 1;
      OSCnewMessage = 1;
      OSCpush[0] = 0;      // activate servo arm
      Serial.println("Servo Up "); 
      sendOSC("/1/label", ssid);      
      BatteryValue = BROBOT_readBattery(true);
      BatteryFloat = map(BatteryValue, 0, 4095, 0, 15);
      dtostrf(BatteryFloat, 4, 1, StringBattery);             
      sendOSC("/1/label9", StringBattery); 
      Serial.println(StringBattery);
      Serial.println("  ");          
      } 
    else
      {
      OSCpage = 1;
      OSCnewMessage = 1;
      OSCpush[0] = 1;      // activate servo arm
      Serial.println("Servo Dn ");        
      }      

}
//**************************************************************************************
// LED Push Button
void LEDButton(OSCMessage &msg, int addrOffset) 
{
    LEDButtonVal = msg.getInt(0);
    
    if (LEDButtonVal == 0) 
      {
      digitalWrite(PIN_WIFI_LED, LOW);
      Serial.println("WEMOS LED OFF "); 
      } 
    else 
      {
      digitalWrite(PIN_WIFI_LED, HIGH); 
      Serial.println("WEMOS LED ON ");  
      }
}
//**************************************************************************************
// PID Push Button
void refreshPID(OSCMessage &msg, int addrOffset) 
{
    PIDButtonVal = msg.getInt(0);
    
    if (PIDButtonVal == 0) 
      {
      Serial.println("PID Button OFF "); 
      } 
    else 
      {
      Serial.println("PID Button ON ");  
      sendOSCFloat("/2/fader1", Kp);  // KP Value
      Serial.print("Kp = ");
      Serial.println(Kp);       
      sendOSCFloat("/2/label1", Kp);  // KP Value
       
      sendOSCFloat("/2/fader2", Kd);  // KD Value
      Serial.print("Kd = ");
      Serial.println(Kd);      
      sendOSCFloat("/2/label2", Kd);  // KD Value
      
      sendOSCFloat("/2/fader3", Kp_thr);  // PThr Value
      Serial.print("Kp_thr = ");
      Serial.println(Kp_thr);      
      sendOSCFloat("/2/label3", Kp_thr);  // KP Value
      
      sendOSCFloat("/2/fader4", Ki_thr);  // IThr Value
      Serial.print("Ki_thr = ");
      Serial.println(Ki_thr);       
      sendOSCFloat("/2/label4", Ki_thr);  // KP Value         
      }
}
//**************************************************************************************
// Face Push Button
void robotFace(OSCMessage &msg, int addrOffset) 
{
    robotFaceVal = msg.getInt(0);
    
    if (robotFaceVal == 0) 
      {
      BlankEyes(); 
      Serial.println("Eyes OFF "); 
      } 
    else 
      {
      ++eyeProgram;
      if (eyeProgram > 5) eyeProgram = 0;
      switch(eyeProgram)
         {
         case 0:

         break;
         case 1:         
              CloseEyes();             
         break;
         case 2:
              QuickBlink(1000);
         break;
         case 3:
              RaiseEyes();
         break;
         case 4:
              OpenEyes(1);
         break;
         case 5:
              EvilEyes();
         break;                  
         default:

         break;        
         }// End switch
 
      Serial.println("Eyes ON ");  
      }
}
//**************************************************************************************
void processOSCMsg() 
  {
  if (OSCpage == 1) 
    {
    if (modifing_control_parameters)  // We came from the PID Screen
      {
      speedMode = 0;          // Normal Mode
      modifing_control_parameters = false;
      }

    if (OSCmove_mode) 
      {
      positionControlMode = true;
      OSCmove_mode = false;
      target_steps1 = steps1 + OSCmove_steps1;
      target_steps2 = steps2 + OSCmove_steps2;
      } 
    else 
      {
      positionControlMode = false;
      throttle = (OSCfader[0] - 0.5) * max_throttle;

      // We add some exponential on steering to smooth the center b
      steering = OSCfader[1] - 0.5;
      if (steering > 0)
        {
        steering = (steering * steering + 0.5 * steering) * max_steering;
        }
      else
        {
        steering = (-steering * steering + 0.5 * steering) * max_steering;
        }
      }

    if ((speedMode == 0) && (OSCtoggle[0])) 
      {
      // Change to PRO mode
      max_throttle = MAX_THROTTLE_PRO;
      max_steering = MAX_STEERING_PRO;
      max_target_angle = MAX_TARGET_ANGLE_PRO;
      speedMode = 1;
      }
    if ((speedMode == 1) && (OSCtoggle[0] == 0)) 
      {
      // Change to NORMAL mode
      max_throttle = MAX_THROTTLE;
      max_steering = MAX_STEERING;
      max_target_angle = MAX_TARGET_ANGLE;
      speedMode = 0;
      }
    } 
  else if (OSCpage == 2) 
    { 
    if (!modifing_control_parameters) 
      {
      for (uint8_t i = 0; i < 6; i++)
        {
        //OSCfader[i] = 0.5;  
        }        
      OSCtoggle[0] = 0;
      modifing_control_parameters = true;    
      }
      
    // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder3,4,5,6)
    // Now we need to adjust all the parameters all the times because we dont know what parameter has been moved
    //Kp_user = KP * 2 * OSCfader[2];
    //Kd_user = KD * 2 * OSCfader[3];
    //Kp_thr_user = KP_THROTTLE * 2 * OSCfader[4];
    //Ki_thr_user = KI_THROTTLE * 2 * OSCfader[5];
    Kp_user = OSCfader[2];
    Kd_user = OSCfader[3];
    Kp_thr_user = OSCfader[4];
    Ki_thr_user = OSCfader[5];

    }//End else if (OSCpage == 2) 

}// End processOSCMsg
//**************************************************************************************
