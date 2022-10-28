// Revision Note:
// This revision restructure the code so that when motors are running, minimal other code are ran. 
// Only interrupt will disrupt the motor
// Added LCD display which can display fault, display speed, select motor when blinking, adjust motor as well
// TODO::Adjust conversion rate so that it matches the real world

// Libraries
#include "Math.h"

#include <Wire.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

// State Variables
enum{
  Display_Speed = 0,
  Prompt_Error,
  Display_Error,
  Select_Motor,
  Adjust_Motor
};
int STATE = Display_Speed;

Adafruit_RGBLCDShield LCD = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
uint8_t BUTTONS = 0;
bool BUTTON_PRESSED = false;
int MotorSelected = 0;

// Parameters TODO::Find out the conversion rate
#define CM_RS_FR 45.5             // Crosslinker's Motor Flow Rate -> Rotational Speed (Step per micro liter)
#define BM_RS_FR 45.5             // Bio-ink's Motor Flow Rate -> Rotational Speed (Step per Sec)
#define RM_RS_TS 33.56            // Roller Motor Translational Speed -> Rotational Speed (Step per mm)
#define SM_RS_TS 12.5             // Stage Motor Translational Speed -> Rotational Speed (Step per mm)

// Pins Constants TODO::Assign actual pins
// Deadman Switch  (digital input)
#define DS 12
// Mode Selection Switch (digital input)
#define MS 15
// Jog Switch  (digital input)
#define FW 16
#define BW 17
// Limit Switches (interrupts)
#define PM_LS 22                  // Printer Mount Limit Switch
#define BM_LS 2                   // Bio-ink's Motor Limit Switch
#define CM_LS 3                   // Crosslinker's Motor Limit Switch
#define SM_FW_LS 18               // Stage Motor Limit Switch 1 (LS in Forward Direction)
#define SM_BW_LS 19               // Stage Motor Limit Switch 2 (LS in Backward Direction)
// Crosslinker's Motor
#define BM_STP 4                  // Pulse Width Modulation
#define BM_DIR 5                  // Direction
// Bio-ink's Motor
#define CM_STP 6                  // Pulse Width Modulation
#define CM_DIR 7                  // Direction
// Roller Motor
#define RM_STP 8                  // Pulse Width Modulation
#define RM_DIR 9                  // Direction
// Stage Motor
#define SM_STP 10                 // Pulse Width Modulation
#define SM_DIR 11                 // Direction
// LEDs (Digital Pins)
#define GLED 13                   // Green LED
#define RLED 14                   // Red LED

// Other Global Variables
// Switch Status
bool DS_STATUS = LOW;             // HIGH::Pressed
bool MODE = LOW;                  // HIGH::Use Stage
bool CM_LS_STATUS = HIGH;         // HIGH::Not Tripped
bool BM_LS_STATUS = HIGH;         // HIGH::Not Tripped
bool SM_FW_LS_STATUS = HIGH;      // HIGH::Not Tripped
bool SM_BW_LS_STATUS = HIGH;      // HIGH::Not Tripped
bool PM_LS_STATUS = HIGH;         // HIGH::No Interference
bool FORWARD = LOW;               // Jog switch forwrad
bool BACKWARD = LOW;              // Jog switch backward

// Motor Speed
float CM_FR = 11.0;               // Crosslinker Flow Rate (uL / s)
float BM_FR = 3.3;                // Bio-ink Flow Rate (uL / s)
float RM_TS = 3.0;                // Roller Translational Speed (mm / s)
float SM_TS = 3.0;                // Stage Translational Speed (mm / s)
#define SM_JOG_SPEED 6.0

// Brightness
int G_B = 0;                      // Green LED
int R_B = 255;                    // Red LED
int INC = 1;                      // Brightness change per cycle

// Motor Objects
AccelStepper CM(AccelStepper::FULL2WIRE, CM_STP, CM_DIR);
AccelStepper BM(AccelStepper::FULL2WIRE, BM_STP, BM_DIR);
AccelStepper RM(AccelStepper::FULL2WIRE, RM_STP, RM_DIR);
AccelStepper SM(AccelStepper::FULL2WIRE, SM_STP, SM_DIR);
  
void setup(){
  // Enable Serial Communication
  Serial.begin(9600);
  // Start LCD Display
  LCD.begin(16, 2);

  // Setup Input Pins
  // Deadman Switch
  pinMode(DS, INPUT);
  // Mode Selection
  pinMode(MS, INPUT);
  // Jog Switch
  pinMode(FW, INPUT);
  pinMode(BW, INPUT);
  // Limit Switches
  pinMode(CM_LS, INPUT);
  attachInterrupt(digitalPinToInterrupt(CM_LS), Update_CM_LS, CHANGE);
  pinMode(BM_LS, INPUT);
  attachInterrupt(digitalPinToInterrupt(BM_LS), Update_BM_LS, CHANGE);
  pinMode(SM_FW_LS, INPUT);
  attachInterrupt(digitalPinToInterrupt(SM_FW_LS), Update_SM_FW_LS, CHANGE);
  pinMode(SM_BW_LS, INPUT);
  attachInterrupt(digitalPinToInterrupt(SM_BW_LS), Update_SM_BW_LS, CHANGE);
  pinMode(PM_LS, INPUT);
    
  // Setup Output Pins
  // Crosslinker's Motor
  CM.setMaxSpeed(10000);
  CM.setSpeed(CM_FR * CM_RS_FR * 15);
  // Bio-ink's Motor
  BM.setMaxSpeed(10000);
  BM.setSpeed(BM_FR * BM_RS_FR * 7.5);
  // Roller Motor
  RM.setMaxSpeed(10000);
  RM.setSpeed(RM_TS * RM_RS_TS * 17.45); //TODO::Figure out why the speed is off by scale of 10
  // Stage Motor
  SM.setMaxSpeed(10000);
  SM.setSpeed(SM_TS * SM_RS_TS * 1);
  // LEDs
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  
  // Initialize Deadman Switch and Limit Switch Status
  DS_STATUS = digitalRead(DS);
  CM_LS_STATUS = digitalRead(CM_LS);
  BM_LS_STATUS = digitalRead(BM_LS);
  SM_FW_LS_STATUS = digitalRead(SM_FW_LS);
  SM_BW_LS_STATUS = digitalRead(SM_BW_LS);
  PM_LS_STATUS = digitalRead(PM_LS);
  FORWARD = digitalRead(FW);
  BACKWARD = digitalRead(BW);
  
  // Initialization Complete
  LCD.print("Initializing...");
  delay(1000);
  DisplaySpeed();
  // Serial.println("Successfully Initialized!");
}

void loop(){ // TODO::Implement Control Loop and State Switching
  // Mode Check
  MODE = digitalRead(MS);
  
  // Button Check
  DS_STATUS = digitalRead(DS);
  FORWARD = digitalRead(FW);
  BACKWARD = digitalRead(BW);

  // Printer Mount Check
  PM_LS_STATUS = digitalRead(PM_LS);
  
  // Fault Display
  if(CM_LS_STATUS == LOW){                  
    // Serial.println("Crosslinker Depleted! Please Replenish!");
    analogWrite(RLED, R_B);
  }
  if(BM_LS_STATUS == LOW){                  
    // Serial.println("Bio-ink Depleted! Please Replenish!");
    analogWrite(RLED, R_B);
  }
  if(SM_FW_LS_STATUS == LOW && MODE == HIGH){
    // Serial.println("End of Forward Direction. Please Reverse!");
    analogWrite(RLED, R_B);
  }
  if(SM_BW_LS_STATUS == LOW && MODE == HIGH){
    // Serial.println("End of Backward Direction. Please Reverse!");
    analogWrite(RLED, R_B);
  }
  if(PM_LS_STATUS == LOW && MODE == HIGH){
    // Serial.println("Please clear the Print Head Interference!");
    analogWrite(RLED, R_B);
  }
  
  // Check Buttons
  uint8_t BUTTONS = LCD.readButtons();
  
  // State 0::At Rest
  if(DS_STATUS == LOW && FORWARD == LOW && BACKWARD == LOW){    // No Button Pressed - GLED Slowly ON Slowly OFF
    // State 0.1::Display Motor Speed
    if(STATE == Display_Speed){
      // State Task
      // Serial.println("Display_Speed");
      LCD.setBacklight(WHITE);
      DisplaySpeed();
      
      // State Transition
      if(CM_LS_STATUS == LOW || BM_LS_STATUS == LOW){
        STATE = Display_Speed;
        // STATE = Prompt_Error;
        // LCD.setBacklight(RED);
        delay(100);
        // LCD.clear();
      }
      if(MODE == HIGH && (SM_FW_LS_STATUS == LOW || SM_BW_LS_STATUS == LOW)){
        STATE = Display_Speed;
        // STATE = Prompt_Error;
        // LCD.setBacklight(RED);
        delay(100);
        // LCD.clear();
      }
      if((BUTTONS & BUTTON_LEFT) || BUTTONS & BUTTON_RIGHT){
        STATE = Select_Motor;
        // Serial.println("Selecting Motor");
        delay(100);
      }
    }
    
    // State 0.2::Prompt Error
    if(STATE == Prompt_Error){
      // State Task
      PromptError();
      uint8_t BUTTONS = LCD.readButtons();
      
      // State Transition
      if(BUTTONS & BUTTON_SELECT){
        STATE = Display_Error;
        delay(100);
        LCD.clear();
      }
    }

    // State 0.3:: Display Error
    if(STATE == Display_Error){
      // State Task
      uint8_t BUTTONS = LCD.readButtons();
      if(CM_LS_STATUS == LOW){
        LCD.setCursor(0, 0);
        LCD.print("Gel Depleted!");
      } 
      else if(BM_LS_STATUS == LOW){
        LCD.setCursor(0, 0);
        LCD.print("Bioink Depleted!");
      }
      else if(MODE == HIGH && SM_FW_LS_STATUS == LOW){
        LCD.setCursor(0, 0);
        LCD.print("FW Overtravel!");
      }
      else if(MODE == HIGH && SM_FW_LS_STATUS == LOW){
        LCD.setCursor(0, 0);
        LCD.print("BW Overtravel!!");
      }
      
      // State Transition
      else{
        STATE = Display_Speed;
        delay(100);
        LCD.clear();
      }
    }

    // State 0.4::Select Motor
    if(STATE == Select_Motor){
      // State Task
      BlinkLCD(MotorSelected);
      uint8_t BUTTONS = LCD.readButtons();
      
      // State Transition
      if(BUTTONS & BUTTON_SELECT){
        STATE = Adjust_Motor;
        delay(100);
        LCD.clear();
      }
      if(BUTTONS & BUTTON_RIGHT){
        if(MotorSelected != 2){
          MotorSelected = MotorSelected + 1;
        }
        else{
          MotorSelected = 0;
        }
      }
      if(BUTTONS & BUTTON_LEFT){
        if(MotorSelected != 0){
          MotorSelected = MotorSelected - 1;
        }
        else{
          MotorSelected = 2;
        }
      }
    }

    // State 0.5::Adjusting Motor Speed
    if(STATE == Adjust_Motor){
      // State Task
      DisplayMotorInfo();
      delay(100);
      uint8_t BUTTONS = LCD.readButtons();
      
      // State Transition
      if(BUTTONS & BUTTON_UP){
        // Serial.println("Increase");
        if(MotorSelected == 0)  
        BM_FR = BM_FR + 0.1;
        if(MotorSelected == 1)  
        CM_FR = CM_FR + 0.1;
        if(MotorSelected == 2)  
        RM_TS = RM_TS + 0.1;
      }
      if(BUTTONS & BUTTON_DOWN){
        // Serial.println("Decrease");
        if(MotorSelected == 0 && BM_FR > 0.1)  
        BM_FR = BM_FR - 0.1;
        if(MotorSelected == 1 && CM_FR > 0.1)  
        CM_FR = CM_FR - 0.1;
        if(MotorSelected == 2 && RM_TS > 0.1)  
        RM_TS = RM_TS - 0.1;
      }
      if(BUTTONS & BUTTON_SELECT){
        STATE = Display_Speed;
        LCD.clear();
      }
    }

  //  analogWrite(GLED, G_B);
  //  if(G_B <= 0){
  //    INC = 1;
  //  }
  //  else if(G_B >= 255){
  //    INC = -1;
  //    delay(100);
  //  }
  //  G_B = G_B + INC;
  }
  
  // State 1::Moving Forward - Printing
  while(DS_STATUS == HIGH && CM_LS_STATUS == HIGH && BM_LS_STATUS == HIGH && (SM_FW_LS_STATUS == HIGH || MODE == LOW)){
    DS_STATUS = digitalRead(DS);
    BM.runSpeed();
    CM.runSpeed();
    if(true){//(MODE == LOW){
      RM.runSpeed();
    }
    else{
      SM.runSpeed();
    }
    // Serial.println("Printing");
  }
  
  // State 2::Moving Forward - Jogging Stage
  while(FORWARD == HIGH && MODE == HIGH && SM_FW_LS_STATUS == HIGH){
    FORWARD = digitalRead(FW);
    SM.setSpeed(SM_JOG_SPEED * SM_RS_TS);
    SM.runSpeed();
    // Serial.println("Jogging");
  }
  
  // State 3::Moving Backward - Jogging Stage
  while(BACKWARD == HIGH && MODE == HIGH && SM_FW_LS_STATUS == HIGH && PM_LS_STATUS == HIGH){
    BACKWARD = digitalRead(BW);
    PM_LS_STATUS = digitalRead(PM_LS);
    SM.setSpeed(0 - SM_JOG_SPEED * SM_RS_TS);
    SM.runSpeed();
    // Serial.println("Jogging");
  }
}

void Update_CM_LS(){
  CM_LS_STATUS = digitalRead(CM_LS);
}

void Update_BM_LS(){
  BM_LS_STATUS = digitalRead(BM_LS);
}

void Update_SM_FW_LS(){
  SM_FW_LS_STATUS = digitalRead(SM_FW_LS);
}

void Update_SM_BW_LS(){
  SM_BW_LS_STATUS = digitalRead(SM_BW_LS);
}

void DisplaySpeed(){
  LCD.setCursor(0, 0);
  LCD.print("  BM   CM   RM  ");
  LCD.setCursor(1, 1);
  LCD.print(BM_FR);
  LCD.setCursor(5, 1);
  LCD.print("   ");
  LCD.setCursor(6, 1);
  LCD.print(CM_FR);
  LCD.setCursor(10, 1);
  LCD.print("   ");
  LCD.setCursor(11, 1);
  LCD.print(RM_TS);
}

void PromptError(){
  LCD.setCursor(0, 0);
  LCD.print("Printer Fault!");
  LCD.setCursor(0, 1);
  LCD.print("Press Select.");
}

void BlinkLCD(int MotorSelected){
  // Serial.println("Blinking");
  if(MotorSelected == 0){
    LCD.setCursor(1, 1);
    LCD.print("     ");
    delay(500);
    LCD.setCursor(1, 1);
    LCD.print(BM_FR);
    delay(250);
  }

    if(MotorSelected == 1){
    LCD.setCursor(6, 1);
    LCD.print("     ");
    delay(500);
    LCD.setCursor(6, 1);
    LCD.print(CM_FR);
    LCD.setCursor(10, 1);
    LCD.print(" ");
    delay(250);
  }

    if(MotorSelected == 2){
    LCD.setCursor(11, 1);
    LCD.print("     ");
    delay(500);
    LCD.setCursor(11, 1);
    LCD.print(RM_TS);
    delay(250);
  }
}

void DisplayMotorInfo(){
  LCD.setCursor(0, 0);
  if(MotorSelected == 0){
    LCD.print("BI Flow Rate:");
    LCD.setCursor(0, 1);
    LCD.print(BM_FR);
    LCD.setCursor(7, 1);
    LCD.print("uL/s");
  }
  if(MotorSelected == 1){
    LCD.print("CL Flow Rate:");
    LCD.setCursor(0, 1);
    LCD.print(CM_FR);
    LCD.setCursor(7, 1);
    LCD.print("uL/s");
  }
  if(MotorSelected == 2){
    LCD.print("Motion Speed:");
    LCD.setCursor(0, 1);
    LCD.print(RM_TS);
    LCD.setCursor(7, 1);
    LCD.print("mm/s");
  }
}
