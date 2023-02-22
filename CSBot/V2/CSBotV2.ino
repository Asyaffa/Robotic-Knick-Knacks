/*
    Written by Aqilla Rahman Musyaffa
    
    Based on:
    > https://circuitdigest.com/microcontroller-projects/record-and-play-3d-printed-robotic-arm-using-arduino
    > https://curiousscientist.tech/blog/20x4lcd-rotaryencoder-menu
    > https://github.com/XRobots/BallContraption/tree/main/IK%20Arm
*/

//Color Identifier Version
//Library for LCD 20x4
#include <LiquidCrystal_I2C.h>

//Library for Multitask
#include <elapsedMillis.h>

//Library for Servo
#include <Servo.h>

//Library for Rotary Encoder
#include "SimpleRotary.h"

//MEGA and Potentiometers Interface
#define pot1 A8
#define pot2 A9
#define pot3 A10
#define pot4 A11

//MEGA and Servos Interface
const byte J1Pin = 7;
const byte J2Pin = 6;
const byte J3Pin = 5;
const byte GPin = 4;

//MEGA and TCS230 Interface
const byte tcs230S0 = 8; 
const byte tcs230S1 = 9;
const byte tcs230S2 = 10; 
const byte tcs230S3 = 11;
const byte tcs230Out = 12;

//MEGA and Rotary Encoder Interface
const byte inputCLK = 37;   //Pin B
const byte inputDT = 40;    //Pin A
const byte inputSW = 28;

//MEGA and IR Sensor Interface
const byte irSensor = 3;

//Objects from Used Library
SimpleRotary rotary (inputDT, inputCLK, inputSW);
Servo joint1; 
Servo joint2; 
Servo joint3; 
Servo gripper;
elapsedMillis sensorTimer;
elapsedMillis lcdInterfaceTimer;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C (0x27, 20, 4);

//Minimum and Maximum Set Points for TCS230 (Calibration)
int minRed      = 19, maxRed    = 216;
int minGreen    = 17, maxGreen  = 214;
int minBlue     = 6, maxBlue   = 165;

//Color Pulse Values
int redPulse, greenPulse, bluePulse;

//Color RGB Values (255,255,255)
byte redValue, greenValue, blueValue;

//Color Identification Variable
String objectColor;

//Array to Store Servo Movements Based On the Color
int rSavedData [700]; 
int gSavedData [700];
int bSavedData [700];
int nSavedData [700];

//Index for Servo Movements Array
int rArrayIndex = 0;
int gArrayIndex = 0;
int bArrayIndex = 0;
int nArrayIndex = 0;

int rArrayTotal = 700;
int gArrayTotal = 700;
int bArrayTotal = 700;
int nArrayTotal = 700;
 
//Store Servo Position
int J1Pos, J2Pos, J3Pos, GPos;
int J1PreviousPos, J2PreviousPos, J3PreviousPos, GPreviousPos;
int J1CurrentPos, J2CurrentPos, J3CurrentPos, GCurrentPos;
int potValue1, potValue2, potValue3, potValue4;
int jointType, jointAction;

byte J1Default = 90;
byte J2Default = 90;
byte J3Default = 90;
byte GDefault = 90;

byte J1LowOffset = 10, J1HighOffset = 170;
byte J2LowOffset = 10, J2HighOffset = 170;
byte J3LowOffset = 10, J3HighOffset = 170;
byte GLowOffset = 10, GHighOffset = 170;

//Timing Variables to Control Multitask Timing
unsigned int sensorInterval = 1000;
unsigned int lcdInterfaceInterval = 1000;

//LCD and Robot Step Counter
byte modeCounter, stepCounter;

//Rotary Encoder Direction Value
byte currentDir;
byte previousDir = 0;

//IR Sensor Readings
byte irRead;
String irCaption;

//Mode Condition for LCD Menu
bool playModeSelected = false;
bool recordRModeSelected = false;
bool recordGModeSelected = false;
bool recordBModeSelected = false;
bool recordNModeSelected = false;
bool deleteModeSelected = false;

//Refresh Condition for LCD Menu
bool refreshLCD = true;
bool refreshLCDSelection= false;

void setup ()
{    
    Serial.begin (115200);
    
    //Initiate LCD 20x4
    lcd.init();
    lcd.backlight();
       
    //Declare the Pins to which Servos are Connected to
    joint1.attach (J1Pin); 
    joint2.attach (J2Pin); 
    joint3.attach (J3Pin); 
    gripper.attach (GPin);
    
    //Move Servos to Default Position
    joint1.write (J1Default); 
    joint2.write (J2Default);
    joint3.write (J3Default); 
    gripper.write (GDefault);

    //Declare Potentiometer Pins as Input
    pinMode (pot1, INPUT);
    pinMode (pot2, INPUT);
    pinMode (pot3, INPUT);
    pinMode (pot4, INPUT);

    //Declare Rotary Encoder Pins Mode
    rotary.setTrigger (HIGH);
    rotary.setDebounceDelay (5);
    rotary.setErrorDelay (250);

    //Declare IR Sensor Pin as Input
    pinMode (irSensor, INPUT);
    
    //Declare TCS230 Pins Mode
    pinMode (tcs230S0, OUTPUT); 
    pinMode (tcs230S1, OUTPUT);
    pinMode (tcs230S2, OUTPUT); 
    pinMode (tcs230S3, OUTPUT);
    pinMode (tcs230Out, INPUT);
        
    //Scale Color Pulse Frequency to 20%
    digitalWrite (tcs230S0, HIGH); 
    digitalWrite (tcs230S1, LOW);
    
    //Display Unchanged Characters in LCD
    lcdDisplay();
}

void loop()
{
    byte buttonStatus;
    buttonStatus = rotary.pushType (10000);
    rotaryCheck();
    if (buttonStatus == 1)
    {
        pushCheck();
    }
    if (refreshLCD == true)
    {
        if (playModeSelected == true || recordRModeSelected == true || recordGModeSelected == true ||recordBModeSelected == true || recordNModeSelected == true || deleteModeSelected == true)
        {
            //Do Nothing        
        }
        else
        {
            menuCaptionUpdate();
        }
    }
    if (refreshLCDSelection == true)
    {
        selectionUpdate();
        refreshLCDSelection = false;    
    }        
    if (sensorTimer >= sensorInterval)
    {
        //readColorPulse();
        readSensor();
        sensorTimer = 0;
    }
    if (lcdInterfaceTimer >= lcdInterfaceInterval)
    {
        lcdSensorUpdate();
        lcdInterfaceTimer = 0;
    }
}

//Set Up LCD Display on Start
void lcdDisplay()
{
    //LCD First Row (COLOR VALUE)
    lcd.setCursor (0, 0);
    lcd.print ("C: ");
    lcd.setCursor (3, 0);
    lcd.print ("R:");
    lcd.setCursor (9, 0);
    lcd.print ("G:");
    lcd.setCursor (15, 0);
    lcd.print ("B:");
    //LCD Second Row (COLOR IDENTIFICATION)
    lcd.setCursor (0, 1);
    lcd.print ("COLOR: ");
    //LCD Third Row (ITEM CONDITION)
    lcd.setCursor (0, 2);
    lcd.print ("ITEM:  ");
    //LCD Fourth Row (MODE MENU)
    lcd.setCursor (0, 3);
    lcd.print ("MODE:");
    lcd.setCursor (5, 3);
    lcd.print ("<  PLAY MODE  >");
}

//LCD Menu Logic
void rotaryCheck()
{
    if (playModeSelected == true)
    {
        playMode();
    }
    else if (recordRModeSelected == true)
    {
        redRecordMode();
    }
    else if (recordGModeSelected == true)
    {
        greenRecordMode();
    }
    else if (recordBModeSelected == true)
    {
        blueRecordMode();
    }
    else if (recordNModeSelected == true)
    {
        otherRecordMode();
    }
    else if (deleteModeSelected == true)
    {
        resetMode();
    }
    else
    {
        currentDir = rotary.rotate();
        if (currentDir == 1) //CW
        {
            if (modeCounter < 5)
            {
                modeCounter++;
            }
            else
            {
                modeCounter = 0;
            }
            refreshLCD = true;
        }
        else if (currentDir == 2) //CCW
        {
            if (modeCounter > 0)
            {
                modeCounter--;
            }
            else
            {
                modeCounter = 5;
            }
            refreshLCD = true;
        }
        else
        {
            refreshLCD = false;
        }
        previousDir = currentDir;
    }
}

//LCD Selected Menu
void pushCheck()
{
    switch (modeCounter)
    {
        case 0:
            playModeSelected = !playModeSelected;
        break;
        case 1:
            recordRModeSelected = !recordRModeSelected;
        break;
        case 2:
            recordGModeSelected = !recordGModeSelected;
        break;
        case 3:
            recordBModeSelected = !recordBModeSelected;
        break;
        case 4:
            recordNModeSelected = !recordNModeSelected;
        break;
        case 5:
            deleteModeSelected = !deleteModeSelected;
        break;
    }   
    refreshLCD = true;
    refreshLCDSelection = true;
}

//Change "<  PLAY MODE  >" to "<  RECORD  R  >" etc... on 3rd Row
void menuCaptionUpdate() 
{
    switch (modeCounter)
    {
        case 0:
            lcd.setCursor (5, 3);
            lcd.print ("<  PLAY MODE  >");
        break;
        case 1:
            lcd.setCursor (5, 3);
            lcd.print ("<  RECORD  R  >");
        break;
        case 2:
            lcd.setCursor (5, 3);
            lcd.print ("<  RECORD  G  >");
        break;
        case 3:
            lcd.setCursor (5, 3);
            lcd.print ("<  RECORD  B  >");
        break;
        case 4:
            lcd.setCursor (5, 3);
            lcd.print ("<  RECORD  N  >");
        break;
        case 5:
            lcd.setCursor (5, 3);
            lcd.print ("< RESET ARRAY >");
        break;
    }
    
}

//Change "< ... >" to "[ ... ]" on 3rd Row
void selectionUpdate() 
{
    if (playModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  PLAY MODE  ]");
    }
    if (recordRModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  RECORD  R  ]");
    }
    if (recordGModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  RECORD  G  ]");
    }
    if (recordBModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  RECORD  B  ]");
    }
    if (recordNModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  RECORD  N  ]");
    }
    if (deleteModeSelected == true)
    {
        lcd.setCursor (5, 3);
        lcd.print ("[  SET TO  0  ]");
    } 
}

//Read Every Color Pulse Width
void readColorPulse() 
{
    redPulse    = getColorPulse (LOW, LOW);
    //Serial.print (redPulse);
    //Serial.print ("\t");
    greenPulse  = getColorPulse (HIGH, HIGH);
    //Serial.print (greenPulse);
    //Serial.print ("\t");
    bluePulse   = getColorPulse (LOW, HIGH);
    //Serial.println (bluePulse);
}

//Read Item Condition
void readSensor() 
{
    redValue    = getColorValue (LOW, LOW, minRed, maxRed);
    greenValue  = getColorValue (HIGH, HIGH, minGreen, maxGreen);
    blueValue   = getColorValue (LOW, HIGH, minBlue, maxBlue);
    irRead      = digitalRead (irSensor);
    switch (irRead)
    {
        case 0:
            irCaption = " Available ";
        break;
        case 1:
            irCaption = "Unavailable";
        break;
    }
    if (redValue > 235 && greenValue > 235 && blueValue > 235)
    {
        objectColor = "  WHITE  "; 
    }
    else if (redValue < 100 && greenValue < 100 && blueValue < 100)
    {
        objectColor = "  BLACK  ";
    }
    else if (redValue > 240 && greenValue > 200 && blueValue < 200)
    {
        objectColor = "  ORANGE ";
    }
    else if (redValue > 230 && redValue > greenValue && redValue > blueValue)
    {
        objectColor = "  RED    ";
    }
    else if (greenValue > 190 && greenValue > redValue && greenValue > blueValue)
    {
        objectColor = "  GREEN  ";
    }
    else if (blueValue > 180 && blueValue > redValue && blueValue > greenValue)
    {
        objectColor = "  BLUE   ";
    }
    else
    {
        objectColor = " UNKNOWN ";
    }
}

//Update Item Condition Readings on 1st, 2nd, and 3rd Row
void lcdSensorUpdate() 
{
    //1st Row
    lcd.setCursor (5, 0);
    lcd.print ("   ");
    lcd.setCursor (5, 0);
    lcd.print (redValue);
    lcd.setCursor (11, 0);
    lcd.print ("   ");
    lcd.setCursor (11, 0);
    lcd.print (greenValue);
    lcd.setCursor (17, 0);
    lcd.print ("   ");
    lcd.setCursor (17, 0);
    lcd.print (blueValue);
    //2nd Row
    lcd.setCursor (7, 1);
    lcd.print (objectColor);
    //3rd Row
    lcd.setCursor (7, 2);
    lcd.print (irCaption);    
}

//Function to Measure LOW Pulse Width
int getColorPulse (int valueA, int valueB)
{
    digitalWrite (tcs230S2, valueA);
    digitalWrite (tcs230S3, valueB);
    int PW;
    PW = pulseIn (tcs230Out, LOW);
    return PW;
}

//Function to Measure Color RGB Value
int getColorValue (int valueA, int valueB, int lowOffset, int highOffset)
{
    digitalWrite (tcs230S2, valueA);
    digitalWrite (tcs230S3, valueB);
    int PW, rgbValue;
    PW = pulseIn (tcs230Out, LOW);
    rgbValue = map (PW, lowOffset, highOffset, 255, 0);
    return rgbValue;
}

//Empty Each Movement Array 
void resetMode()
{
    emptyArray (rSavedData);
    emptyArray (gSavedData);
    emptyArray (bSavedData);
    emptyArray (nSavedData);
    
    rArrayIndex = 0;
    gArrayIndex = 0;
    bArrayIndex = 0;
    nArrayIndex = 0;

    rArrayTotal = 700;
    gArrayTotal = 700;
    bArrayTotal = 700;
    nArrayTotal = 700;
}

//Empty Array Function
void emptyArray (int *savedData)
{
    memset (savedData, 0, sizeof (savedData));
}

//Function to Move Robot with Sequencer
void playMode()
{
    if (irRead == 0 && stepCounter == 0)
    {
        stepCounter = 1;
    }  
    if (stepCounter == 1)
    {
        if (redValue > 230 && redValue > greenValue && redValue > blueValue)
        {
            stepCounter = 2;
            readArray (rSavedData, rArrayIndex);
        }
        else if (greenValue > 190 && greenValue > redValue && greenValue > blueValue)
        {
            stepCounter = 3;
            readArray (gSavedData, gArrayIndex);
        }
        else if (blueValue > 180 && blueValue > redValue && blueValue > greenValue)
        {
            stepCounter = 4;
            readArray (bSavedData, bArrayIndex);
        }
        else
        {
            stepCounter = 5;
            readArray (nSavedData, nArrayIndex);
        }
    }
}

//Function to Move Each Joint
void readArray (int *savedData, int index)
{
    for (int playAction = 0; playAction < index; playAction++)
    {
        jointType = savedData [playAction] / 1000;
        jointAction = savedData [playAction] % 1000;
        switch (jointType)
        {
            case 0:
                joint1.write (jointAction);
            break;
            case 1:
                joint2.write (jointAction);
            break;
            case 2:
                joint3.write (jointAction);
            break;
            case 3:
                gripper.write (jointAction);
            break;
        }
        delay (50);
    }
    stepCounter = 0; 
}

//One of Solution to Remove Jitter Problem on Servo
void jitterRemoval()
{
    readPot();
    J1PreviousPos = J1Pos;
    J2PreviousPos = J2Pos;
    J3PreviousPos = J3Pos;
    GPreviousPos = GPos;
    readPot();
}

//Function to Map Potentiometer Readings into Servo Degrees
void readPot()
{
    potValue1 = analogRead (pot1);
    potValue2 = analogRead (pot2);
    potValue3 = analogRead (pot3);
    potValue4 = analogRead (pot4);   
    J1Pos = map (potValue1, 0, 1023, J1LowOffset, J1HighOffset);
    J2Pos = map (potValue2, 0, 1023, J2LowOffset, J2HighOffset);
    J3Pos = map (potValue3, 0, 1023, J3LowOffset, J3HighOffset);
    GPos = map (potValue4, 0, 1023, GLowOffset, GHighOffset);
}

//Function to Record Red Color Movement
void redRecordMode() 
{
    jitterRemoval();
    int arrayLeft = rArrayTotal - rArrayIndex;    
    if (J1PreviousPos == J1Pos)
    {
        joint1.write (J1Pos);
        if (J1CurrentPos != J1Pos)
        {
            rSavedData [rArrayIndex] = J1Pos + 0; 
            rArrayIndex++;
        }
        J1CurrentPos = J1Pos;
    }
    
    if (J2PreviousPos == J2Pos)
    {
        joint2.write (J2Pos);
        if (J2CurrentPos != J2Pos)
        {
            rSavedData [rArrayIndex] = J2Pos + 1000; 
            rArrayIndex++;
        }
        J2CurrentPos = J2Pos;
    }

    if (J3PreviousPos == J3Pos)
    {
        joint3.write (J3Pos);
        if (J3CurrentPos != J3Pos)
        {
            rSavedData [rArrayIndex] = J3Pos + 2000; 
            rArrayIndex++;
        }
        J3CurrentPos = J3Pos;
    }

    if (GPreviousPos == GPos)
    {
        gripper.write (GPos);
        if (GCurrentPos != GPos)
        {
            rSavedData [rArrayIndex] = GPos + 3000; 
            rArrayIndex++;
        }
        GCurrentPos = GPos;
    }
}

//Function to Record Green Color Movement
void greenRecordMode() 
{
    jitterRemoval();
    int arrayLeft = gArrayTotal - gArrayIndex; 
    if (J1PreviousPos == J1Pos)
    {
        joint1.write (J1Pos);
        if (J1CurrentPos != J1Pos)
        {
            gSavedData [gArrayIndex] = J1Pos + 0; 
            gArrayIndex++;
        }
        J1CurrentPos = J1Pos;
    }
    
    if (J2PreviousPos == J2Pos)
    {
        joint2.write (J2Pos);
        if (J2CurrentPos != J2Pos)
        {
            gSavedData [gArrayIndex] = J2Pos + 1000; 
            gArrayIndex++;
        }
        J2CurrentPos = J2Pos;
    }

    if (J3PreviousPos == J3Pos)
    {
        joint3.write (J3Pos);
        if (J3CurrentPos != J3Pos)
        {
            gSavedData [gArrayIndex] = J3Pos + 2000; 
            gArrayIndex++;
        }
        J3CurrentPos = J3Pos;
    }

    if (GPreviousPos == GPos)
    {
        gripper.write (GPos);
        if (GCurrentPos != GPos)
        {
            gSavedData [gArrayIndex] = GPos + 3000; 
            gArrayIndex++;
        }
        GCurrentPos = GPos;
    }
}

//Function to Record Blue Color Movement
void blueRecordMode() 
{
    jitterRemoval();
    int arrayLeft = bArrayTotal - bArrayIndex;
    if (J1PreviousPos == J1Pos)
    {
        joint1.write (J1Pos);
        if (J1CurrentPos != J1Pos)
        {
            bSavedData [bArrayIndex] = J1Pos + 0; 
            bArrayIndex++;
        }
        J1CurrentPos = J1Pos;
    }
    
    if (J2PreviousPos == J2Pos)
    {
        joint2.write (J2Pos);
        if (J2CurrentPos != J2Pos)
        {
            bSavedData [bArrayIndex] = J2Pos + 1000; 
            bArrayIndex++;
        }
        J2CurrentPos = J2Pos;
    }

    if (J3PreviousPos == J3Pos)
    {
        joint3.write (J3Pos);
        if (J3CurrentPos != J3Pos)
        {
            bSavedData [bArrayIndex] = J3Pos + 2000; 
            bArrayIndex++;
        }
        J3CurrentPos = J3Pos;
    }

    if (GPreviousPos == GPos)
    {
        gripper.write (GPos);
        if (GCurrentPos != GPos)
        {
            bSavedData [bArrayIndex] = GPos + 3000; 
            bArrayIndex++;
        }
        GCurrentPos = GPos;
    }
}

//Function to Record Movement for Color Except RGB
void otherRecordMode() 
{
    jitterRemoval();
    int arrayLeft = nArrayTotal - nArrayIndex;
    if (J1PreviousPos == J1Pos)
    {
        joint1.write (J1Pos);
        if (J1CurrentPos != J1Pos)
        {
            nSavedData [nArrayIndex] = J1Pos + 0; 
            nArrayIndex++;
        }
        J1CurrentPos = J1Pos;
    }
    
    if (J2PreviousPos == J2Pos)
    {
        joint2.write (J2Pos);
        if (J2CurrentPos != J2Pos)
        {
            nSavedData [nArrayIndex] = J2Pos + 1000; 
            nArrayIndex++;
        }
        J2CurrentPos = J2Pos;
    }

    if (J3PreviousPos == J3Pos)
    {
        joint3.write (J3Pos);
        if (J3CurrentPos != J3Pos)
        {
            nSavedData [nArrayIndex] = J3Pos + 2000; 
            nArrayIndex++;
        }
        J3CurrentPos = J3Pos;
    }

    if (GPreviousPos == GPos)
    {
        gripper.write (GPos);
        if (GCurrentPos != GPos)
        {
            nSavedData [nArrayIndex] = GPos + 3000; 
            nArrayIndex++;
        }
        GCurrentPos = GPos;
    }
}
