/*
  ---------------------------------------------------------
  PROGRAM INFORMATION
  ---------------------------------------------------------
  Bob Jones Patriot Racing Motor Dynamometer
  Copyright 2016-2022, All Rights reserved
  This code cannot be used outside of Bob Jones High School
  Code for Teensy 3.2
  ---------------------------------------------------------
  COMPILE INSTRUCTIONS
  ---------------------------------------------------------
  Compile Speed:  72MHz
  Optimize:       Smallest code

  rev   author      date        code
  4.6   kasprzak    9-2022      added support for dual load cells
  4.7   kasprzak    11-2022     moved all setups to menu system


*/

// #define DEBUG

#include <SPI.h>
#include <EEPROM.h>          // standard library that ships with Teensy
#include <ILI9341_t3.h>
#include <font_Arial.h>      // custom fonts that ships with ILI9341_t3.h
#include <font_ArialBold.h>      // custom fonts that ships with ILI9341_t3.h
#include <avr/io.h>          // standard library that ships with Teensy
#include <avr/interrupt.h>   // standard library that ships with Teensy
#include <SdFat.h>           // SdFat library for more SD functions
#include "HX711.h"           // load cell lib
#include "UTouch.h"          // touchscreen lib
#include <TimeLib.h>
#include <ILI9341_t3_Controls.h>
#include <FlickerFreePrint.h>// library to draw w/o flicker
#include "Dyno_Icons.h"
#include "Colors.h"
#include <ILI9341_t3_PrintScreen_SD.h>
#include "ILI9341_t3_Menu.h"

/////////////////////////////////////////////////////////////////////////
#define CODE_VERSION "4.7"
/////////////////////////////////////////////////////////////////////////

#define LIMIT_L 1000
#define LIMIT_H 3000

#define LEFT 0
#define RIGHT 1

#define CHINESE_MOTOR 1
#define UK_MOTOR 0

#define F_A20 Arial_20
#define F_A16 Arial_16
#define F_A12B Arial_12_Bold
#define F_A12 Arial_12
#define F_A08 Arial_8

#define EFF_COLOR C_GREEN
#define TOR_COLOR C_CYAN
#define AMP_COLOR C_PINK
#define VOLT_COLOR C_DKYELLOW
#define POINT_DIA 1
#define BTN_RADIUS 4
#define BTN_THICK 4
#define TIME_HEADER  "T"

#define NTC_A  3.354016E-03  // from the data sheet
#define NTC_B  2.569850E-04  // from the data sheet
#define NTC_C  2.620131E-06  // from the data sheet
#define NTC_D  6.383091E-08  // from the data sheet
#define NTC_R1  9800.0       // resistor for thermsitor voltage divider

// timers for button presses
#define NO_PRESS      0
#define SHORT_PRESS   60
#define LONG_PRESS    1000
#define DEBOUNCE      50

// pin defs
#define BUZZ_PIN  A7     // alert buzzer
#define RPM_PIN A0       // pin for the RPM
#define TH_PIN  A1       // thermisto measurement pin
#define AM_PIN  A2       // amp sensor pin
#define VM_PIN  A9       // voltage divider input
#define SDCS_PIN A8      // CS for SD card
#define tr1 9800.0       // resistor for thermsitor voltage divider
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10

// variables for the locations of the keypad buttons
#define BUTTON_X 90
#define BUTTON_Y 60
#define BUTTON_W 70
#define BUTTON_H 30
#define BUTTON_SPACING_X 5
#define BUTTON_SPACING_Y 5
#define BUTTON_TEXTSIZE 2

#define LOADCELL_DOUT_PIN  A5
#define LOADCELL_SCK_PIN  A6
//Resistor for Thermistor Voltage Divider
#define tr1 9800.0                            // resistor for thermsitor voltage divider
#define R1  9850                              // resistor for the voltage divider
#define R2  984

int BtnX, BtnY;

byte np = 0;
char dn[12];

char KeyPadBtnText[12][5] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "-", "0", "." };
char buf[50];
bool StartTest = false;

int LC_year = 2022, LC_month = 10, LC_day = 16, SC_year = 2022, SC_month = 10, SC_day = 16;
int LC_TestsSinceCal = 0;
int SC_TestsSinceCal = 0;
int LoadCellReads = 1;
int Update = 100;            // milliseconds between calculations and screen refresh and SD write
long tr2 = 0;
float LoadForce = 0.0;
float Torque = 0.0;
float MPower = 0.0;
float LoadMass = 0.0;
float MotorEff = 0.0;
byte RampDownOnly = true;
byte MotorType = 0;
byte MotorDirection = 0;
bool redraw = true;
int MotorNumber;
byte LineThickness = 2;
float dAmps[LIMIT_H - LIMIT_L] ;
float dTorque[LIMIT_H - LIMIT_L] ;
float dEffeciency[LIMIT_H - LIMIT_L] ;
float dVolts[LIMIT_H - LIMIT_L] ;
float dLoad[LIMIT_H - LIMIT_L] ;

float LastAmps = 0;
float LastTorque = 0;
float LastEffeciency = 0;
float LastVolts = 0;
float LastLoad = 0;

// setup variables

float ForceArm = .08255;        // size in meters
float LoadCellCal = -473.20;    // sum of both batteries at 10.5 volt mark
float LoadCellOffset = 136240.0;

float VDCompensation = 13.0;    // Vin is comming through diodes before vvoltage divider--can't use standard equation.
int RPMDebounceTime = 20;       // debounce time for speed sensor in millis()
float VMOffset = 0.75;
byte m, h, s;
byte col = 240;                             // location for the setup variables
byte row = 20;                              // height of each row in setup
float VMid = 0.396;                           // offset for current sensor straignt from data sheet
float mVPerAmp = 26.4;                      // sensitivity for current sensor straignt from data sheet
char FileName[15] = "MND_TTT_RR.CSV";        // SD card filename
float thVolts = 0.0, TempF = 0.0, TempK = 0.0;    // computed temp values
float TempOffset = 0.0;
volatile unsigned long PulseStartTime = 0, PulseEndTime = 0, PulseCurrentTime = 0, PulseDebounce = 0, ErrorCount = 0;
volatile float PulseCount = 0.0;
volatile float PulseTime = 0;       // Counter for the hall effect sensor
volatile unsigned long Counter = 0; // the number of measurements between each display

unsigned long WRPM = 0;             // wheel rpm (measured)
unsigned long PreviousRPM = 15000;

float vVolts = 0.0, Volts = 0.0;    // computed Volts
float aVolts = 0.0, Amps = 0.0;     // computed Amps
float EPower = 0.0;                 // computed EPower
float Energy = 0.0;                 // computed Energy basically EPower x time slice

// resistor for the voltage divider

byte Point = 0;                     // Counter for the data Point
bool isSD = false;                  // is SD card working
byte FileCount = 0;

int top, wide, high, Left;          // holders for screen coordinate drawing
bool KeepIn;
byte b;
// time variables
unsigned long curtime = 0;            // current time actual - startup time
float TempNum;              // some temp number

int pEff, pTorque, pAmps, pVolts;
int i = 0;                  // just some storage variables
const char *OffOnItems[] =   {"Off", "On"};
const char *ReadText[] =  {"Read"};
const char *RecordTypeText[] =  {"Down Only", "Up/Down"};

int MenuOption1 = 0, MenuOption2 = 0, MenuOption3 = 0, MenuOption4 = 0, MenuOption5 = 0;
int MenuOption6 = 0, MenuOption7 = 0, MenuOption8 = 0, MenuOption9 = 0, MenuOption10 = 0;
int MenuOption11 = 0, MenuOption12 = 0, MenuOption13 = 0;

ILI9341_t3 Display(TFT_CS, TFT_DC, 240, 320, TFT_RST); //, 11, 13, TFT_MISO);

// create the // Touch screen object
UTouch  Touch( 6, 5, 4, 3, 2);

EditMenu OptionMenu(&Display, true);

Button ProfileBtn(&Display);
Button SensorsBtn(&Display);
Button ConnectionBtn(&Display);

Button MotorNumberBtn(&Display);
Button MotorTypeBtn(&Display);
Button KeyPadBtn[12](&Display);
Button StartBtn(&Display);
Button StartTestBtn(&Display);
Button StopBtn(&Display);
Button ReRunBtn(&Display);
Button SetTareBtn(&Display);

Button BackBtn(&Display);
Button DoneBtn(&Display);
Button MenuDoneBtn(&Display);

SdFat sd;
SdFile DataFile;
SdFile Root;
SdFile FileFile;

HX711 scale;

time_t RTCTime;

elapsedMillis CalTime;
elapsedMillis UpdateTime;

CGraph Graph(&Display, 20, 210, 260, 160, LIMIT_L, LIMIT_H, 200, 0, 100, 10);

FlickerFreePrint<ILI9341_t3> ffCalLoad(&Display, C_WHITE, C_BLACK);

FlickerFreePrint<ILI9341_t3> ffPoint(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffTorque(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffTemp(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffVolts(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffAmps(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffRPM(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffMass(&Display, C_WHITE, C_BLACK);
FlickerFreePrint<ILI9341_t3> ffFileName(&Display, C_WHITE, C_BLACK);

/**********************************************************************************************

  program setup

***********************************************************************************************/
void setup() {

  Serial.begin(9600);

  pinMode(RPM_PIN, INPUT); // must have external 4k7 pullup
  pinMode(BUZZ_PIN, OUTPUT);

  delay(5);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // setup display
  Display.begin();

  asm(".global _printf_float");

  analogWriteFrequency(BUZZ_PIN, 1500);

  for (i = 0; i < 2; i ++) {
    analogWrite(BUZZ_PIN, 100);
    delay(100);
    analogWrite(BUZZ_PIN, 0);
    delay(100);
  }

  GetParameters();

  // set orientation--using math to flip display upside down if display is mounted that way
  Display.setRotation(1);
  Display.fillScreen(C_BLACK);
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
  RTCTime = processSyncMessage();

  if (RTCTime != 0) {
    setTime(RTCTime);
    Teensy3Clock.set(RTCTime); // set the RTC
  }
  //setTime(hours, minutes, seconds, days, months, years);
  // setTime(19, 58, 0, 1, 12, 2020);
  // Teensy3Clock.set(now());

  Display.fillRect(10, 10, 300, 10, C_RED);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A20);

  Display.setCursor(160 - Display.measureTextWidth("PATRIOT RACING") / 2, 35);
  Display.print(F("PATRIOT RACING"));

  Display.setCursor(160 - Display.measureTextWidth("DYNAMOMETER") / 2, 70);
  Display.print(F("DYNAMOMETER"));

  Display.setCursor(60, 105);
  strcpy(buf, "Version: ");
  strcat(buf, CODE_VERSION);
  Display.setCursor(160 - Display.measureTextWidth(buf) / 2, 105);
  Display.print(buf);

  sprintf(dn, "%01d/%01d/%04d", month(), day(), year());
  Display.setCursor(160 - Display.measureTextWidth(dn) / 2, 140);
  Display.print(dn);

  Display.fillRect(10, 180, 300, 10, C_BLUE);

  Display.fillRoundRect(20, 200, 280, 30, 8, C_WHITE);
  Display.fillRoundRect(20, 200, 20, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Building menus"));

  delay(50);
  // fire up the touch display
  Touch.InitTouch(PORTRAIT);
  Touch.setPrecision(PREC_EXTREME);

  ProfileBtn.init (  160, 85, 250, 55, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Start Test", 0, 0 , F_A20);
  SensorsBtn.init(   160, 145, 250, 55, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Sensors", 0, 0, F_A20);
  ConnectionBtn.init(160, 205, 250, 55, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Wiring", 0, 0, F_A20);

  OptionMenu.init(C_WHITE, C_BLACK, C_WHITE, C_DKBLUE, C_WHITE, C_DKRED, 200, 30, 5, "SENSORS", F_A16, F_A20);

  MenuOption1 = OptionMenu.addNI("Brake Arm", ForceArm, 0.07, 0.09 , .001, 3);
  MenuOption2 = OptionMenu.addNI("Weight 0 kg", 0,    0, sizeof(ReadText) / sizeof(ReadText[0]), 1, 0, ReadText);
  MenuOption3 = OptionMenu.addNI("Weight 1 kg", 0, 0, sizeof(ReadText) / sizeof(ReadText[0]), 1, 0, ReadText);
  MenuOption4 = OptionMenu.addNI("Reads", LoadCellReads, 1, 10, 1, 0);
  MenuOption5 = OptionMenu.addNI("Update", Update, 50, 3000, 50, 0);
  MenuOption6 = OptionMenu.addNI("Debounce", RPMDebounceTime, 5, 150, 5, 0);
  MenuOption7 = OptionMenu.addNI("Volt Offset", VMOffset, .4, .8, .01, 2);
  MenuOption8 = OptionMenu.addNI("Volt slope", VDCompensation, 9, 12, .01, 2);
  MenuOption9 = OptionMenu.addNI("xxx", VMid, .4, .8, .001, 3);
  MenuOption10 = OptionMenu.addNI("Amp slope", mVPerAmp, 24.0, 27.0, .1, 1);
  MenuOption11 = OptionMenu.addNI("Temp bias", TempOffset, -10.0, 10.0, 0.1, 1);
  MenuOption12 = OptionMenu.addNI("Record type", RampDownOnly, 0, sizeof(RecordTypeText) / sizeof(RecordTypeText[0]), 1, 0,  RecordTypeText);
  MenuOption13 = OptionMenu.addNI("Line thickness", LineThickness, 1, 4, 1, 0);

  OptionMenu.setItemTextMargins(5, 6, 5);
  OptionMenu.setMenuBarMargins(5, 315, 3, 1);

  StopBtn.init(     278, 20, 80, 35, C_DKRED, C_RED, C_WHITE, C_DKGREY, "Stop", 0, 0, F_A16);
  ReRunBtn.init(    278, 20, 80, 35, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "New", 0, 0, F_A16);
  StartTestBtn.init(278, 20, 80, 35, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Start", 0, 0, F_A16);

  BackBtn.init(BUTTON_X + (2 * (BUTTON_W + BUTTON_SPACING_X)), BUTTON_Y - BUTTON_H - BUTTON_SPACING_Y, BUTTON_W, BUTTON_H, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "<", 0, 0, F_A16);
  DoneBtn.init(BUTTON_X + (BUTTON_W + BUTTON_SPACING_X), BUTTON_SPACING_Y + BUTTON_SPACING_Y + BUTTON_Y + (4 * (BUTTON_H + BUTTON_SPACING_Y)), 3 * (BUTTON_W + BUTTON_SPACING_X), BUTTON_H, C_BLACK, C_GREEN, C_BLACK, C_DKGREY, "Done", 0, 0, F_A16);

  MenuDoneBtn.init(278, 25, 80, 40, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Done", 0, 0, F_A16);

  SetTareBtn.init( 80, 210, 140, 50, C_DKRED, C_RED, C_BLACK, C_DKGREY, "Set Tare", 0, 0, F_A20);
  StartBtn.init(  240, 210, 140, 50, C_DKGREEN, C_GREEN, C_BLACK, C_DKGREY, "Start", 0, 0, F_A20);

  MotorNumberBtn.init(240, 80, 100, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Number", 0, 0, F_A16);
  MotorTypeBtn.init(  240, 140, 100, 50, C_DKGREY, C_WHITE, C_BLACK, C_DKGREY, "Mfg.", 0, 0, F_A16);

  // create KeyPadBtn
  for (row = 0; row < 4; row++) {
    for (col = 0; col < 3; col++) {
      KeyPadBtn[col + row * 3].init(BUTTON_X + col * (BUTTON_W + BUTTON_SPACING_X),
                                    BUTTON_Y + BUTTON_SPACING_Y + BUTTON_SPACING_Y + (row * (BUTTON_H + BUTTON_SPACING_Y)),
                                    BUTTON_W, BUTTON_H, C_BLACK, C_WHITE, C_BLACK, C_DKGREY,
                                    KeyPadBtnText[col + row * 3], 0, 0, F_A16);
      KeyPadBtn[col + row * 3].setCornerRadius(BTN_RADIUS);
    }
  }

  Graph.init("", "RPM x100", "", C_WHITE, C_MDGREY, C_YELLOW, C_BLACK, C_BLACK, F_A12, F_A08);
  Graph.showTitle(false);
  Graph.showLegend(true);
  Graph.drawLegend(LOCATION_BOTTOM);

  pEff = Graph.add("Eff.", EFF_COLOR);
  pTorque = Graph.add("Torque x10", TOR_COLOR);
  pAmps = Graph.add("Amps", AMP_COLOR);
  pVolts = Graph.add("Volts", VOLT_COLOR);

  Graph.setMarkerSize(pEff, POINT_DIA);
  Graph.setMarkerSize(pTorque, POINT_DIA);
  Graph.setMarkerSize(pAmps, POINT_DIA);
  Graph.setMarkerSize(pVolts, POINT_DIA);

  Graph.setXTextOffset(5);
  Graph.setYTextOffset(18);
  Graph.setXTextScale(.01);


  ProfileBtn.setCornerRadius(BTN_RADIUS);
  SensorsBtn.setCornerRadius(BTN_RADIUS);
  ConnectionBtn.setCornerRadius(BTN_RADIUS);
  MotorNumberBtn.setCornerRadius(BTN_RADIUS);
  MotorTypeBtn.setCornerRadius(BTN_RADIUS);
  StartBtn.setCornerRadius(BTN_RADIUS);
  StartTestBtn.setCornerRadius(BTN_RADIUS);
  StopBtn.setCornerRadius(BTN_RADIUS);
  ReRunBtn.setCornerRadius(BTN_RADIUS);
  SetTareBtn.setCornerRadius(BTN_RADIUS);
  BackBtn.setCornerRadius(BTN_RADIUS);
  DoneBtn.setCornerRadius(BTN_RADIUS);
  MenuDoneBtn.setCornerRadius(BTN_RADIUS);

  ProfileBtn.setBorderThickness(BTN_THICK);
  SensorsBtn.setBorderThickness(BTN_THICK);
  ConnectionBtn.setBorderThickness(BTN_THICK);
  MotorNumberBtn.setBorderThickness(BTN_THICK);
  MotorTypeBtn.setBorderThickness(BTN_THICK);
  StartBtn.setBorderThickness(BTN_THICK);
  StartTestBtn.setBorderThickness(BTN_THICK);
  StopBtn.setBorderThickness(BTN_THICK);
  ReRunBtn.setBorderThickness(BTN_THICK);
  SetTareBtn.setBorderThickness(BTN_THICK);
  BackBtn.setBorderThickness(BTN_THICK);
  DoneBtn.setBorderThickness(BTN_THICK);
  MenuDoneBtn.setBorderThickness(BTN_THICK);


  Display.fillRoundRect(20, 200, 50, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Starting SD reader"));
  // setup SD card
  delay(5);
  isSD = sd.begin(SDCS_PIN, SD_SCK_MHZ(20));
  Display.fillRoundRect(20, 200, 150, 30, 8, C_MDGREY);
  // setup analog read resolutions
  analogReadRes(12);
  analogReadAveraging(16);

  if (isSD == false) {
    Display.setFont(F_A20);
    Display.fillScreen(C_BLACK);
    Display.setTextColor(C_RED, C_BLACK);
    Display.setCursor(20, 50);
    Display.print(F("NO SD CARD")) ;
    Serial.println("no card");
    while (1) {}

  }

  Display.fillRoundRect(20, 200, 200, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Starting RPM sensor"));

  // setup speed sensor
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), ISR_MS, FALLING);
  Display.fillRoundRect(20, 200, 280, 30, 8, C_MDGREY);
  Display.setCursor(30, 203);
  Display.setTextColor(C_BLACK, C_MDGREY);
  Display.print(F("Setup done"));
  delay(1500);

  ShowInstructions();

  GetDirection();

  MainMenu();

  GetFileName();

  PrepTest();

}

/***********************************************************************************************

  Main program loop

***********************************************************************************************/
void loop() {

  // let's back out the setup time to make stuff start at 0
  curtime = millis() ;

  // increment the Counter used for averaging
  // basically we are going to read values until time to display, then compute averages
  Counter++;

  // measure battery Volts
  vVolts =  vVolts + (analogRead(VM_PIN));

  // measure battery Amps
  aVolts =  aVolts + (analogRead(AM_PIN)) ;

  if (Touch.dataAvailable()) {

    ProcessTouch();

    if (PressIt(StartTestBtn) == true) {

      Display.setFont(F_A20);
      Display.setCursor(10 , 10 );
      Display.setTextColor(C_WHITE, C_DKBLUE);
      Display.fillRect(0, 0, 320, 40, C_DKBLUE);
      Display.print(F("Begin testing"));
      StartTest = true;
      StartTestBtn.disable();
      StopBtn.enable();
      StopBtn.draw();
      BtnX = 0;
      BtnY = 0;
    }
    else if (PressIt(ReRunBtn) == true) {
      MainMenu();
      GetFileName();
      PrepTest();
    }
    else if (PressIt(StopBtn) == true) {
      StopBtn.disable();
      StartTestBtn.disable();
      EndTest();
      ReRunBtn.enable();
      ReRunBtn.draw();
    }
  }

  if (UpdateTime >= (unsigned long) Update ) {

    ComputeData();

    DisplayData();

    if ((WRPM < LIMIT_L) && (StartTest)) {
      StopBtn.disable();
      StartTestBtn.disable();

      EndTest();

      ReRunBtn.enable();
      ReRunBtn.draw();

    }

    if (StartTest) {

      if ((WRPM >= LIMIT_L) && (WRPM <= LIMIT_H) && (MotorEff >= 0.0)) {

        // attempt to keep hysteresis out of the picture
        // and since vibrations bounce unit around only
        // record data on the decrease
        if (RampDownOnly) {
          if (WRPM <= PreviousRPM) {
            PreviousRPM = WRPM;
            dAmps[WRPM - LIMIT_L] = Amps;
            dTorque[WRPM - LIMIT_L] = Torque;
            dEffeciency[WRPM - LIMIT_L] = 100.0 * MotorEff;
            dVolts[WRPM - LIMIT_L] = Volts;
            dLoad[WRPM - LIMIT_L] = LoadMass;

            LastAmps = Amps;
            LastTorque = Torque;
            LastEffeciency = 100.0 * MotorEff;
            LastVolts = Volts;
            LastLoad = LoadMass;

            // plot the data
            Graph.setX(WRPM);
            Graph.plot(pEff, (int) (100.0 * MotorEff));
            Graph.plot(pTorque, Torque * 10.0);
            Graph.plot(pAmps, Amps);
            Graph.plot(pVolts, Volts);
            Point++;
          }
        }
        else {

          dAmps[WRPM - LIMIT_L] = Amps;
          dTorque[WRPM - LIMIT_L] = Torque;
          dEffeciency[WRPM - LIMIT_L] = 100.0 * MotorEff;
          dVolts[WRPM - LIMIT_L] = Volts;
          dLoad[WRPM - LIMIT_L] = LoadMass;

          LastAmps = Amps;
          LastTorque = Torque;
          LastEffeciency = 100.0 * MotorEff;
          LastVolts = Volts;
          LastLoad = LoadMass;

          // plot the data
          Graph.setX(WRPM);
          Graph.plot(pEff, (int) (100.0 * MotorEff));
          Graph.plot(pTorque, Torque * 10.0);
          Graph.plot(pAmps, Amps);
          Graph.plot(pVolts, Volts);
          Point++;
        }
      }
    }

    // reset the counters
    vVolts = 0.0;
    aVolts = 0.0;
    thVolts = 0.0;
    Counter = 0;
    UpdateTime = 0;
    WRPM = 0;
    PulseTime = 0;
    PulseCount = 0;
    ErrorCount = 0;

  }

}

/***********************************************************************************************

  Main calculation code

***********************************************************************************************/

void ComputeData() {

  if (PulseCount > 2) {
    PulseTime = (PulseEndTime - PulseStartTime) / (PulseCount - 1.0);
  }

  if (PulseTime > 0) {
    WRPM = (unsigned long) (1000.0 * 60.0) /  (PulseTime );
  }

  // get the battey voltage
  vVolts = vVolts / Counter;
  vVolts =  vVolts / (4096.0 / 3.3) ;
  Volts = (vVolts * VDCompensation) + VMOffset;

  // get current draw
  aVolts = aVolts / Counter;
  aVolts =  aVolts / (4096.0 / 3.3);

  Amps = (aVolts - VMid) / (mVPerAmp / 1000.0);

  // measure temp volts
  // just one shot is good enough
  thVolts = analogRead(TH_PIN);
  thVolts = thVolts / (4096.0 / 3.3);

  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r1
  tr2 = ( thVolts * tr1) / (3.3 - thVolts);

  //equation from data sheet
  TempK = 1.0 / (NTC_A + (NTC_B * (log(tr2 / 10000.0))) + (NTC_C * pow(log(tr2 / 10000.0), 2)) + (NTC_D * pow(log(tr2 / 10000.0), 3)));
  TempF = (TempK * 1.8) - 459.67;
  TempF = TempF + TempOffset;

  // compute EPower
  EPower = Volts * Amps;

  // compute Energy
  Energy = Energy + (EPower * (Update / 3600000.0));


  //Serial.print("LoadCellCal "); Serial.println(LoadCellCal );
  //Serial.print("LoadCellOffset "); Serial.println(LoadCellOffset );

  scale.set_scale(LoadCellCal);
  scale.set_offset(LoadCellOffset);

  LoadMass = scale.get_units(LoadCellReads); // 1 is still pretty good

  LoadMass = LoadMass / 1000.0;
  LoadForce = LoadMass * 9.80665; // in kg
  Torque = LoadForce * ForceArm; // kg x Nm
  MPower = (Torque * WRPM * 2.0 * 3.14159 ) / 60.0;

  // compute efficiency
  MotorEff = MPower / EPower;

}

void PrepTest() {

  Display.fillScreen(C_BLACK);

  // clear array
  for (i = LIMIT_L; i < (LIMIT_H - LIMIT_L); i ++) {
    dAmps[i] = 0.0 ;
    dTorque[i] = 0.0 ;
    dEffeciency[i] = 0.0 ;
    dVolts[i] = 0.0 ;
    dLoad[i] = 0.0 ;
  }

  // reset conditions
  PreviousRPM = 15000;

  // reset the starting point
  // needed only for restarts
  Graph.resetStart(pEff);
  Graph.resetStart(pTorque);
  Graph.resetStart(pAmps);
  Graph.resetStart(pVolts);

  Graph.setLineThickness(pEff, LineThickness);
  Graph.setLineThickness(pTorque, LineThickness);
  Graph.setLineThickness(pAmps, LineThickness);
  Graph.setLineThickness(pVolts, LineThickness);

  Graph.drawGraph();

  Display.fillRect(0, 0, 319, 40, C_DKBLUE);
  Display.setFont(F_A20);

  Display.setCursor(10 , 10 );
  Display.setTextColor(C_WHITE, C_DKBLUE);

  for (i = 0; i < 10; i++) {
    Display.print(FileName[i]);
  }

  StopBtn.disable();
  ReRunBtn.disable();
  StartTestBtn.enable();
  StartTestBtn.draw();
  UpdateTime = 0;


}

void DisplayData() {

  int Col = 282;
  int TextOffset = 12;
  // Display.fillRect(Col - 5,  50, 55 , 250, C_BLACK);
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A08);

  // point
  Display.setCursor(Col, 50);
  Display.print(F("Point: "));
  Display.setFont(F_A08);
  Display.setCursor(Col + 5, 50 + TextOffset);
  ffPoint.setTextColor(C_WHITE, C_BLACK);
  ffPoint.print(Point);

  // torque
  Display.setCursor(Col, 75);
  Display.print(F("Torque: "));
  Display.setCursor(Col + 5, 75 + TextOffset);
  ffTorque.setTextColor(C_WHITE, C_BLACK);
  ffTorque.print(Torque, 2);

  // temp
  Display.setCursor(Col, 100);
  Display.print(F("Temp: "));
  Display.setCursor(Col + 5, 100 + TextOffset);
  ffTemp.setTextColor(C_WHITE, C_BLACK);
  ffTemp.print(TempF, 1);

  // Volts
  Display.setCursor(Col, 125);
  Display.print(F("Volts: "));
  Display.setCursor(Col + 5, 125 + TextOffset);
  ffVolts.setTextColor(C_WHITE, C_BLACK);
  ffVolts.print(Volts, 1);

  // Amps
  Display.setCursor(Col, 150);
  Display.print(F("Amps: "));
  Display.setCursor(Col + 5, 150 + TextOffset);
  ffAmps.setTextColor(C_WHITE, C_BLACK);
  ffAmps.print(Amps, 1);

  // RPM
  Display.setCursor(Col, 175);
  Display.print(F("RPM: "));
  Display.setCursor(Col + 5, 175 + TextOffset);
  ffRPM.setTextColor(C_WHITE, C_BLACK);
  ffRPM.print(WRPM);

  // mass
  Display.setCursor(Col, 200);
  Display.print(F("Mass: "));
  Display.setCursor(Col + 5, 200 + TextOffset);
  ffMass.setTextColor(C_WHITE, C_BLACK);
  ffMass.print(LoadMass, 3);

}

void GetParameters() {

  // use this to force default values for new chip
  if (0 == 1) {
    EEPROM.put(10, Update);
    EEPROM.put(20, LoadCellReads);
    EEPROM.put(30, RampDownOnly);
    EEPROM.put(40, TempOffset);
    EEPROM.put(50, MotorNumber);
    EEPROM.put(60, MotorType);
    EEPROM.put(70, LineThickness);
    EEPROM.put(90, VMOffset);
    EEPROM.put(100, MotorDirection);
    EEPROM.put(110, VDCompensation);
    EEPROM.put(120, ForceArm);
    EEPROM.put(130, LoadCellCal);
    EEPROM.put(140, LoadCellOffset);
    EEPROM.put(150, ForceArm);
    EEPROM.put(160, LoadCellCal);
    EEPROM.put(170, LoadCellOffset);
    EEPROM.put(200, RPMDebounceTime );
    EEPROM.put(220, VMid );
    EEPROM.put(230, mVPerAmp );
    EEPROM.put(240, LC_year);
    EEPROM.put(250, LC_month);
    EEPROM.put(260, LC_day);
    EEPROM.put(270, SC_year);
    EEPROM.put(280, SC_month);
    EEPROM.put(290, SC_day);
    EEPROM.put(300, LC_TestsSinceCal);
    EEPROM.put(310, SC_TestsSinceCal);

  }

  EEPROM.get(10, Update);
  EEPROM.get(20, LoadCellReads);
  EEPROM.get(30, RampDownOnly);
  EEPROM.get(40, TempOffset);
  EEPROM.get(50, MotorNumber);
  EEPROM.get(60, MotorType);
  EEPROM.get(70, LineThickness);
  EEPROM.get(90, VMOffset);
  EEPROM.get(100, MotorDirection);
  EEPROM.get(110, VDCompensation);

  if (MotorDirection == LEFT) {
    EEPROM.get(120, ForceArm);
    EEPROM.get(130, LoadCellCal);
    EEPROM.get(140, LoadCellOffset);
  }

  if (MotorDirection == RIGHT) {
    EEPROM.get(150, ForceArm);
    EEPROM.get(160, LoadCellCal);
    EEPROM.get(170, LoadCellOffset);
  }

  EEPROM.get(200, RPMDebounceTime );
  EEPROM.get(220, VMid );
  EEPROM.get(230, mVPerAmp );

  EEPROM.get(240, LC_year);
  EEPROM.get(250, LC_month);
  EEPROM.get(260, LC_day);
  EEPROM.get(270, SC_year);
  EEPROM.get(280, SC_month);
  EEPROM.get(290, SC_day);

  EEPROM.get(300, LC_TestsSinceCal);
  EEPROM.get(310, SC_TestsSinceCal);

#ifdef debug
  Serial.print("Update "); Serial.println(Update);
  Serial.print("LoadCellReads "); Serial.println(LoadCellReads);

  Serial.print("ForceArm "); Serial.println(ForceArm);
  Serial.print("LoadCellCal "); Serial.println(LoadCellCal);
  Serial.print("LoadCellOffset "); Serial.println(LoadCellOffset);

  Serial.print("TempOffset "); Serial.println(TempOffset);
  Serial.print("MotorNumber "); Serial.println(MotorNumber);
  Serial.print("MotorType "); Serial.println(MotorType);
  Serial.print("MotorDir "); Serial.println(MotorDirection);

  Serial.print("VMOffset "); Serial.println(VMOffset);
  Serial.print("VDCompensation "); Serial.println(VDCompensation);
  Serial.print("RPMDebounceTime "); Serial.println(RPMDebounceTime);
  Serial.print("VMid "); Serial.println(VMid);
  Serial.print("mVPerAmp "); Serial.println(mVPerAmp);
  Serial.print("Line Thickness "); Serial.println(LineThickness);
  Serial.print("LC_year "); Serial.println(LC_year);
  Serial.print("LC_month "); Serial.println(LC_month);
  Serial.print("LC_day "); Serial.println(LC_day);
  Serial.print("SC_year "); Serial.println(SC_year);
  Serial.print("SC_month "); Serial.println(SC_month);
  Serial.print("SC_day "); Serial.println(SC_day);

  Serial.print("LC_TestsSinceCal "); Serial.println(LC_TestsSinceCal);
  Serial.print("SC_TestsSinceCal "); Serial.println(SC_TestsSinceCal);


#endif

  delay(10);

}

void ShowInstructions() {

  bool KeepIn = true;
  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE);
  Display.setFont(F_A20);
  Display.setCursor(5 , 13 );
  Display.print(F("CHECK LIST"));

  Display.setFont(F_A16);

  Display.setTextColor(C_WHITE, C_BLACK );

  Display.setCursor(5, 65);
  Display.print(F("1. Is the left brake clear?"));
  Display.setCursor(5, 95);
  Display.print(F("2. Is the right brake clear?"));
  Display.setCursor(5, 125);
  Display.print(F("3. Is the load sensor calibrated?"));
  Display.setCursor(5, 155);
  Display.print(F("4. Is the rear cover secure?"));
  Display.setCursor(5, 185);
  Display.print(F("5. Is the disk secure to the hub?"));
  Display.setCursor(5, 215);
  Display.print(F("6. Is the load cell connected?"));

  MenuDoneBtn.draw();

  while (KeepIn) {

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(MenuDoneBtn)) {
        KeepIn = false;
      }
    }
  }
  delay(100);

}

void GetDirection() {

  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE);
  Display.setFont(F_A20);
  Display.setCursor(5 , 13 );
  Display.print(F("SET ROTATION"));
  MenuDoneBtn.draw();
  bool KeepIn = true;

  if (MotorDirection == LEFT) {
    drawBitmap(10, 85, CCW_ARROW, 130, 130, C_GREEN);
    drawBitmap(180, 85, CW_ARROW, 130, 130, C_GREY);
  }

  else if (MotorDirection == RIGHT) {
    drawBitmap(10, 85, CCW_ARROW, 130, 130, C_GREY);
    drawBitmap(180, 85, CW_ARROW, 130, 130, C_GREEN);
  }

  while (KeepIn) {

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(MenuDoneBtn)) {
        KeepIn = false;
      }

      else if ((BtnX > 10) && (BtnX < 160) &&  (BtnY > 80)) {
        Click();
        drawBitmap(10, 85, CCW_ARROW, 130, 130, C_GREEN);
        drawBitmap(180, 85, CW_ARROW, 130, 130, C_GREY);
        delay(50);
        MotorDirection = LEFT;
        while (Touch.dataAvailable()) {
          drawBitmap(10, 85, CCW_ARROW, 130, 130, C_DKGREEN);
        }
        drawBitmap(10, 85, CCW_ARROW, 130, 130, C_GREEN);
      }
      else if ((BtnX > 160) && (BtnX < 320) && (BtnY > 80)) {
        Click();
        drawBitmap(10, 85, CCW_ARROW, 130, 130, C_GREY);
        drawBitmap(180, 85, CW_ARROW, 130, 130, C_GREEN);
        delay(50);
        MotorDirection = RIGHT;
        while (Touch.dataAvailable()) {
          drawBitmap(180, 85, CW_ARROW, 130, 130, C_DKGREEN);
        }
        drawBitmap(180, 85, CW_ARROW, 130, 130, C_GREEN);
      }

    }

  }

  EEPROM.put(100, MotorDirection);

  if (MotorDirection == LEFT) {
    EEPROM.get(120, ForceArm);
    EEPROM.get(130, LoadCellCal);
    EEPROM.get(140, LoadCellOffset);
  }

  if (MotorDirection == RIGHT) {
    EEPROM.get(150, ForceArm);
    EEPROM.get(160, LoadCellCal);
    EEPROM.get(170, LoadCellOffset);
  }

}
void MainMenu() {

  bool KeepIn = true;

  DrawMainMenuScreen();

  while (KeepIn) {

    // if touch screen pressed handle it

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(ProfileBtn) == true) {
        KeepIn = false;
      }
      if (PressIt(SensorsBtn) == true) {
        CalibrateSensors();
        DrawMainMenuScreen();
      }
      if (PressIt(ConnectionBtn) == true) {
        Connections();
        DrawMainMenuScreen();
      }
    }
  }
}

/*

  service function UI screen

*/
void DrawMainMenuScreen() {

  //nothing fancy, just a header and some buttons
  Display.fillScreen(C_BLACK);


  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE);
  Display.setFont(F_A20);
  Display.setCursor(5 , 13 );
  Display.print(F("DYNAMOMETER"));

  ProfileBtn.draw();
  SensorsBtn.draw();
  ConnectionBtn.draw();

}

void CalibrateSensors() {

  bool LCalDone = false;
  bool SCalDone = false;
  int EditMenuOption = 1;

  CalTime = 0;

  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 200, 320, 40, C_LTGREY);
  // draw headers
  Display.setTextColor(C_BLACK, C_LTGREY );
  Display.setFont(F_A12B);
  
  Display.setCursor(5, 205 );
  Display.print(F("Volts: "));

  Display.setCursor(70, 205 );
  Display.print(F("Amps: "));

  Display.setCursor(135, 205 );
  Display.print(F("Temp: "));

  Display.setCursor(200, 205 );
  Display.print(F("RPM: "));

  Display.setCursor(265, 205 );
  Display.print(F("Load: "));

  Counter = 0;
  aVolts = 0.0;
  aVolts = analogRead(AM_PIN);
  Counter++;
  ComputeData();

  sprintf(buf, "Amp offset (%.3f)", aVolts);
  OptionMenu.setItemText(MenuOption9, buf);
  OptionMenu.draw();

  while (EditMenuOption != 0) {

    vVolts = vVolts + analogRead(VM_PIN);
    aVolts = aVolts + analogRead(AM_PIN);
    Counter++;

    if (CalTime > 1000) {

      Display.setFont(F_A12);

      if (PulseCount > 4) {
        PulseTime = (PulseEndTime - PulseStartTime) / (PulseCount - 1.0);
      }

      if (PulseTime > 0) {
        WRPM = (unsigned long) (1000.0 * 60.0) /  (PulseTime );
      }

      ComputeData();

      Display.setFont(F_A12);

      Display.setCursor(5, 225 );
      ffVolts.setTextColor(C_BLACK, C_LTGREY );
      ffVolts.print(Volts, 2);

      Display.setCursor(70, 225 );
      ffAmps.setTextColor(C_BLACK, C_LTGREY );
      ffAmps.print(Amps, 2);

      Display.setCursor(135, 225 );
      ffTemp.setTextColor(C_BLACK, C_LTGREY );
      ffTemp.print(TempF, 1);

      Display.setCursor(200, 225 );
      ffRPM.setTextColor(C_BLACK, C_LTGREY );
      ffRPM.print(WRPM);

      Display.setCursor(265, 225 );
      ffCalLoad.setTextColor(C_BLACK, C_LTGREY);
      ffCalLoad.print(LoadMass * 1000.0, 1);

      vVolts = 0.0;
      aVolts = 0.0;
      thVolts = 0.0;
      Counter = 0;
      WRPM = 0;
      PulseTime = 0;
      PulseCount = 0;
      ErrorCount = 0;
      CalTime = 0;
    }

    if (Touch.dataAvailable()) {
      Click();
      ProcessTouch();

      EditMenuOption = OptionMenu.press(BtnX, BtnY);

      OptionMenu.drawRow(EditMenuOption);

      if (EditMenuOption == MenuOption2) {
        LCalDone = true;
        scale.tare();
        delay(100);
        LoadCellCal = scale.get_scale();
        LoadCellOffset = scale.get_offset();
        delay(100);
      }
      if (EditMenuOption == MenuOption3 ) {
        LCalDone = true;
        scale.calibrate_scale(1000, 5);
        delay(100);
        LoadCellCal = scale.get_scale();
        LoadCellOffset = scale.get_offset();
        delay(100);
      }
      if (EditMenuOption == MenuOption1) {
        ForceArm = OptionMenu.value[MenuOption1];
        LCalDone = true;
      }
    }
    if ( (EditMenuOption == MenuOption7) ||
         (EditMenuOption == MenuOption8) ||
         (EditMenuOption == MenuOption9) ||
         (EditMenuOption == MenuOption10) ) {
      VDCompensation = OptionMenu.value[MenuOption8];
      VMid =  OptionMenu.value[MenuOption9];
      mVPerAmp =  OptionMenu.value[MenuOption10];
      VMOffset = OptionMenu.value[MenuOption7];
      SCalDone = true;
    }
  }

  Update = (int) OptionMenu.value[MenuOption5];
  LoadCellReads = (int) OptionMenu.value[MenuOption4];
  RampDownOnly = (int) OptionMenu.value[MenuOption12];
  LineThickness = (int) OptionMenu.value[MenuOption13];
  TempOffset = OptionMenu.value[MenuOption11];
  VDCompensation = OptionMenu.value[MenuOption8];
  ForceArm = OptionMenu.value[MenuOption1];
  RPMDebounceTime = (int) OptionMenu.value[MenuOption6];
  VMid =  OptionMenu.value[MenuOption9];
  mVPerAmp =  OptionMenu.value[MenuOption10];
  VMOffset = OptionMenu.value[MenuOption7];



  EEPROM.put(10, Update );
  EEPROM.put(20, LoadCellReads );
  EEPROM.put(30, RampDownOnly);
  EEPROM.put(40, TempOffset );
  EEPROM.put(70, LineThickness );
  EEPROM.put(90, VMOffset );
  EEPROM.put(110, VDCompensation);
  if (MotorDirection == LEFT) {
    EEPROM.put(120, ForceArm);
    EEPROM.put(130, LoadCellCal);
    EEPROM.put(140, LoadCellOffset);
  }
  if (MotorDirection == RIGHT) {
    EEPROM.put(150, ForceArm);
    EEPROM.put(160, LoadCellCal);
    EEPROM.put(170, LoadCellOffset);
  }
  EEPROM.put(200, RPMDebounceTime);
  EEPROM.put(220, VMid);
  EEPROM.put(230, mVPerAmp);
  if (LCalDone) {
    EEPROM.put(240, year());
    EEPROM.put(250, month());
    EEPROM.put(260, day());
    EEPROM.put(300, 0); // SC_TestsSinceLCCal
  }
  if (SCalDone) {
    EEPROM.put(270, year());
    EEPROM.put(280, month());
    EEPROM.put(290, day());
    EEPROM.put(310, 0); // SC_TestsSinceLCCal
  }

  delay(10);

}

void  Connections() {

  bool KeepIn = true;
  Display.fillScreen(C_BLACK);

  Display.fillRect(0, 0, 320, 50, C_DKBLUE);

  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(5, 13);
  Display.print(F("CONNECTIONS"));

  Display.setFont(F_A16);
  Display.setTextColor(C_WHITE);

  Display.drawRect(20, 70, 280, 160, C_WHITE); // outer case

  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A20);
  Display.setCursor(145, 180);
  Display.print(F("TOP"));

  Display.drawRect(30, 90, 90, 70, C_WHITE); // left USB
  Display.drawLine(30, 125, 118, 125, C_WHITE); // left USB

  Display.drawRect(180, 90, 90, 70, C_WHITE); // righ USB
  Display.drawLine(180, 125, 268, 125, C_WHITE); // left USB

  Display.setTextColor(C_CYAN);
  Display.setCursor(40, 170); Display.print(F("Upper"));
  Display.setTextColor(C_GREEN);
  Display.setCursor(40, 195); Display.print(F("Lower"));

  Display.setTextColor(C_CYAN);
  Display.setCursor(40, 98); Display.print(F("Temp"));
  Display.setTextColor(C_GREEN);
  Display.setCursor(40, 130); Display.print(F("Amp"));

  Display.setTextColor(C_CYAN);
  Display.setCursor(188, 98); Display.print(F("Load"));
  Display.setTextColor(C_GREEN);
  Display.setCursor(188, 130); Display.print(F("Speed"));

  MenuDoneBtn.draw();

  while (KeepIn) {

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(MenuDoneBtn)) {
        KeepIn = false;
      }
    }
  }
}

unsigned int Debounce(int pin, unsigned long & dtime) {

  if (digitalRead(pin) == LOW) {
    delay(10);
    if ((millis() - dtime) > DEBOUNCE) {
      // button now debounced, long press or short one

      dtime = millis();
      while (digitalRead(pin) == LOW) {

        if ((millis() - dtime) > LONG_PRESS) {
          dtime = millis();
          return LONG_PRESS;
        }
      }
      dtime = millis();
      return SHORT_PRESS;

    }
  }
  return NO_PRESS;
}

void ISR_MS() {

  if (digitalRead(RPM_PIN) == LOW) {
    PulseCurrentTime = millis();

    if ((PulseCurrentTime - PulseDebounce) > (unsigned long) RPMDebounceTime) {
      PulseDebounce = PulseCurrentTime;
      PulseCount ++;
      if (PulseCount == 1.0) {
        PulseStartTime = PulseCurrentTime;
      }
      else {
        PulseEndTime = PulseCurrentTime;
      }
    }
    ErrorCount++;
  }
}

void EndTest() {

  // assume test was completed so bump the counters

  LC_TestsSinceCal++;
  SC_TestsSinceCal++;

  EEPROM.put(300, LC_TestsSinceCal);
  EEPROM.put(310, SC_TestsSinceCal);
  delay(100);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(10 , 10 );
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Saving data..."));

  // update filename with latest temp
  FileName[4] =  ((int) (TempF / 100)) + '0';
  FileName[5] =  ((int) (TempF / 10) % 10) + '0';
  FileName[6] =  ((int) (TempF - (int)(TempF / 10) * 10)) + '0';

  StartTest = false;

  isSD = DataFile.open(FileName,  O_WRITE | O_CREAT);

  if (!isSD) {

    Display.setFont(F_A20);
    Display.setCursor(10 , 10 );
    Display.setTextColor(C_WHITE, C_DKBLUE);
    Display.fillRect(0, 0, 320, 40, C_DKBLUE);
    Display.print(F("Saving failed..."));

    isSD = sd.begin(SDCS_PIN, SD_SCK_MHZ(20));
    isSD = DataFile.open(FileName,  O_WRITE | O_CREAT);
    if (!isSD) {
      Display.setFont(F_A20);
      Display.setCursor(10 , 10 );
      Display.setTextColor(C_WHITE, C_DKBLUE);
      Display.fillRect(0, 0, 320, 40, C_DKBLUE);
      Display.print(F("Retry failed..."));
    }

  }
  // write date info to file
  DataFile.timestamp(T_ACCESS, year(), month(), day(), hour(), minute(), second());
  DataFile.timestamp(T_CREATE, year(), month(), day(), hour(), minute(), second());
  DataFile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());

  // print header
  DataFile.print(F("RPM"));
  DataFile.write(44);

  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(F("_"));
  DataFile.print(FileName[4]);
  DataFile.print(FileName[5]);
  DataFile.print(FileName[6]);
  DataFile.print(F("_"));
  DataFile.print(F("Amps"));
  DataFile.write(44);

  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(F("_"));
  DataFile.print(FileName[4]);
  DataFile.print(FileName[5]);
  DataFile.print(FileName[6]);
  DataFile.print(F("_"));
  DataFile.print(F("Torque"));
  DataFile.write(44);

  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(F("_"));
  DataFile.print(FileName[4]);
  DataFile.print(FileName[5]);
  DataFile.print(FileName[6]);
  DataFile.print(F("_"));
  DataFile.print(F("Efficiency"));
  DataFile.write(44);

  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(F("_"));
  DataFile.print(FileName[4]);
  DataFile.print(FileName[5]);
  DataFile.print(FileName[6]);
  DataFile.print(F("_"));
  DataFile.print(F("Volts"));
  DataFile.write(44);

  DataFile.print(FileName[0]);
  DataFile.print(FileName[1]);
  DataFile.print(FileName[2]);
  DataFile.print(F("_"));
  DataFile.print(FileName[4]);
  DataFile.print(FileName[5]);
  DataFile.print(FileName[6]);
  DataFile.print(F("_"));
  DataFile.print(F("Load"));

  DataFile.write(44);
  DataFile.write(44);
  DataFile.print(F("PATRIOT RACING - DYNAMOMETER TEST"));

  DataFile.println(F(""));

  // dump stuff from array to SD card

  DataFile.print(LIMIT_L); DataFile.write(44);
  DataFile.print(LastAmps, 3); DataFile.write(44);
  DataFile.print(LastTorque, 4); DataFile.write(44);
  DataFile.print(LastEffeciency, 2); DataFile.write(44);
  DataFile.print(LastVolts, 2); DataFile.write(44);
  DataFile.println(LastLoad, 3);

  for (i = 0; i < (LIMIT_H - LIMIT_L); i++) {

    DataFile.print(i + LIMIT_L + 1); // this is RPM, add 1 to get the upper limit
    DataFile.write(44);

    if (dAmps[i] == 0) {
      DataFile.print(F(""));
    }
    else {
      DataFile.print(dAmps[i], 3);
    }
    DataFile.write(44);

    if (dTorque[i] == 0) {
      DataFile.print(F(""));
    }
    else {
      DataFile.print(dTorque[i], 4);
    }
    DataFile.write(44);

    if (dEffeciency[i] == 0) {
      DataFile.print(F(""));
    }
    else {
      DataFile.print(dEffeciency[i], 2);
    }
    DataFile.write(44);

    if (dVolts[i] == 0) {
      DataFile.print(F(""));
    }
    else {
      DataFile.print(dVolts[i], 2);
    }
    DataFile.write(44);

    if (dLoad[i] == 0) {
      DataFile.print(F(""));
    }
    else {
      DataFile.print(dLoad[i], 3);
    }

    if (i < 15) {

      DataFile.write(44);
      DataFile.write(44);
      switch (i) {

        case 0:
          DataFile.print(F("Load sensor calibration date: "));
          DataFile.write(44);
          DataFile.print(LC_month); DataFile.write(47); DataFile.print(LC_day); DataFile.write(47); DataFile.print(LC_year);
          DataFile.write(44);
          DataFile.print(F("Tests since load cell calibration: "));
          DataFile.write(44);
          DataFile.print(LC_TestsSinceCal);
          break;

        case 1:
          DataFile.print(F("Sensor calibration date: "));
          DataFile.write(44);
          DataFile.print(SC_month); DataFile.write(47); DataFile.print(SC_day); DataFile.write(47); DataFile.print(SC_year);
          DataFile.write(44);
          DataFile.print(F("Tests since sensor calibration: "));
          DataFile.write(44);
          DataFile.print(SC_TestsSinceCal);
          break;

        case 2:
          DataFile.print(F("Code v.: "));
          DataFile.write(44);
          DataFile.print(CODE_VERSION);
          DataFile.write(44);
          DataFile.print(F("Date: "));
          DataFile.write(44);
          DataFile.print(month());
          DataFile.print(F(" / "));
          DataFile.print(day());
          DataFile.print(F(" / "));
          DataFile.print(year());
          DataFile.write(44);
          DataFile.print(hour());
          DataFile.print(F(": "));
          if (minute() < 10) {
            DataFile.print(F("0"));
          }
          DataFile.print(minute());
          break;

        case 3:
          DataFile.print(F("Motor MFG: "));
          DataFile.write(44);
          if (FileName[0] == 'U') {
            DataFile.print(F("UK"));
          }
          else {
            DataFile.print(F("Chinese"));
          }

          DataFile.write(44);
          DataFile.print(F("Rotation: "));
          if (MotorDirection == LEFT) {
            DataFile.print(F("Left (CCW)"));
          }
          else {
            DataFile.print(F("Right (CW)"));
          }
          DataFile.write(44);
          DataFile.print(F("Motor number: "));
          DataFile.write(44);
          DataFile.print(FileName[1]) ;
          DataFile.write(44);
          DataFile.print(F("Motor Temp [deg F]: "));
          DataFile.write(44);
          DataFile.print(TempF, 1);
          break;
        case 4:
          DataFile.print(F("Update time [ms]: "));
          DataFile.write(44);
          DataFile.print(Update);
          break;
        case 5:
          DataFile.print(F("Load cell averages: "));
          DataFile.write(44);
          DataFile.print(LoadCellReads);
          break;
        case 6:
          DataFile.print(F("Force arm [m]: "));
          DataFile.write(44);
          DataFile.print(ForceArm, 4);
          break;
        case 7:
          DataFile.print(F("Load cell cal: "));
          DataFile.write(44);
          DataFile.print(LoadCellCal);
          DataFile.write(44);
          DataFile.print(F("Load cell offset: "));
          DataFile.write(44);
          DataFile.print(LoadCellOffset);
          break;
        case 8:
          DataFile.print(F("Voltage sensor offset: "));
          DataFile.write(44);
          DataFile.print(VMOffset, 2);
          DataFile.print(F("Voltage divider: "));
          DataFile.write(44);
          DataFile.print(VDCompensation, 2);
          break;
        case 9:
          DataFile.print(F("Current sensor offset: "));
          DataFile.write(44);
          DataFile.print(VMid, 4);
          DataFile.write(44);
          DataFile.print(F("Current sensor mVPerAmp: "));
          DataFile.write(44);
          DataFile.print(mVPerAmp, 4);
          break;
        case 10:
          DataFile.print(F("RPM debounce [ms]: "));
          DataFile.write(44);
          DataFile.print(RPMDebounceTime);
          break;
        case 11:
          DataFile.print(F("Temp Offset: "));
          DataFile.write(44);
          DataFile.print(TempOffset);
          break;
        case 12:
          DataFile.print(F("Data points: "));
          DataFile.write(44);
          DataFile.print(Point);
          DataFile.write(44);
          DataFile.print(F("Record Type: "));
          DataFile.write(44);
          if (RampDownOnly) {
            DataFile.print(F("Record only on decrease of RPM"));
          }
          else {
            DataFile.print(F("Record regardless of RPM"));
          }
          break;
        case 13:
          DataFile.print(F("Line thickness: "));
          DataFile.write(44);
          DataFile.print(LineThickness);

          break;
      }

    }

    DataFile.println("");
  }

  delay(20);
  DataFile.close();
  delay(200);

  // "MND_TTT_RR.CSV"

  FileName[11] =  'b';
  FileName[12] =  'm';
  FileName[13] =  'p';

  Display.setFont(F_A20);
  Display.setCursor(10 , 10 );
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Motor: "));
  for (i = 0; i < 10; i++) {
    Display.print(FileName[i]);
  }

  SaveBMP24(&Display, SDCS_PIN , FileName);

  Display.setFont(F_A20);

  Display.setFont(F_A20);
  Display.setCursor(10 , 10 );
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.fillRect(0, 0, 320, 40, C_DKBLUE);
  Display.print(F("Test Complete"));

}

void ProcessTouch() {

  Touch.read();


  BtnX = Touch.getX();
  BtnY = Touch.getY();

#ifdef DEBUG
  Serial.print("real coordinates: ");
  Serial.print(BtnX);
  Serial.print(", ");
  Serial.print (BtnY);
  //Display.drawPixel(BtnX, BtnY, C_GREEN);
#endif

  BtnX  = map(BtnX,  0, 235, 0, 320);
  BtnY  = map(BtnY, 25, 380, 0, 240);


#ifdef DEBUG
  Serial.print(", Mapped coordinates: ");
  Serial.print(BtnX);
  Serial.print(", ");
  Serial.println(BtnY);
  Display.drawPixel(BtnX, BtnY, C_RED);
#endif
  delay(20);



}

bool PressIt(Button TheButton) {

  if (TheButton.press(BtnX, BtnY)) {
    Click();
    TheButton.draw(B_PRESSED);
    while (Touch.dataAvailable()) {
      if (TheButton.press(BtnX, BtnY)) {
        TheButton.draw(B_PRESSED);
      }
      else {
        TheButton.draw(B_RELEASED);
        return false;
      }
      ProcessTouch();
    }

    TheButton.draw(B_RELEASED);
    return true;
  }
  return false;

}

void GetFileName() {

  bool FNKeepIn = true;
  bool Redraw = false;

  DrawFileNameScreen();

  vVolts = 0.0;
  aVolts = 0.0;
  thVolts = 0.0;
  Counter = 0;
  WRPM = 0;
  PulseTime = 0;
  PulseCount = 0;
  Volts = 0.0, Amps = 0.0; WRPM = 0;
  FileCount = 0;

  Root.open("/");

  while (FileFile.openNext(&Root, O_RDONLY)) {
    FileCount++;

    FileFile.close();
  }

  Root.close();
  delay(10);
  Root.close();

  if (MotorType == CHINESE_MOTOR) {
    FileName[0] =  'C';
  }
  else {
    FileName[0] =  'U';
  }

  FileName[1] =  MotorNumber + '0';


  if (MotorDirection == LEFT) {
    FileName[2] =  'L';
  }
  else {
    FileName[2] =  'R';
  }

  FileName[8] =  (int) ((FileCount / 10) % 10) + '0';
  FileName[9] =  (int) (FileCount  % 10) + '0';
  FileName[11] =  'c';
  FileName[12] =  's';
  FileName[13] =  'v';

  while (FNKeepIn) {

    curtime = millis() ;
    vVolts =  vVolts + (analogRead(VM_PIN));
    aVolts =  aVolts + (analogRead(AM_PIN)) ;
    Counter++;

    if (UpdateTime >= 1000 ) {

      ComputeData();
      Display.setFont(F_A16);

      Display.setCursor(20, 105);
      ffTemp.setTextColor(C_WHITE, C_BLACK);
      ffTemp.print((int) TempF);

      Display.setCursor(20, 155);
      if (abs(LoadMass > 0.05)) {
        ffMass.setTextColor(C_RED, C_BLACK);
      }
      else {
        ffMass.setTextColor(C_WHITE, C_BLACK);
      }
      ffMass.print(LoadMass, 2);

      FileName[4] =  ((int) (TempF / 100)) + '0';
      FileName[5] =  ((int) (TempF / 10) % 10) + '0';
      FileName[6] =  ((int) (TempF - (int)(TempF / 10) * 10)) + '0';

      ffFileName.setTextColor(C_WHITE, C_DKBLUE);
      Display.setFont(F_A20);
      Display.setCursor(90 , 13);
      ffFileName.print(FileName);


      // reset the counters
      vVolts = 0.0;
      aVolts = 0.0;
      thVolts = 0.0;
      Counter = 0;
      UpdateTime = 0;
      WRPM = 0;
      PulseTime = 0;
      PulseCount = 0;

    }

    if (Touch.dataAvailable()) {

      ProcessTouch();

      if (PressIt(StartBtn) == true) {
        FNKeepIn = false;
      }

      if (PressIt(SetTareBtn) == true) {
        scale.tare();
      }

      if (PressIt(MotorNumberBtn) == true) {

        KeyPad(MotorNumber, 1, 9);
        Redraw = true;
      }

      if (PressIt(MotorTypeBtn) == true) {
        if (FileName[0] ==  'U') {
          FileName[0] =  'C';
          MotorType = 1;
        }
        else {
          FileName[0] =  'U';
          MotorType = 0;
        }

        Redraw = true;
      }

      FileName[1] =  MotorNumber + '0';

      if (Redraw) {
        Redraw = false;
        DrawFileNameScreen();

      }
    }
  }

  EEPROM.put(50, MotorNumber);
  EEPROM.put(60, MotorType);

}

/*

  service function to write the filename screen

*/

void DrawFileNameScreen() {

  Display.fillScreen(C_BLACK);
  Display.fillRect(0, 0, 320, 50, C_DKBLUE);
  Display.setTextColor(C_WHITE, C_DKBLUE);
  Display.setFont(F_A20);
  Display.setCursor(5 , 13 );
  Display.println(F("FILE: "));

  // Display data
  Display.setTextColor(C_WHITE, C_BLACK);
  Display.setFont(F_A16);
  Display.setCursor(10, 80);
  Display.print(F("Temp"));
  Display.setCursor(10, 130);
  Display.print(F("Mass"));

  // for manufacture
  Display.drawLine(103, 60, 103, 140, C_WHITE); // vert
  Display.drawLine(103, 140, 200, 140, C_WHITE); // horiz

  // for motor number
  Display.drawLine(116, 60, 116, 80, C_WHITE); // vert
  Display.drawLine(116, 80, 200, 80, C_WHITE); // horiz

  MotorTypeBtn.draw();
  MotorNumberBtn.draw();
  StartBtn.draw();
  SetTareBtn.draw();

}

void KeyPad(float & TheNumber, float TheMin, float TheMax) {

  bool KPKeepIn = true;
  bool hasDP = false;

  Left = BUTTON_X -  BUTTON_SPACING_X - (0.5 * BUTTON_W);
  top = BUTTON_Y - (2 * BUTTON_H);
  wide = (3 * BUTTON_W ) + (5 * BUTTON_SPACING_X);
  high = 6 * (BUTTON_H +  BUTTON_SPACING_Y) + 30;

  sprintf(dn, " % .*f", 3, TheNumber);

  np = strlen(dn);

  for (i = np - 1; i > 0; i--) {
    if (dn[i] == '0') {
      dn[i] = ' ';
      np = i;
      if (dn[i - 1] == '.') {
        dn[i - 1] = ' ';
        np = i - 1;
        break;
      }
    }
    else {
      break;
    }
  }
  // is there a decimal point
  for (i = 0; i < 8; i++) {
    if (dn[i] == '.') {
      hasDP = true;
      break;
    }
  }

  TempNum = TheNumber;
  KeepIn = true;

  Display.fillRect(Left, top, wide , high, C_DKGREY);
  Display.drawRect(Left, top, wide , high, C_LTGREY);

  Display.fillRect(Left + 10, top + 10, wide - 100 , 40, C_GREY);
  Display.drawRect(Left + 10, top + 10, wide - 100 , 40, C_BLACK);

  Display.setCursor(Left  + 20 , top + 20);
  Display.setTextColor(C_BLACK, C_GREY);
  Display.setFont(F_A16);
  Display.print(dn);

  for (row = 0; row < 4; row++) {
    for (col = 0; col < 3; col++) {
      KeyPadBtn[col + row * 3].draw();
    }
  }

  DoneBtn.draw();
  BackBtn.draw();

  while (KPKeepIn) {

    if (Touch.dataAvailable()) {
      Click();
      ProcessTouch();
      if (PressIt(BackBtn) == true) {
        if (np > 0) {
          if (dn[np - 1] == '.') {
            hasDP = false;
          }
          dn[--np] = ' ';
        }
      }
      if (PressIt(DoneBtn) == true) {

        TheNumber = atof(dn);

        if ((TheNumber < TheMin) || (TheNumber > TheMax)) {
          TheNumber = TempNum;
        }
        KPKeepIn = false;
      }

      // go thru all the KeyPadBtn, checking if they were pressed
      for (b = 0; b < 12; b++) {

        if (PressIt(KeyPadBtn[b]) == true) {

          if (b < 9) {
            // 1 to 9
            dn[np] = (b + 1) + '0';
            np++;
          }
          if (b == 10) {
            // 0
            dn[np] = '0';
            np++;
          }
          if (b == 11) {
            if (!hasDP) {
              dn[np++] = '.';
              hasDP = true;
            }
          }
          if (b == 9) {
            // negative
            if (dn[0] == '-') {
              for (i = 0; i < 7; i++) {
                dn[i] = dn[i + 1];
              }
            }
            else {
              for (i = 7; i > 0; i--) {
                dn[i] = dn[i - 1];
              }
              dn[0] = '-';
            }
          }
        }
      }

      Display.fillRect(Left + 10, top + 10, wide - 100 , 40, C_GREY);
      Display.drawRect(Left + 10, top + 10, wide - 100 , 40, C_BLACK);
      Display.setFont(F_A16);
      Display.setCursor(Left  + 20 , top + 20);
      Display.setTextColor(C_BLACK, C_GREY);
      Display.print(dn);

    }
  }

}


void KeyPad(int & TheNumber, int TheMin, int TheMax) {

  bool KPKeepIn = true;
  bool FirstPress = true;
  // bool hasDP;

  Left = BUTTON_X -  BUTTON_SPACING_X - (0.5 * BUTTON_W);
  top = BUTTON_Y - (2 * BUTTON_H);
  wide = (3 * BUTTON_W ) + (5 * BUTTON_SPACING_X);
  high = 6 * (BUTTON_H +  BUTTON_SPACING_Y) + 30;

  sprintf(dn, " % d", TheNumber);

  np = strlen(dn);

  TempNum = TheNumber;
  KeepIn = true;

  Display.fillRect(Left, top, wide , high, C_DKGREY);
  Display.drawRect(Left, top, wide , high, C_LTGREY);

  Display.fillRect(Left + 10, top + 10, wide - 100 , 40, C_GREY);
  Display.drawRect(Left + 10, top + 10, wide - 100 , 40, C_BLACK);

  Display.setFont(F_A16);
  Display.setCursor(Left  + 20 , top + 20);
  Display.setTextColor(C_BLACK, C_GREY);
  Display.print(dn);

  for (row = 0; row < 4; row++) {
    for (col = 0; col < 3; col++) {
      KeyPadBtn[col + row * 3].draw();
    }
  }

  KeyPadBtn[11].disable();

  DoneBtn.draw();
  BackBtn.draw();

  while (KPKeepIn) {

    if (Touch.dataAvailable()) {
      Click();
      ProcessTouch();
      if (PressIt(BackBtn) == true) {
        if (np > 0) {
          dn[--np] = ' ';
        }
      }
      if (PressIt(DoneBtn) == true) {
        TheNumber = atof(dn);
        if ((TheNumber < TheMin) || (TheNumber > TheMax)) {
          TheNumber = TempNum;
        }
        KPKeepIn = false;
      }

      // go thru all the KeyPadBtn, checking if they were pressed
      for (b = 0; b < 12; b++) {
        if (PressIt(KeyPadBtn[b]) == true) {
          if (b < 9) {
            if (FirstPress) {
              FirstPress = false;
              np = 0;
              strcpy(dn, "             ");
            }
            // 1 to 9
            dn[np] = (b + 1) + '0';
            np++;
          }
          else if (b == 10) {
            // 0
            dn[np] = '0';
            np++;
          }
          else if (b == 9) {
            // negative
            if (dn[0] == '-') {
              for (i = 0; i < 7; i++) {
                dn[i] = dn[i + 1];
              }
            }
            else {
              for (i = 7; i > 0; i--) {
                dn[i] = dn[i - 1];
              }
              dn[0] = '-';
            }
          }
        }
      }

      Display.fillRect(Left + 10, top + 10, wide - 100 , 40, C_GREY);
      Display.drawRect(Left + 10, top + 10, wide - 100 , 40, C_BLACK);
      Display.setFont(F_A16);
      Display.setCursor(Left  + 20 , top + 20);
      Display.setTextColor(C_BLACK, C_GREY);
      Display.print(dn);
    }
  }

}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void drawBitmap(int16_t x, int16_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, uint16_t color) {

  uint8_t sbyte = 0;
  uint8_t byteWidth = 0;
  int jj, ii;

  byteWidth = (w + 7) / 8;

  for (jj = 0; jj < h; jj++) {
    for (ii = 0; ii < w; ii++) {
      if (ii & 7)  sbyte <<= 1;
      else sbyte   = pgm_read_byte(bitmap + jj * byteWidth + ii / 8);
      if (sbyte & 0x80) Display.drawPixel(x + ii, y + jj, color);
    }
  }

}

void Click() {

  analogWriteFrequency(BUZZ_PIN, 700);
  analogWrite(BUZZ_PIN, 127);
  delay(5);
  analogWrite(BUZZ_PIN, 0);

  // fire up the touch display
  Touch.InitTouch(PORTRAIT);
  Touch.setPrecision(PREC_EXTREME);

}

///////////////////////////////////////////////////
